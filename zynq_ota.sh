#!/usr/bin/env bash
# =============================================================================
#  zynq_ota.sh  --  Generation + deploiement OTA du firmware Zynq (Opossum)
# =============================================================================
#
#  Deux roles, utilisables separement :
#
#    build : genere BOOT.bin (bootgen) a partir du FSBL + bitstream + les 2 ELF
#            (CPU0 = opossum_core1, CPU1 = opossum_core2). A lancer sur une
#            machine ou les outils Xilinx sont dans le PATH (source .../settings64.sh).
#
#    send  : envoie un BOOT.bin deja genere vers le Zynq par liaison serie
#            (UART, ou le pont USB-serie de la carte), puis declenche un REBOOT.
#            Ne depend d'AUCUN outil Xilinx -> c'est ce role que le Raspberry Pi
#            jouera a terme (python3 + pyserial suffisent).
#
#    all   : build puis send.
#    jtag  : bonus, flash QSPI par JTAG (program_flash) si Vitis dispo.
#
#  ---------------------------------------------------------------------------
#  PROTOCOLE D'UPDATE SERIE (host <-> firmware) -- role "send"
#  ---------------------------------------------------------------------------
#  Transporte sur la MEME UART que l'interpreteur de commandes (UART_COMM).
#  Le firmware doit implementer une commande "FWUPDATE" (a ajouter dans
#  command_list.c) qui bascule l'UART en reception binaire brute (SANS echo,
#  SANS log) le temps du transfert. Sequence :
#
#    1. host  -> zynq : "FWUPDATE <taille_dec> <crc32_dec>\r"   (ligne ASCII)
#    2. zynq  -> host : "+READY <bloc>\n"    (bloc = taille max d'un bloc, ex 4096)
#                       ou "-ERR <raison>\n" (taille trop grande, QSPI KO, ...)
#    3. Pour chaque bloc de <bloc> octets (le dernier peut etre plus court) :
#         host -> zynq : <octets binaires du bloc>
#         zynq -> host : "+ACK <total_recu>\n"   -> bloc suivant
#                        "-NAK <total_recu>\n"   -> le host renvoie le meme bloc
#       Le firmware ecrit chaque bloc en QSPI a la volee (offset 0).
#    4. Apres le dernier bloc, le firmware relit/verifie le CRC32 global :
#         zynq -> host : "+DONE\n"   (image ecrite et verifiee)
#                        "-ERR <raison>\n"
#    5. host  -> zynq : "REBOOT\r"  (commande deja existante) -> boot QSPI.
#
#  Flow-control par ACK bloc-a-bloc : indispensable a 115200 bauds avec le
#  petit ring-buffer RX du bare-metal. CRC32 = zlib.crc32 (polynome standard),
#  transmis en DECIMAL (parse par Get_Param_u32 cote firmware).
#
#  NOTE : le cote firmware est implemente dans opossum_common/APP_FWUPDATE/
#  (commande FWUPDATE + qspi_flash via XQspiPs), enregistre dans command_list.c
#  (CPU0). Rebuild opossum_core1 dans Vitis pour l'activer.
# =============================================================================

set -euo pipefail

# --- Racine repo = dossier du script (les chemins par defaut en dependent) ---
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# --- Valeurs par defaut (surchargeables par options ou variables d'env) ------
FSBL="${FSBL:-$ROOT/Zynq_block_design_wrapper/export/Zynq_block_design_wrapper/sw/Zynq_block_design_wrapper/boot/fsbl.elf}"
BIT="${BIT:-$ROOT/opossum_core1/_ide/bitstream/Zynq_block_design_wrapper.bit}"
ELF0="${ELF0:-$ROOT/opossum_core1/Debug/opossum_core1.elf}"   # CPU0
ELF1="${ELF1:-$ROOT/opossum_core2/Debug/opossum_core2.elf}"   # CPU1
BOOTBIN="${BOOTBIN:-$ROOT/BOOT.bin}"

PORT="${PORT:-auto}"   # "auto" = detection (USB-serie) ; sinon /dev/ttyUSBx, COMx...
BAUD="${BAUD:-921600}"   # doit correspondre a UART_COMM_BAUDRATE du firmware
CHUNK="${CHUNK:-4096}"
GOLDEN_OFFSET="${GOLDEN_OFFSET:-0x400000}"   # doit == QSPI_GOLDEN_OFFSET (board_config.h)
DO_REBOOT=1

# --- Jolis logs -------------------------------------------------------------
c_ok(){ printf '\033[32m%s\033[0m\n' "$*"; }
c_info(){ printf '\033[36m%s\033[0m\n' "$*"; }
c_err(){ printf '\033[31m%s\033[0m\n' "$*" >&2; }
die(){ c_err "ERREUR: $*"; exit 1; }
need(){ command -v "$1" >/dev/null 2>&1 || die "outil '$1' introuvable dans le PATH${2:+ ($2)}"; }

# --- OS + resolution de l'environnement Xilinx (bootgen / program_flash) -----
# Detecte Windows (Git Bash/MSYS/Cygwin) : sur Windows on appelle le VRAI
# bootgen.exe natif, jamais le loader Linux via settings64.sh (sinon
# "Unsupported architecture: i686-pc"). Sur Linux/WSL on source settings64.sh.
# Surchargeable : VITIS_DIR=/chemin/vers/Vitis/<version> ./zynq_ota.sh build
case "$(uname -s 2>/dev/null)" in
  MINGW*|MSYS*|CYGWIN*) IS_WIN=1;;
  *) IS_WIN=0;;
esac
BOOTGEN="bootgen"    # resolu par find_vitis (chemin du wrapper sur Windows)
BOOTGEN_BAT=0        # 1 => BOOTGEN est un bootgen.bat a lancer via cmd (Windows)

find_vitis(){
  command -v "$BOOTGEN" >/dev/null 2>&1 && return 0

  local roots=(/tools/Xilinx /opt/Xilinx "$HOME/Xilinx" \
               /c/Xilinx /mnt/c/Xilinx "/c/Program Files/Xilinx" "/mnt/c/Program Files/Xilinx")
  [ -n "${VITIS_DIR:-}" ] && roots=("$(dirname "$(dirname "$VITIS_DIR")")" "${roots[@]}")

  local root cand best=""

  if [ "$IS_WIN" = 1 ]; then
    # Windows : liste des dossiers d'install candidats (VITIS_DIR + Vitis/Vivado sous les racines)
    local vdirs=() d
    [ -n "${VITIS_DIR:-}" ] && vdirs+=("$VITIS_DIR")
    for root in "${roots[@]}"; do
      for d in "$root"/Vitis/* "$root"/Vivado/*; do [ -d "$d" ] && vdirs+=("$d"); done
    done
    # a) exe natif + dossiers de DLLs ajoutes au PATH (comme le fait le wrapper).
    #    Appel direct depuis bash : pas de couche cmd/.bat => pas de galere de
    #    quoting sur les chemins Windows.
    for d in "${vdirs[@]}"; do
      for cand in "$d"/bin/unwrapped/*/bootgen.exe; do
        if [ -f "$cand" ]; then
          c_info "[env] bootgen exe : $cand"
          [ -d "$d/lib/win64.o" ]           && PATH="$d/lib/win64.o:$PATH"
          [ -d "$d/bin/unwrapped/win64.o" ] && PATH="$d/bin/unwrapped/win64.o:$PATH"
          [ -d "$d/tps/win64.o" ]           && PATH="$d/tps/win64.o:$PATH"
          BOOTGEN="$cand"; BOOTGEN_BAT=0; return 0
        fi
      done
    done
    # b) dernier recours : wrapper bootgen.bat lance sous cmd.exe
    for d in "${vdirs[@]}"; do
      if [ -f "$d/bin/bootgen.bat" ]; then
        c_info "[env] bootgen (wrapper .bat) : $d/bin/bootgen.bat"
        BOOTGEN="$d/bin/bootgen.bat"; BOOTGEN_BAT=1; return 0
      fi
    done
    return 1
  fi

  # Linux / WSL : settings64.sh configure tout l'environnement
  if [ -n "${VITIS_DIR:-}" ] && [ -f "$VITIS_DIR/settings64.sh" ]; then
    . "$VITIS_DIR/settings64.sh"; command -v bootgen >/dev/null 2>&1 && return 0
  fi
  for root in "${roots[@]}"; do
    [ -d "$root" ] || continue
    for cand in "$root"/Vitis/*/settings64.sh; do [ -f "$cand" ] && best="$cand"; done
  done
  if [ -n "$best" ]; then c_info "[env] Vitis detecte : $best"; . "$best"; command -v bootgen >/dev/null 2>&1 && return 0; fi
  return 1
}

usage(){
  cat <<USAGE
Usage: $(basename "$0") <build|send|all|jtag> [options]

  build            Genere BOOT.bin via bootgen (necessite les outils Xilinx).
  send             Envoie BOOT.bin par serie + reboot (Raspberry Pi : python3 + pyserial).
  all              build puis send.
  jtag             Flash QSPI (image primaire, offset 0) par JTAG via program_flash.
  golden           Flash l'image de SECOURS (golden) par JTAG a GOLDEN_OFFSET (a faire
                   UNE fois, avec un BOOT.bin connu-bon). Filet anti-brick de l'OTA.

Options :
  -p, --port DEV   Port serie (auto|/dev/ttyUSBx|COMx) (defaut: $PORT)
  -b, --baud N     Debit                         (defaut: $BAUD)
  -o, --bin FILE   Chemin du BOOT.bin            (defaut: $BOOTBIN)
      --fsbl FILE  FSBL .elf                     (defaut: .../boot/fsbl.elf)
      --bit FILE   Bitstream .bit
      --elf0 FILE  ELF CPU0 (opossum_core1)
      --elf1 FILE  ELF CPU1 (opossum_core2)
      --chunk N    Taille de bloc serie          (defaut: $CHUNK)
      --no-reboot  Ne pas envoyer REBOOT apres l'ecriture
  -h, --help       Cette aide

Exemples :
  ./zynq_ota.sh build
  ./zynq_ota.sh send -p /dev/ttyUSB0 -b 115200
  ./zynq_ota.sh all --no-reboot
USAGE
}

# --- Parsing ----------------------------------------------------------------
[ $# -ge 1 ] || { usage; exit 1; }
CMD="$1"; shift || true
while [ $# -gt 0 ]; do
  case "$1" in
    -p|--port)  PORT="$2"; shift 2;;
    -b|--baud)  BAUD="$2"; shift 2;;
    -o|--bin)   BOOTBIN="$2"; shift 2;;
    --fsbl)     FSBL="$2"; shift 2;;
    --bit)      BIT="$2"; shift 2;;
    --elf0)     ELF0="$2"; shift 2;;
    --elf1)     ELF1="$2"; shift 2;;
    --chunk)    CHUNK="$2"; shift 2;;
    --no-reboot) DO_REBOOT=0; shift;;
    -h|--help)  usage; exit 0;;
    *) die "option inconnue: $1";;
  esac
done

# --- build ------------------------------------------------------------------
do_build(){
  find_vitis || die "bootgen introuvable. Indique l'install : VITIS_DIR=/c/Xilinx/Vitis/2020.2 (Windows) ou source .../settings64.sh (Linux/WSL)"
  for f in "$FSBL" "$BIT" "$ELF0" "$ELF1"; do
    [ -f "$f" ] || die "artefact manquant: $f"
  done
  local bif fsbl bit elf0 elf1 bifarg outarg
  bif="$(mktemp /tmp/opossum_ota.XXXXXX.bif)"
  if [ "$IS_WIN" = 1 ]; then
    # Le bootgen.exe natif veut des chemins Windows (C:/...), y compris ceux
    # ecrits DANS le .bif. cygpath -m => forme "mixte" C:/Users/... acceptee.
    fsbl=$(cygpath -m "$FSBL");  bit=$(cygpath -m "$BIT")
    elf0=$(cygpath -m "$ELF0");  elf1=$(cygpath -m "$ELF1")
    bifarg=$(cygpath -m "$bif"); outarg=$(cygpath -m "$BOOTBIN")
  else
    fsbl="$FSBL"; bit="$BIT"; elf0="$ELF0"; elf1="$ELF1"
    bifarg="$bif"; outarg="$BOOTBIN"
  fi
  # Ordre = celui du .bif Vitis : FSBL (bootloader), bitstream, ELF CPU0
  # (handoff FSBL), ELF CPU1 (charge en DDR, reveille par CPU0 dans main.c).
  cat > "$bif" <<BIF
the_ROM_image:
{
	[bootloader]$fsbl
	$bit
	$elf0
	$elf1
}
BIF
  c_info "[build] bootgen -> $BOOTBIN"
  if [ "$IS_WIN" = 1 ] && [ "$BOOTGEN_BAT" = 1 ]; then
    # Wrapper .bat sous cmd.exe : chemins en backslash quotes (sinon cmd casse)
    local batwin bifwin outwin
    batwin=$(cygpath -w "$BOOTGEN"); bifwin=$(cygpath -w "$bif"); outwin=$(cygpath -w "$BOOTBIN")
    MSYS_NO_PATHCONV=1 MSYS2_ARG_CONV_EXCL='*' "${COMSPEC:-cmd}" //c "\"$batwin\" -arch zynq -image \"$bifwin\" -w on -o \"$outwin\""
  else
    # Appel direct de l'exe (Windows: DLLs deja dans le PATH) ou de bootgen (Linux)
    MSYS_NO_PATHCONV=1 MSYS2_ARG_CONV_EXCL='*' "$BOOTGEN" -arch zynq -image "$bifarg" -w on -o "$outarg"
  fi
  rm -f "$bif"
  c_ok "[build] OK : $BOOTBIN ($(stat -c%s "$BOOTBIN" 2>/dev/null || wc -c <"$BOOTBIN") octets)"
}

# --- send (transfert serie, pur python3/pyserial) ---------------------------
do_send(){
  [ -f "$BOOTBIN" ] || die "BOOT.bin introuvable: $BOOTBIN (lance d'abord 'build')"
  local PY
  PY=$(command -v python3 || command -v python) || die "python introuvable (installe Python 3)"
  c_info "[send] $BOOTBIN -> port=$PORT @ ${BAUD} bauds"
  "$PY" - "$PORT" "$BAUD" "$BOOTBIN" "$CHUNK" "$DO_REBOOT" <<'PYEOF'
import sys, time, zlib
try:
    import serial
    from serial.tools import list_ports
except ImportError:
    import platform
    if platform.system() == "Windows":
        sys.exit("pyserial manquant -> py -m pip install pyserial")
    sys.exit("pyserial manquant -> sudo apt install python3-serial  (ou pip3 install pyserial)")

req, baud, path, chunk_req, do_reboot = sys.argv[1], int(sys.argv[2]), sys.argv[3], int(sys.argv[4]), sys.argv[5] == "1"

# --- Auto-detection du port serie (Windows COMx comme Linux ttyUSB/ttyACM) ---
# VID des ponts USB-serie courants : FTDI, CP210x, CH340/CH341, Prolific, Arduino
KNOWN_VID = {0x0403, 0x10c4, 0x1a86, 0x067b, 0x2341, 0x2e8a, 0x239a}
def looks_usb(p):
    if getattr(p, "vid", None) is not None:
        return True
    dev = (p.device or "")
    return ("ttyUSB" in dev) or ("ttyACM" in dev) or ("USB" in (p.description or "").upper())

def pick_port(req):
    if req and req.lower() != "auto":
        return req
    ports = list(list_ports.comports())
    cands = [p for p in ports if looks_usb(p)] or ports
    if not cands:
        sys.exit("[send] aucun port serie trouve (carte branchee ? pilote USB installe ?)")
    if len(cands) == 1:
        print(f"[send] port auto-detecte : {cands[0].device} ({cands[0].description})")
        return cands[0].device
    known = [p for p in cands if getattr(p, "vid", None) in KNOWN_VID]
    if len(known) == 1:
        print(f"[send] port auto-detecte : {known[0].device} ({known[0].description})")
        return known[0].device
    print("[send] plusieurs ports possibles, precise avec -p :")
    for p in cands:
        print(f"    {p.device:14} {p.description}  [{p.hwid}]")
    sys.exit(2)

port = pick_port(req)
data = open(path, "rb").read()
size = len(data)
crc  = zlib.crc32(data) & 0xffffffff
print(f"[send] {port} @ {baud}  taille={size} o  crc32={crc}")

try:
    ser = serial.Serial(port, baud, timeout=10)
except serial.SerialException as e:
    sys.exit(f"[send] ouverture {port} impossible : {e}")
time.sleep(0.2); ser.reset_input_buffer()

def rl(t=10):
    ser.timeout = t
    return ser.readline().decode("ascii", "replace").strip()

def wait_prefix(prefixes, t=15):
    end = time.time() + t
    while time.time() < end:
        line = rl(2)
        if not line:
            continue
        for pfx in prefixes:
            if line.startswith(pfx):
                return line
        if line.startswith("-ERR"):
            sys.exit(f"[send] firmware -> {line}")
    return None

# 1) handshake (le firmware efface la zone QSPI avant de repondre +READY :
#    ca peut prendre plusieurs dizaines de secondes selon la taille)
ser.write(f"FWUPDATE {size} {crc}\r".encode())
print("[send] effacement QSPI + attente +READY ...")
line = wait_prefix(["+READY"], 120)
if not line:
    sys.exit("[send] pas de +READY (commande FWUPDATE cote firmware ? bon port/baud ?)")
parts = line.split()
chunk = int(parts[1]) if len(parts) >= 2 else chunk_req
print(f"[send] firmware pret, bloc={chunk} o")

# 2) envoi bloc par bloc avec ACK
sent = 0; t0 = time.time(); MAX_RETRY = 5
while sent < size:
    block = data[sent:sent + chunk]
    retry = 0
    while True:
        ser.write(block)
        ans = wait_prefix(["+ACK", "-NAK"], 10)
        if ans and ans.startswith("+ACK"):
            sent += len(block); break
        retry += 1
        if retry > MAX_RETRY:
            sys.exit(f"\n[send] echec bloc @ {sent} o (trop de NAK/timeout)")
    print(f"\r[send] {sent}/{size} ({100*sent//size}%)", end="", flush=True)
print()

# 3) fin + verification
line = wait_prefix(["+DONE"], 30)
if not line:
    sys.exit("[send] pas de +DONE (verification CRC KO cote firmware ?)")
print(f"[send] ecrit + verifie en {time.time()-t0:.1f}s")

# 4) reboot
if do_reboot:
    print("[send] REBOOT -> boot QSPI")
    ser.write(b"REBOOT\r"); time.sleep(0.5)
ser.close()
PYEOF
  c_ok "[send] termine"
}

# --- jtag (bonus) -----------------------------------------------------------
do_jtag(){
  [ -f "$BOOTBIN" ] || die "BOOT.bin introuvable: $BOOTBIN (lance d'abord 'build')"
  find_vitis || die "program_flash introuvable. Source l'env Vitis ou exporte VITIS_DIR=/chemin/vers/Vitis/<version>"
  c_info "[jtag] program_flash QSPI @ offset 0"
  program_flash -f "$BOOTBIN" -offset 0 -flash_type qspi_single \
    -fsbl "$FSBL" -cable type xilinx_tcf url TCP:127.0.0.1:3121
  c_ok "[jtag] OK (reset la carte pour booter QSPI)"
}

# --- golden : image de secours a GOLDEN_OFFSET (une fois, BOOT.bin connu-bon) --
do_golden(){
  [ -f "$BOOTBIN" ] || die "BOOT.bin introuvable: $BOOTBIN (lance d'abord 'build')"
  [ "$IS_WIN" = 1 ] && die "sur Windows/Git Bash, lance 'golden' depuis un Vitis Command Prompt (program_flash)."
  find_vitis || die "program_flash introuvable (source l'env Vitis)"
  c_info "[golden] program_flash image de secours @ $GOLDEN_OFFSET"
  program_flash -f "$BOOTBIN" -offset "$GOLDEN_OFFSET" -flash_type qspi_single \
    -fsbl "$FSBL" -cable type xilinx_tcf url TCP:127.0.0.1:3121
  c_ok "[golden] OK : image de secours en place. L'OTA (offset 0) ne la touche jamais ;"
  c_ok "          le BootROM y retombe si l'image primaire est corrompue."
}

case "$CMD" in
  build) do_build;;
  send)  do_send;;
  all)   do_build; do_send;;
  jtag)  do_jtag;;
  golden) do_golden;;
  -h|--help|help) usage;;
  *) die "commande inconnue: $CMD (build|send|all|jtag)";;
esac
