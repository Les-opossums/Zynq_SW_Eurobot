#!/usr/bin/env bash
set -euo pipefail

# Usage : ./patch_phy_speed.sh
#
# Force le lien Ethernet a 100 Mbps pour le PHY RTL8201F (au lieu du 10 Mbps
# neglocie par defaut), en reecrivant get_Realtek_phy_speed() dans chaque
# copie de xemacpsif_physpeed.c presente dans le projet.
#
# Pourquoi ce script : xemacpsif_physpeed.c fait partie du BSP genere par
# Vitis (libsrc/lwip211_v1_3/.../netif/). A chaque "Re-generate BSP Sources"
# (ou reimport du .xsa), Vitis ecrase ce fichier avec son propre template et
# le patch manuel est perdu. Ce script le reapplique automatiquement et de
# facon idempotente : a relancer apres chaque regeneration de BSP dans Vitis.
#
# Ce que fait le patch : le RTL8201F est un PHY Fast Ethernet (10/100
# uniquement, pas de Gigabit). Le code Xilinx d'origine annonce quand meme le
# 1000BASE-T puis relit un registre de statut specifique au Gigabit pour
# deduire le debit negocie ; sur ce PHY, cette lecture renvoie a tort 10 Mbps
# au lieu du 100 Mbps reellement cable. Le patch retire l'annonce Gigabit,
# ajoute un reset explicite du PHY avant l'autonegotiation, et retourne
# directement 100 Mbps une fois l'autonegotiation terminee.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$SCRIPT_DIR"
MARKER="EUROBOT_PATCH_RTL8201F_100M"
TARGET_FUNC_SIG='static u32_t get_Realtek_phy_speed(XEmacPs \*xemacpsp, u32_t phy_addr)'

# --- Corps de la fonction patchee -----------------------------------------
PATCHED_FUNC_FILE="$(mktemp)"
trap 'rm -f "$PATCHED_FUNC_FILE"' EXIT

cat > "$PATCHED_FUNC_FILE" <<EOF
static u32_t get_Realtek_phy_speed(XEmacPs *xemacpsp, u32_t phy_addr)
{
	/* === ${MARKER} ===
	 * Ne pas editer a la main : ce bloc est regenere par
	 * patch_phy_speed.sh a chaque "Re-generate BSP Sources" dans Vitis
	 * (qui ecrase sinon ce fichier avec le template Xilinx d'origine).
	 * Voir l'entete de patch_phy_speed.sh pour le detail du pourquoi.
	 */
	u16_t control;
	u16_t status;
	u32_t timeout_counter = 0;

	xil_printf("Start PHY autonegotiation \r\n");

	XEmacPs_PhyRead(xemacpsp, phy_addr, IEEE_AUTONEGO_ADVERTISE_REG, &control);
	control |= IEEE_ASYMMETRIC_PAUSE_MASK;
	control |= IEEE_PAUSE_MASK;
	control |= ADVERTISE_100;
	control |= ADVERTISE_10;
	XEmacPs_PhyWrite(xemacpsp, phy_addr, IEEE_AUTONEGO_ADVERTISE_REG, control);

	/* Reset explicite du PHY avant de (re)lancer l'autonegotiation */
	XEmacPs_PhyRead(xemacpsp, phy_addr, IEEE_CONTROL_REG_OFFSET, &control);
	control |= IEEE_CTRL_RESET_MASK;
	XEmacPs_PhyWrite(xemacpsp, phy_addr, IEEE_CONTROL_REG_OFFSET, control);

	while (1) {
		XEmacPs_PhyRead(xemacpsp, phy_addr, IEEE_CONTROL_REG_OFFSET, &control);
		if (control & IEEE_CTRL_RESET_MASK)
			continue;
		else
			break;
	}

	XEmacPs_PhyRead(xemacpsp, phy_addr, IEEE_CONTROL_REG_OFFSET, &control);
	control |= IEEE_CTRL_AUTONEGOTIATE_ENABLE;
	control |= IEEE_STAT_AUTONEGOTIATE_RESTART;
	XEmacPs_PhyWrite(xemacpsp, phy_addr, IEEE_CONTROL_REG_OFFSET, control);

	XEmacPs_PhyRead(xemacpsp, phy_addr, IEEE_STATUS_REG_OFFSET, &status);

	xil_printf("Waiting for PHY to complete autonegotiation.\r\n");

	while ( !(status & IEEE_STAT_AUTONEGOTIATE_COMPLETE) ) {
		sleep(1);
		timeout_counter++;
		if (timeout_counter == 30) {
			xil_printf("Auto negotiation error \r\n");
			return XST_FAILURE;
		}
		XEmacPs_PhyRead(xemacpsp, phy_addr, IEEE_STATUS_REG_OFFSET, &status);
	}
	xil_printf("autonegotiation complete \r\n");

	/* RTL8201F : Fast Ethernet uniquement -> lien force a 100 Mbps
	 * (cf commentaire de patch en tete de fonction). */
	return 100;
}
EOF

# --- Recherche de toutes les copies du fichier dans le projet -------------
mapfile -t TARGET_FILES < <(find "$PROJECT_ROOT" -type f -name "xemacpsif_physpeed.c" 2>/dev/null)

if [[ ${#TARGET_FILES[@]} -eq 0 ]]; then
    echo "Aucun fichier xemacpsif_physpeed.c trouve sous $PROJECT_ROOT." >&2
    exit 1
fi

echo "Fichiers trouves : ${#TARGET_FILES[@]}"

PATCHED_COUNT=0
SKIPPED_COUNT=0
FAILED_COUNT=0

for f in "${TARGET_FILES[@]}"; do
    if ! grep -q "$TARGET_FUNC_SIG" "$f"; then
        echo "  [IGNORE] $f (signature get_Realtek_phy_speed introuvable, verifier manuellement)"
        FAILED_COUNT=$((FAILED_COUNT + 1))
        continue
    fi

    already_patched=0
    if grep -q "$MARKER" "$f"; then
        already_patched=1
    fi

    tmp_out="$(mktemp)"

    awk -v patched_file="$PATCHED_FUNC_FILE" -v sig="static u32_t get_Realtek_phy_speed(" '
        BEGIN { in_func = 0 }
        {
            if (!in_func && index($0, sig) > 0) {
                in_func = 1
                while ((getline line < patched_file) > 0) print line
                close(patched_file)
                next
            }
            if (in_func) {
                if ($0 == "}") { in_func = 0 }
                next
            }
            print
        }
    ' "$f" > "$tmp_out"

    if ! grep -q "$MARKER" "$tmp_out"; then
        echo "  [ECHEC]  $f (le remplacement automatique a echoue, fichier laisse intact)"
        rm -f "$tmp_out"
        FAILED_COUNT=$((FAILED_COUNT + 1))
        continue
    fi

    cat "$tmp_out" > "$f"
    rm -f "$tmp_out"

    if [[ $already_patched -eq 1 ]]; then
        echo "  [A JOUR] $f"
    else
        echo "  [PATCHE] $f"
    fi
    PATCHED_COUNT=$((PATCHED_COUNT + 1))
done

echo ""
echo "Termine : $PATCHED_COUNT fichier(s) a jour, $FAILED_COUNT en echec."
if [[ $FAILED_COUNT -gt 0 ]]; then
    exit 1
fi
