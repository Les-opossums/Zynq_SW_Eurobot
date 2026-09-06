# Mise en place du projet SW sous Vitis 
Les étapes suivantes décrivent la manière pour importer le projet SW sur Vitis en conservant notre *gconf* actuelle. 

## 1. Récupération du projet

Clonez le dépôt et basculez sur la branche de développement :
```bash
git clone git@github.com:Les-opossums/Zynq_SW_Eurobot.git
cd Zynq_SW_Eurobot
git checkout feature/feetech`
```
Lancer Vitis (**Xilinx Vitis 2020.2**), quand le launcher demande de choisir un **Workspace**, il faut donc sélectionner `Zynq_SW_Eurobot/` fraîchement cloné.


## 2. Récupération du wrapper
Pour récupérer le wrapper nous avons un script bash pour télécharger le fichier `.xsa` (configuration matérielle Vivado/Vitis)
depuis les [releases GitHub](https://github.com/Les-opossums/Zynq_HW_Eurobot_2025/releases)
du dépôt `Zynq_HW_Eurobot_2025`.
 
Compatible Linux natif et Windows via Git Bash.
 
### Prérequis
 
- `curl` (déjà présent sous Linux et dans Git Bash)
- `python3` ou `python` dans le PATH (utilisé pour parser le JSON de l'API GitHub —
  déjà présent avec un environnement Vitis/ROS2 classique)
### Utilisation
 
```bash
# Récupérer la dernière release publiée (y compris les pre-releases)
bash update_hw.sh
 
# Récupérer une release précise, par son tag
bash update_hw.sh v0.1.0-alpha1
```
 
### Comportement
 
1. Le répertoire `./opossum_hw` est créé s'il n'existe pas, et tout `.xsa` déjà
   présent y est supprimé avant le téléchargement.
2. Si aucune version n'est précisée (`latest`), le script récupère **la release
   la plus récente**, pre-release incluse (contrairement à l'endpoint GitHub
   `/releases/latest`, qui ignore les pre-releases).
3. Si la release choisie est marquée comme **pre-release** sur GitHub, un
   warning est affiché avant le téléchargement.
4. Le premier fichier `.xsa` trouvé parmi les assets de la release est
   téléchargé dans `./opossum_hw/`.

### Erreurs courantes
 
| Message | Cause probable |
|---|---|
| `HTTP 404` | Tag inexistant, ou dépôt privé sans authentification |
| `HTTP 403` (mention *rate limit*) | Quota anonyme de l'API GitHub dépassé (60 requêtes/h/IP) |
| `Aucun fichier .xsa trouve pour cette release` | La release existe mais n'a pas d'asset `.xsa` attaché |
| `python n'est pas installe` | Installer Python, ou vérifier qu'il est dans le PATH (`python3 --version`) |

## 3. Creation de la **Platform**
1. Cliquer sur `Create Platform Project` 
2. Lui donner un nom (ex: `Zynq_block_design_wrapper)
3. Sélectionner `Create a new platform from hardware (XSA)`
4. Choisir le fichier de description HW `Zynq_block_design_wrapper.xsa` présent dans `Zynq_SW_Eurobot/`
5. Cliquer sur `Finish`


## 4. Creation du **System** + **Applications**
Vitis c'est super ! (non.) 
**Note importante :** Vitis refuse de créer un projet si le dossier de destination existe déjà. Pour conserver nos fichiers sources actuels, on est obligé de contourner en faisant un renommage temporaire.

#### étape de contournement :
1. Renommer les répertoires `opossum_core1/` et `opossum_core2/` (`_opossum_core1/` et `_opossum_core2/` par exemple)
#### Création d'opossum_core1 :
1. `File` -> `New` -> `Application Project ...`
2. Sélectionner la **platform** créée précédemment 
3. `Application project name` : `opossum_core1`
4. `System project name` : `Eurobot_2025_System`
5. Cocher `Show all processors in the hardware specification`
6. Sélectionner `ps7_cortexa9_1`
7. \[Optionnel] : Renommer le `Name` et `Display Name` 
8. Choisir `Empty Application` puis `Finish`
#### Création d'opossum_core2 :
Répéter les étapes pour créer l'Application `opossum_core2`, en choisissant cette fois le System existant et le coeur `ps7_cortexa9_0`.

#### Restauration des sources : 
1. Copier les sources de `_opossum_core1/src/` vers `opossum_core1/src/` puis supprimer `_opossum_core1`
2. Copier les sources de `_opossum_core2/src/` vers `opossum_core2/src/` puis supprimer `_opossum_core2`
## 5. Configuration des **bibliothèques**
On a maintenant un projet fonctionnel mais qui ne build pas ! 
Pas de soucis, il faut simplement ajouter la librairie math (`m`) au Build tool : 
1. Clique droit sur `opossum_core1` puis `C/C++ Build Settings`
2. Sélectionner `[ All configurations ]` pour la `Configuration`
3. Chercher `Librairies` dans la catégorie `ARM v7 gcc linker` de l'onglet `Tool Settings`
4. Cliquer sur le `+` puis ajouter `m`
5. Apply and close
6. Réitérer pour `opossum_core2`

## 6. Définir `THIS_CORE` dans les paramètres de Vitis

Cette étape est indispensable pour configurer le compilateur afin qu'il ne prenne que ce qui concerne le coeur qu'il compile dans les src partagées.

### Pour le projet du CPU0

1. Fais un clic droit sur le projet d'application **opossum_core1** dans l'explorateur, puis ouvre :
   - **C/C++ Build Settings**, ou
   - **Properties** → **C/C++ Build** → **Settings**.
2. Va dans **ARM v7 gcc compiler** → **Symbols**.
3. Clique sur l'icône **Ajouter (+)**.
4. Saisis exactement pour le core1 :
   ```text
   THIS_CORE=0 
   ```
   *(sans espaces)*.
5. Applique les modifications et ferme la fenêtre.

### Pour le projet du CPU1

Répète la même opération sur le projet **opossum_core2**.

Ajoute le symbole suivant :

```text
THIS_CORE=1
```

> **Remarque :** On utilise directement les valeurs `0` et `1` car le préprocesseur doit pouvoir évaluer les directives `#if` avant même l'inclusion des fichiers d'en-tête.

## 7. Setup du dossier partagé entre les coeurs

Pour éviter la duplication de code entre le CPU0 et le CPU1, les fichiers communs (comme le gestionnaire d'IO) sont placés dans le dossier externe `opossum_common`.
Lors de la première importation du projet sur un nouveau PC, vous devez indiquer au compilateur où trouver ce dossier.

### Étape 1 : Ajouter le chemin d'inclusion (Include Path)
Cette manipulation est à faire pour le projet d'application CPU0 **ET** CPU1.

1. Dans l'explorateur Vitis, faites un **clic droit** sur le projet d'application (ex: `app_cpu0`) -> **Properties** (ou *C/C++ Build Settings*).
2. Vérifiez en haut de la fenêtre que **Configuration** est réglé sur **[ All configurations ]** (très important).
3. Naviguez vers **C/C++ Build** -> **Settings**.
4. Dans l'onglet *Tool Settings*, déroulez **ARM v7 gcc compiler** et cliquez sur **Directories**.
5. Dans la zone *Include paths (-I)*, cliquez sur l'icône **Ajouter (+)**.
6. Tapez directement le chemin relatif suivant :
   `"${workspace_loc}\opossum_common"` 
7. Cliquez sur **Apply and Close**.

### Étape 2 : Nettoyer et Compiler
1. Project -> **Clean...** (Nettoyez tous les projets).
2. Project -> **Build All**.



On est good, normalement le repo git est clean et le projet est fonctionnel ! 


# Setup nécessaire pour la communication avec le haut niveau : 

## 1. Communication Réseau (Ethernet)

La communication entre le Raspberry Pi (ROS 2) et le Zynq 7000 se fait via une liaison Ethernet UDP bare-metal 100Mbits/s.

👉 [Voir la documentation détaillée du Driver Ethernet UDP](./opossum_common/IO_MANAGER/DRIVER_ETH/README.md)

## 2. Communication série (UART)

Alternativement, la communication peut se faire en UART (console de commandes, cf. [APP_COM](./opossum_common/APP_COM/README.md)). Débit **921600 bauds** (`UART_COMM_BAUDRATE` dans [`IO_config.h`](./opossum_common/IO_config.h)) — pense à régler ton terminal série en conséquence.

👉 [Voir la documentation détaillée du Driver UART PS](./opossum_common/IO_MANAGER/DRIVER_UART_PS/README.md)


# Setup entre hardware et software : 

## 1. Setup GPIO PS
👉 [Voir la documentation détaillée du Driver GPIO](./opossum_common/IO_MANAGER/DRIVER_PS_GPIO/README.md)

## 2. Mise à jour du PHY Ethernet après régénération du BSP

Le fichier BSP Xilinx `xemacpsif_physpeed.c` est écrasé par Vitis à chaque régénération et perd le patch forçant le lien à 100 Mbits (sinon négocié à tort à 10 Mbits sur le PHY RTL8201F). Relancer `patch_phy_speed.sh` (racine du dépôt) après chaque "Re-generate BSP Sources".

---

# Mise à jour du firmware par liaison série (OTA)

Le firmware peut être reflashé **à distance**, par la liaison série (UART, ou le pont USB-série de la carte / du Raspberry), **sans JTAG ni ouverture du robot**. Le host envoie un `BOOT.bin`, le Zynq l'écrit en QSPI, le vérifie (CRC32) puis reboote dessus.

* Côté firmware : [APP_FWUPDATE](./opossum_common/APP_FWUPDATE/README.md) (commande `FWUPDATE` + driver QSPI).
* Côté host : le script [`zynq_ota.sh`](./zynq_ota.sh) (racine du dépôt).

### Prérequis : boot QSPI

La carte doit être **strappée QSPI** (mode de boot échantillonné au POR sur Zynq-7000 ; un reset logiciel ne le rechange pas). Une fois strappée QSPI, elle démarre seule sur la flash au power-up — plus de JTAG à chaque démarrage du robot.

### Rôles de `zynq_ota.sh`

| Commande | Rôle | Machine |
|---|---|---|
| `build` | génère `BOOT.bin` (bootgen : FSBL + bitstream + core1 + core2) | PC avec Vitis |
| `send`  | envoie le `BOOT.bin` par série + `REBOOT` (auto-détection du port) | PC ou **Raspberry** |
| `all`   | `build` puis `send` | PC avec Vitis |
| `jtag`  | flash QSPI de l'image primaire par JTAG (`program_flash`, offset 0) | PC avec Vitis |
| `golden`| flash de l'image de **secours** par JTAG (offset `0x400000`), **à faire une fois** | PC avec Vitis |

`build`/`jtag`/`golden` ont besoin des outils Xilinx (le script auto-détecte Vitis/Vivado, y compris sous Git Bash Windows) ; `send` n'a besoin que de `python3` + `pyserial` — c'est ce rôle que le Raspberry jouera à terme.

### Mise en place (une fois) puis usage

```bash
./zynq_ota.sh build     # génère BOOT.bin
./zynq_ota.sh golden    # installe le filet de secours (JTAG, BOOT.bin connu-bon)
./zynq_ota.sh jtag      # image primaire initiale (ou 'send' si déjà en QSPI)

# ensuite, mises à jour à distance :
./zynq_ota.sh send      # ou 'all'
```

### Sécurité anti-brick (image golden)

Un update interrompu ne peut **pas** briquer la carte : une image **golden** (secours) vit à `0x400000` (cf. [`board_config.h`](./opossum_common/board_config.h)), jamais touchée par l'OTA. Si l'image primaire (offset 0) est corrompue, le BootROM fait sa *Golden Image Search* (recherche d'un en-tête valide tous les 32 Ko) et boote la golden. En prime, l'OTA écrit **l'en-tête de boot en dernier**, donc l'offset 0 reste invalide tant que l'update n'est pas complet et vérifié. Détails : [APP_FWUPDATE](./opossum_common/APP_FWUPDATE/README.md).

> ⚠️ Si après `REBOOT` la carte ne repart que par un vrai power-cycle, voir la note « boot mode et soft reset » de [SYSTEM_MANAGER](./opossum_common/SYSTEM_MANAGER/README.md).

# Configuration « carte nue » (`board_config.h`)

Pour accélérer l'init quand la carte est seule (pas de LIDAR, moteurs CAN, IMU, servos...), les drivers non utilisés sont désactivés à la compilation via des flags dans [`board_config.h`](./opossum_common/board_config.h) :

```c
#define USE_ETHERNET   0
#define USE_WS2812B    1
#define USE_IMU        0
#define USE_FEETECH    0
#define USE_CAN        0
#define USE_LIDAR      0
#define USE_ASSERV     0   // boucle d'asserv CPU1 (dépend de CAN + IMU)
```

Chaque flag pilote **en un seul endroit** l'entrée dans la table `DeviceTable[]` ([`IO_manager.c`](./opossum_common/IO_MANAGER/README.md)) **et** les appels directs correspondants ([`core0_loop.c`](./opossum_core1/README.md), [`core1_loop.c`](./opossum_core2/README.md)). `UART_COMM` et `GPIO_PS` restent toujours actifs. Pour rebrancher un périphérique : repasser son flag à `1` et recompiler.

# LED de vie (« alive »)

Une LED câblée sur **MIO0** (banque 500) clignote à **1 Hz** dès que la boucle principale de CPU0 tourne : elle sert d'indicateur visuel « carte bien flashée et opérationnelle » (pilotée via [DRIVER_PS_GPIO](./opossum_common/IO_MANAGER/DRIVER_PS_GPIO/README.md), toggle dans `core0_loop.c`). Si elle se fige, CPU0 est bloqué.

# Documentation du code (architecture et navigation)

Toute l'architecture logicielle (drivers, applications, découpage CPU0/CPU1) est documentée module par module, avec des liens croisés pour naviguer de proche en proche. Point d'entrée :

👉 **[opossum_common — vue d'ensemble et sommaire complet](./opossum_common/README.md)**

Projets spécifiques à un cœur :
* [opossum_core1](./opossum_core1/README.md) — projet CPU0 (communication, actionneurs)
* [opossum_core2](./opossum_core2/README.md) — projet CPU1 (asservissement, odométrie, Kalman)
