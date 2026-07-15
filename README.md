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

On est good, normalement le repo git est clean et le projet est fonctionnel ! 


# Setup nécessaire pour la communication avec le haut niveau : 

## 1. Communication Réseau (Ethernet)

La communication entre le Raspberry Pi (ROS 2) et le Zynq 7000 se fait via une liaison Ethernet UDP bare-metal 100MBits/s.

👉 [Voir la documentation détaillée du Driver Ethernet UDP](./opossum_core1/src/ETHERNET/README.md)

## 2. Communication série (UART)

Alternativement, la communication peut se faire en UART (Legacy 2024/2025).

👉 [Documentation TBD]