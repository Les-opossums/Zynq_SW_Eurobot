# Mise en place du projet SW sous Vitis 
Les �tapes suivantes d�crivent la mani�re pour importer le projet SW sur Vitis en conservant notre *gconf* actuelle. 

## 1. R�cup�ration du projet

Clonez le d�p�t et basculez sur la branche de d�veloppement :
```bash
git clone git@github.com:Les-opossums/Zynq_SW_Eurobot.git
cd Zynq_SW_Eurobot
git checkout feature/feetech`
```

Lancer Vitis (**Xilinx Vitis 2020.2**), quand le launcher demande de choisir un **Workspace**, il faut donc s�lectionner `Zynq_SW_Eurobot/` fra�chement clon�.
## 2. Creation de la **Platform**
1. Cliquer sur `Create Platform Project` 
2. Lui donner un nom (ex: `Zynq_block_design_wrapper)
3. S�lectionner `Create a new platform from hardware (XSA)`
4. Choisir le fichier de description HW `Zynq_block_design_wrapper.xsa` pr�sent dans `Zynq_SW_Eurobot/`
5. Cliquer sur `Finish`
## 3. Creation du **System** + **Applications**
Vitis c'est super ! (non.) 
**Note importante :** Vitis refuse de cr�er un projet si le dossier de destination existe d�j�. Pour conserver nos fichiers sources actuels, on est oblig� de contourner en faisant un renommage temporaire.

#### �tape de contournement :
1. Renommer les r�pertoires `opossum_core1/` et `opossum_core2/` (`_opossum_core1/` et `_opossum_core2/` par exemple)
#### Cr�ation d'opossum_core1 :
1. `File` -> `New` -> `Application Project ...`
2. S�lectionner la **platform** cr��e pr�c�demment 
3. `Application project name` : `opossum_core1`
4. `System project name` : `Eurobot_2025_System`
5. Cocher `Show all processors in the hardware specification`
6. S�lectionner `ps7_cortexa9_1`
7. \[Optionnel] : Renommer le `Name` et `Display Name` 
8. Choisir `Empty Application` puis `Finish`
#### Cr�ation d'opossum_core2 :
R�p�ter les �tapes pour cr�er l'Application `opossum_core2`, en choisissant cette fois le System existant et le c�ur `ps7_cortexa9_0`.

#### Restauration des sources : 
1. Copier les sources de `_opossum_core1/src/` vers `opossum_core1/src/` puis supprimer `_opossum_core1`
2. Copier les sources de `_opossum_core2/src/` vers `opossum_core2/src/` puis supprimer `_opossum_core2`
## 4. Configuration des **biblioth�ques**
On a maintenant un projet fonctionnel mais qui ne build pas ! 
Pas de soucis, il faut simplement ajouter la librairie math (`m`) au Build tool : 
1. Clique droit sur `opossum_core1` puis `C/C++ Build Settings`
2. S�lectionner `[ All configurations ]` pour la `Configuration`
3. Chercher `Librairies` dans la cat�gorie `ARM v7 gcc linker` de l'onglet `Tool Settings`
4. Cliquer sur le `+` puis ajouter `m`
5. Apply and close
6. R�it�rer pour `opossum_core2`

On est good, normalement le repo git est clean et le projet est fonctionnel ! 


# Setup nécessaire pour la communication avec le haut niveau : 

## 1. Communication Réseau (Ethernet)

La communication entre le Raspberry Pi (ROS 2) et le Zynq 7000 se fait via une liaison Ethernet UDP bare-metal 100MBits/s.

👉 [Voir la documentation détaillée du Driver Ethernet UDP](./opossum_core1/src/ETHERNET/README.md)

## 2. Communication série (UART)

Alternativement, la communication peut se faire en UART (Legacy 2024/2025).

👉 [Documentation TBD]