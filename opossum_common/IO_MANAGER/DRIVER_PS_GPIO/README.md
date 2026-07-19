# Driver PS GPIO

Ce sous-dossier contient le pilote spécifique pour la gestion des GPIO du Processing System (PS) du Zynq-7000. Il est conçu pour s'intégrer de manière transparente avec l'architecture centralisée de l'`IO_MANAGER`.

## Description

Le `DRIVER_PS_GPIO` agit comme une surcouche ("wrapper") autour des bibliothèques standards Xilinx (`xgpiops.h`). Son objectif principal est de simplifier la gestion des broches GPIO (entrées/sorties) et de leurs interruptions, en liant automatiquement l'état matériel à des variables logicielles partagées.

## Fonctionnalités Principales

*   **Configuration par table :** Les broches sont configurées via un tableau de structures `gpio_pin_config_t`, permettant de définir en une seule ligne le numéro de broche, la direction, le type d'interruption, et la variable liée.
*   **Mise à jour automatique des états :** 
    *   En mode entrée (sans interruption) ou en mode sortie, la fonction `PS_GPIO_Update` synchronise automatiquement l'état physique de la broche avec la variable pointée par `state_var`.
*   **Gestion centralisée des interruptions :** 
    *   Une fonction de callback générique (`PS_GPIO_Callback`) détermine la banque et la broche ayant déclenché l'interruption.
    *   Elle met instantanément à jour la variable liée et appelle un callback spécifique à la broche s'il a été défini dans la configuration.
*   **Types d'interruptions supportées :** Front montant, front descendant, ou les deux (via l'enum `pin_irq_t`).

## Structure de Données

Le pilote s'articule autour de deux structures principales :
1.  `gpio_pin_config_t` : Représente la configuration unitaire d'une broche.
2.  `ps_gpio_context_t` : Représente le contexte global du driver (instance matérielle, pointeur vers la table des broches, et nombre total de broches).

## Interface Standard (`IO_MANAGER`)

Le driver expose deux fonctions conformes aux attentes de l'`IO_MANAGER` :
*   `int PS_GPIO_Init(void *instance)` : Initialise le périphérique, configure la direction de chaque broche, active les interruptions si nécessaire, et attache le gestionnaire d'interruptions Xilinx.
*   `void PS_GPIO_Update(void *instance)` : Fonction de polling ("scrutation") appelée dans la boucle principale pour gérer les broches non configurées en interruption.

## Voir aussi

* [IO_MANAGER](../README.md) — table des périphériques et cycle de vie générique
* [DRIVER_BNO085](../DRIVER_BNO085/README.md) — utilise ce driver pour ses broches CS/RST/INT (contexte GPIO partagé)
* [APP_TEST](../../APP_TEST/README.md) — test de bout en bout GPIO PS → LED
* [opossum_common](../../README.md) — vue d'ensemble de l'architecture