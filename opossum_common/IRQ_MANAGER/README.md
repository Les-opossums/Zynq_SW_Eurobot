# IRQ Manager

Ce dossier contient le gestionnaire centralisé des interruptions (IRQ) pour le système. Il sert d'interface simplifiée pour configurer et utiliser le contrôleur d'interruptions générique (GIC) du Zynq-7000.

## Description

L'`IRQ_Manager` encapsule les appels de bas niveau à la bibliothèque Xilinx `xscugic.h`. Son but est de fournir une API claire et standardisée pour initialiser le contrôleur SCUGIC, attacher des routines de service d'interruption (ISR) à des identifiants matériels spécifiques, et activer globalement la gestion des exceptions du processeur.

## Fonctionnalités Principales

* **Initialisation du Contrôleur :** Recherche de la configuration matérielle via l'ID de périphérique `XPAR_SCUGIC_0_DEVICE_ID`, initialisation de l'instance globale `XScuGic`, et enregistrement du gestionnaire d'exceptions par défaut.
* **Connexion Dynamique :** Permet d'associer un ID d'interruption matériel (ex: Timer, GPIO, UART) à une fonction de callback et d'activer cette interruption spécifique.
* **Activation Globale :** Fournit une commande unique pour activer les interruptions au niveau du processeur (`Xil_ExceptionEnable`) une fois la configuration matérielle terminée.
* **Accès à l'Instance :** Possibilité de récupérer le pointeur vers l'instance locale du contrôleur pour des usages spécifiques non couverts par l'API simplifiée.

## Fonctions de l'API

* `int IRQ_Manager_Init(void)` : Initialise le driver du contrôleur d'interruptions.
* `int IRQ_Manager_Connect(u32 irq_id, Xil_InterruptHandler handler, void *callback_ref)` : Connecte le gestionnaire spécifique au contrôleur et active l'interruption pour l'ID donné.
* `void IRQ_Manager_Start(void)` : Autorise la levée des interruptions matérielles par le cœur CPU.
* `XScuGic *IRQ_Manager_GetInstance(void)` : Retourne un pointeur vers l'objet `InterruptController`.

## Intégration dans l'architecture

L'`IRQ_Manager` est la brique de base exploitée par l'`IO_Manager`. Lorsque ce dernier itère sur la table des périphériques pour les initialiser, il utilise automatiquement `IRQ_Manager_Connect()` pour lier chaque driver ayant déclaré un `irq_id` non nul.
