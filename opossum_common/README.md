# opossum_common — Code partagé CPU0/CPU1

Ce dossier contient tout le code partagé entre les deux projets Vitis ([opossum_core1](../opossum_core1/README.md) = CPU0, [opossum_core2](../opossum_core2/README.md) = CPU1), sur une architecture Zynq-7000 en AMP (deux binaires indépendants, un par cœur, pas de SMP/OS). Chaque cœur compile l'intégralité de ce dossier, et [CORE_ID](CORE_ID/README.md) (`THIS_CORE_ID`) permet à un même fichier de se comporter différemment (ou de ne rien faire) selon le cœur qui l'exécute.

## Démarrage (`main.c`)

`main()` est commun aux deux cœurs et orchestre une séquence maître/esclave stricte :

1. `init_platform()` ([PLATFORM](PLATFORM/README.md)), `IPC_Init()` ([IPC_MANAGER](IPC_MANAGER/README.md)), `IRQ_Manager_Init()` ([IRQ_MANAGER](IRQ_MANAGER/README.md)), `Timer_Manager_Init()` ([TIMER_MANAGER](TIMER_MANAGER/README.md)).
2. **CPU0** initialise tous ses périphériques (`IO_Manager_Init()`, cf. [IO_MANAGER](IO_MANAGER/README.md)), puis réveille CPU1 (écriture du registre de wake-up + `sev()`).
3. **CPU1** initialise ses propres périphériques, exporte son rapport d'init vers la mémoire partagée, puis signale `core1_init_done`.
4. CPU0 attend ce flag puis imprime un rapport d'initialisation **unique et combiné** pour les deux cœurs (évite tout entrelacement sur l'UART partagé).
5. `IRQ_Manager_Start()`, `IPC_SyncCores()` (barrière de synchronisation), puis boucle principale : `App_Init()`/`App_Loop()` (implémentés séparément par chaque projet, cf. [`app_interface.h`](app_interface.h)).

## Configuration matérielle (`IO_config.h` / `IO_globals.c`)

Ces deux fichiers constituent le **point d'entrée unique** pour la configuration matérielle : c'est ici que tous les périphériques (GPIO, UART, capteurs, LEDs, CAN, Ethernet, FEETECH) sont déclarés, configurés, et assignés à un cœur spécifique (`CORE_CPU0`, `CORE_CPU1`, ou `CORE_BOTH`) via la table `IO_DEVICE_TABLE`, consommée par l'[IO_MANAGER](IO_MANAGER/README.md).

* **`IO_config.h`** — includes des drivers, macros de configuration (adresses, baudrates...), déclarations `extern` des contextes, et la macro `IO_DEVICE_TABLE`.
* **`IO_globals.c`** — instanciation concrète des contextes (`Xxx_Ctx = { ... }`) et des variables d'état globales (`AU_state`, `leash_state`...).

---

## 🛠 Comment ajouter un nouveau driver / périphérique ?

L'ajout d'une nouvelle fonctionnalité a été pensé pour être le plus simple et modulaire possible. Si vous avez développé ou intégré un nouveau driver (ex: un capteur I2C, un moteur en SPI), voici les étapes à suivre pour l'intégrer au système :

### Étape 1 : Créer le module du driver
1. Dans le dossier `IO_MANAGER/`, créez un sous-dossier pour votre driver (ex: `DRIVER_I2C_SENSOR/`).
2. Créez vos fichiers `.c` et `.h`.
3. Votre driver **doit** définir :
   * Une structure de contexte (ex: `i2c_sensor_context_t`) qui contiendra toutes les variables nécessaires au fonctionnement du driver.
   * Une fonction d'initialisation : `int SENSOR_Init(void *instance);`
   * Une fonction de mise à jour (polling/lecture) : `void SENSOR_Update(void *instance);`

### Étape 2 : Mettre à jour `IO_manager.h` (Pré-requis)
Assurez-vous que le type de votre nouveau périphérique existe dans l'énumération `dev_type_t` (ex: `DEV_TYPE_I2C`).

### Étape 3 : Déclarations dans `IO_config.h`
Ouvrez le fichier `IO_config.h` et ajoutez les éléments suivants :

1. **L'include** de votre nouveau driver :
   ```c
   #include "IO_MANAGER/DRIVER_I2C_SENSOR/driver_i2c_sensor.h"
   ```
2. **Les macros de configuration** (adresses, constantes) :
   ```c
   #define SENSOR_BASEADDR 0x43C00000
   ```
3. **La déclaration externe de votre contexte** :
   ```c
   extern i2c_sensor_context_t Sensor_Ctx;
   ```

### Étape 4 : Instanciation dans `IO_globals.c`
Ouvrez le fichier `IO_globals.c`. C'est ici que vous allouez concrètement la mémoire pour votre contexte et que vous le paramétrez.

```c
// ==================================================================
// Définition du contexte du driver SENSOR
// ==================================================================
i2c_sensor_context_t Sensor_Ctx = {
    .base_addr = SENSOR_BASEADDR,
    .resolution = 12,
    // ... autres paramètres spécifiques à votre driver
};
```

### Étape 5 : Ajout à la table globale `IO_DEVICE_TABLE`
Retournez dans `IO_config.h`. La macro `IO_DEVICE_TABLE` est le cœur du système. C'est elle qui est lue par l'`IO_Manager` au démarrage et dans la boucle principale.

Ajoutez une nouvelle entrée à la fin de la table :

```c
#define IO_DEVICE_TABLE { \
    // ... [Drivers existants] ... \
    { \
        .type = DEV_TYPE_I2C,               /* Le type défini dans io_manager.h */ \
        .owner = CORE_CPU1,                 /* Le cœur qui gère ce driver (CORE_CPU0, CORE_CPU1 ou CORE_BOTH) */ \
        .driver_instance = &Sensor_Ctx,     /* Le pointeur vers le contexte défini dans IO_globals.c */ \
        .irq_id = XPAR_FABRIC_SENSOR_INTR,  /* L'ID de l'interruption (mettre 0 si pas d'interruption) */ \
        .irq_handler = (Xil_InterruptHandler)Sensor_IntrHandler, /* Fonction de callback IRQ (ou NULL) */ \
        .init = SENSOR_Init,                /* Pointeur vers la fonction d'initialisation */ \
        .update = SENSOR_Update             /* Pointeur vers la fonction de mise à jour/polling */ \
    } \
}
```

---

## 🚀 Utilisation au quotidien

Une fois ces étapes validées, **vous n'avez plus rien à faire !** 

Grâce à l'`IO_MANAGER` :
* Si le driver appartient à `CORE_CPU1`, lors du démarrage du processeur 1, `IO_Manager_Init()` appellera automatiquement `SENSOR_Init(&Sensor_Ctx)`.
* Si le driver possède une interruption matérielle, elle sera automatiquement connectée via l'`IRQ_Manager`.
* La fonction `IO_Manager_Update()` appellera automatiquement `SENSOR_Update(&Sensor_Ctx)` à chaque cycle de la boucle principale du cœur correspondant.

### Gestion simplifiée des GPIO PS
Si vous voulez simplement ajouter un bouton ou une LED basique reliée au processeur (Zynq PS) :
1. Allez dans `IO_config.h`.
2. Déclarez une nouvelle variable globale : `extern volatile int mon_bouton;`
3. Ajoutez une ligne dans la macro `PS_GPIO_PINS` :
   `{ 42, PS_GPIO_DIR_INPUT, PIN_IRQ_EDGE_RISING, &mon_bouton, mon_bouton_Callback }`
4. Déclarez la variable concrètement dans `IO_globals.c` : `volatile int mon_bouton = 0;`

L'état sera automatiquement mis à jour par le gestionnaire GPIO centralisé.

---

## Sommaire des modules

### Infrastructure

* [CORE_ID](CORE_ID/README.md) — identification du cœur à la compilation
* [PLATFORM](PLATFORM/README.md) — initialisation bas niveau (BSP standalone)
* [IRQ_MANAGER](IRQ_MANAGER/README.md) — gestionnaire centralisé des interruptions (GIC)
* [IPC_MANAGER](IPC_MANAGER/README.md) — communication et synchronisation inter-cœurs
* [TIMER_MANAGER](TIMER_MANAGER/README.md) — bases de temps ms/µs et instrumentation (`TIMING_MEASURE`)
* [SYSTEM_MANAGER](SYSTEM_MANAGER/README.md) — redémarrage logiciel complet

### Périphériques ([IO_MANAGER](IO_MANAGER/README.md))

* [DRIVER_PS_GPIO](IO_MANAGER/DRIVER_PS_GPIO/README.md), [DRIVER_WS2812B](IO_MANAGER/DRIVER_WS2812B/README.md), [DRIVER_BNO085](IO_MANAGER/DRIVER_BNO085/README.md), [DRIVER_UART_PS](IO_MANAGER/DRIVER_UART_PS/README.md), [DRIVER_CAN](IO_MANAGER/DRIVER_CAN/README.md), [DRIVER_ETH](IO_MANAGER/DRIVER_ETH/README.md), [DRIVER_FEETECH](IO_MANAGER/DRIVER_FEETECH/README.md)

### Application (communication et logique métier)

* [APP_COM](APP_COM/README.md) — interpréteur de commandes (UART + Ethernet)
* [APP_ASSERV_BRIDGE](APP_ASSERV_BRIDGE/README.md) — commandes de mouvement/asservissement (CPU0 → CPU1)
* [APP_DRIVER_BRIDGE](APP_DRIVER_BRIDGE/README.md) — commandes génériques de pilotage des drivers
* [APP_MOTORS](APP_MOTORS/README.md) — retour et commande des ESC C610
* [APP_TEST](APP_TEST/README.md) — tests de bout en bout

### Projets spécifiques à un cœur

* [opossum_core1](../opossum_core1/README.md) (CPU0) → [APP_ACTIONNEURS](../opossum_core1/src/APP_ACTIONNEURS/README.md) (pinces FEETECH)
* [opossum_core2](../opossum_core2/README.md) (CPU1) → [ASSERV](../opossum_core2/src/ASSERV/README.md), [KALMAN](../opossum_core2/src/KALMAN/README.md)

Voir aussi le [README racine](../README.md) pour la mise en place du projet sous Vitis.