# Configuration et Intégration des Drivers (IO_config)

Ce dossier racine contient les fichiers `IO_config.h` et `IO_config.c`. Ces fichiers constituent le **point d'entrée unique** pour la configuration matérielle de l'architecture multi-cœurs. 

C'est ici que tous les périphériques (GPIO, UART, Capteurs, LEDs) sont instanciés, configurés, et assignés à un cœur spécifique (CPU0, CPU1, ou les deux).

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

### Étape 4 : Instanciation dans `IO_config.c`
Ouvrez le fichier `IO_config.c`. C'est ici que vous allouez concrètement la mémoire pour votre contexte et que vous le paramétrez.

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
        .driver_instance = &Sensor_Ctx,     /* Le pointeur vers le contexte défini dans IO_config.c */ \
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
4. Déclarez la variable concrètement dans `IO_config.c` : `volatile int mon_bouton = 0;`

L'état sera automatiquement mis à jour par le gestionnaire GPIO centralisé.