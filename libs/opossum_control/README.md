# opossum_control — Asservissement et Kalman en C pur

Odométrie, génération de trajectoire, contraintes de vitesse/accélération, PID de vitesse
et filtre de Kalman (fusion odométrie / lidar / caméras / IMU) du robot. Le **même code**
tourne sur le CPU1 du Zynq et sur PC (tests, simulation SIL avec MuJoCo).

La bibliothèque ne connaît ni l'IPC, ni le CAN, ni les timers : les consignes et capteurs
entrent par [`control_loop.h`](src/control_loop.h), les commandes moteur en sortent. Les
quelques services restants (log, remontée de `motion_done`, chronométrage) passent par
[`control_hal.h`](src/control_hal.h).

```
           CORE0 (IPC)                                   PC (SIL / tests)
               │                                                 │
 opossum_core2/src/ASSERV/asserv_loop.c          pc/test_control.c, simulateur
 control_hal_zynq.c (xil_printf, IPC)            pc/control_hal_pc.c
               │                                                 │
               └──────────────► libs/opossum_control/src ◄───────┘
                                control_loop.c  (séquencement)
                                odometry, motion, speed_constrainer,
                                pid_speed, PWM_Calculator, kalman, kalman_FIFO
```

## Fichiers

| Fichier | Rôle |
|---|---|
| `src/opossum_control.h` | En-tête agrégateur |
| `src/control_loop.c` / `.h` | **Point d'entrée** : consignes/observations, pas rapide (1 ms), pas lent (10 ms), fusion du cap IMU |
| `src/control_types.h` | Types partagés avec tout le firmware (`Position`, `Speed`, `PID_*`, `ESC_Command`, `Set_lidar`, `Set_camera`…), inclus par `opossum_common/common_type.h` |
| `src/control_hal.h` | Services plateforme à fournir (log, `motion_done`, chronométrage) |
| `src/asserv_default.h` | Paramètres du robot : entraxe, roues, réduction, gains PID, limites, tolérances d'arrivée |
| `src/odometry.c` | Vitesses et pose à partir des RPM moteurs |
| `src/motion.c` | Modes position / vitesse / vitesse absolue / bloqué, état `motion_done` |
| `src/speed_constrainer.c` | Limiteurs de vitesse et d'accélération, consignes par roue |
| `src/pid_speed.c`, `src/PWM_Calculator.c` | PID de vitesse par roue → commandes ESC |
| `src/kalman.c`, `src/kalman_FIFO.c` | EKF et FIFO de repropagation des mesures retardées (cf. [KALMAN.md](KALMAN.md)) |
| `pc/control_hal_pc.c` | HAL PC |
| `pc/test_control.c` | Test en boucle fermée sur un modèle simple du robot |
| `CMakeLists.txt` | Build PC : `libopossum_control.a`, `libopossum_control.so`, tests |

## Utilisation

```c
control_init(imu_yaw_seq);
for (;;) {                                   // à chaque tour de boucle
    control_apply_commands(&cmd, imu_yaw, imu_yaw_seq);   // consignes reçues
    if (une ms est écoulée) {
        if (control_fast_step(&inputs))     // odométrie + Kalman
            control_slow_step(au_state, &out);   // trajectoire + PID -> out.motors
    }
}
```

Tout l'état est interne (variables globales des modules) : **une seule instance**
d'asservissement par processus. Pour simuler deux robots, lancer deux processus.

## Firmware Zynq

Le projet Vitis `opossum_core2` compile directement `src/` (dossier lié, cf.
[README racine](../../README.md#7-setup-du-dossier-partagé-entre-les-coeurs)). Ne rien mettre
dans `src/` qui dépende du Zynq : tout ce qui touche au matériel va dans
`opossum_core2/src/ASSERV/`.

## Build PC et tests

```bash
cmake -S libs/opossum_control -B build/opossum_control
cmake --build build/opossum_control
ctest --test-dir build/opossum_control --output-on-failure   # ou build/opossum_control/test_control
```

`test_control` simule chaque roue par un premier ordre et vérifie : arrivée en position
(< 2 cm, < 2°) sur plusieurs trajets dont une rotation pure, commandes nulles sous arrêt
d'urgence, arrêt sur `BLOCK`, et qu'un lidar parfait retardé (0, 90, 200 ms) ne dégrade pas
l'estimation du Kalman (< 1 cm).

## Comportements observés en SIL

- **Glissement après l'arrivée** : à la fin d'un `MOVE`, l'asserv passe en mode `FREE`
  (consigne nulle, PID actif) sans remettre l'intégrale du PID à zéro. Sans frottement
  (modèle du test), l'intégrale accumulée pendant le trajet maintient quelques centaines
  d'unités de commande et le robot glisse lentement (~5 cm/s) jusqu'à ce que l'intégrale se
  vide. Sur le robot, les frottements masquent probablement cet effet. À vérifier avant de
  corriger (`pid_vitesse_reset()` à l'entrée en `FREE` ?).
