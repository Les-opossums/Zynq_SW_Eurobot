# APP_ACTIONNEURS — Logique applicative des pinces (CPU0)

Ce dossier contient la machine à états qui pilote les 8 pinces du robot (gros servo de levage, deux petits servos de clapet, pompe à vide double + électrovannes) au-dessus du [driver FEETECH](../../../opossum_common/IO_MANAGER/DRIVER_FEETECH/README.md). Spécifique à CPU0 (`opossum_core1`, cf. [README du projet](../../README.md)) — volontairement **hors** de `opossum_common` puisque ce n'est pas partagé avec CPU1.

## Fichiers

* **`feetech_Action.c` / `feetech_Action.h`**

## Modèle de données

* `Pince_t` — état complet d'une pince : positions calibrées (`Gros_Servo_Pos_t`, `Petit_Servo_Pos_t`), état des pompes (`Pump_t`, moyenne glissante de courant), machine à états (`action_step`), commande en cours (`Pince_Command_t`), watchdog anti-blocage.
* `robot_pinces[NBR_PINCES]` (8 pinces) — initialisées par `Init_Pinces_Loop()` avec leurs IDs matériels (`id_gros`/`id_droite`/`id_gauche`/`id_pump`) et leurs positions calibrées (constantes `PINCE_n_*` en tête de `feetech_Action.h`).

## Séquences (machine à états, `pince_action_loop`)

| Plage d'étapes | Séquence |
|---|---|
| 0 | Idle |
| 10-2000-20-22 | Ramasser un/deux objet(s) (descente avec overshoot, détection de contact par couple, confirmation par variation de courant pompe) |
| 100-102 | Remonter la pince |
| 200-215 | Lâcher un/deux objet(s) (retourner) |
| 300-312 | Déposer un objet |
| 400-413 | Déposer un objet et retourner l'autre en simultané |
| 500-506 | **Séquence d'abandon/erreur** — coupe les pompes, ouvre les électrovannes, remonte en sécurité (déclenchée par timeout ou par le watchdog) |
| 600 | Arrêt forcé immédiat (pompes coupées, valves ouvertes, pas de feedback) |
| 700-702 / 800-802 | Pousser / revenir de la position "pousser" |
| 900-902 | Approche en position basse |

Chaque séquence complétée ou abandonnée envoie un message `PINCEFEEDBACK <id_pince> <code_commande> <succes_gauche> <succes_droit>` sur la console (repris par la stratégie haut niveau).

## Sécurité

* **Watchdog** : si une pince reste plus de 4 s sur la même étape (hors idle/erreur), elle est forcée vers la séquence d'abandon (500).
* **Arrêt d'urgence** : `AU_pinces()` réinitialise instantanément toutes les pinces à l'état idle. Appelée depuis `core0_loop.c` sur front montant de `AU_state` (cf. [DRIVER_PS_GPIO](../../../opossum_common/IO_MANAGER/DRIVER_PS_GPIO/README.md)) — les moteurs CAN sont coupés côté CPU1 via IPC, ceci fait l'équivalent côté servos/pompes.

## Commandes (cf. [APP_COM](../../../opossum_common/APP_COM/README.md))

| Commande | Rôle |
|---|---|
| `STSSEND id reg valeur` / `STSGET id reg` | Écriture/lecture directe d'un registre servo, protocole STS |
| `SCSSEND id reg valeur` / `SCSGET id reg` | Idem, protocole SCS |
| `PINCE id commande param` | Déclenche une séquence sur une pince (`id=10` + `commande=0` + `param=0` = arrêt forcé de toutes les pinces) |
| `PINCEDEBUG 0/1` | Ramasser/lâcher simultanément sur deux pinces de test |
| `SET_PINCE id type param valeur` | Recalibre une position à chaud (sans reflash) |

Ces commandes ne sont enregistrées que sur CPU0 (`#if THIS_CORE_ID == CORE_ID_CPU0` dans `command_list.c`, cf. [CORE_ID](../../../opossum_common/CORE_ID/README.md)).

## Calibration

`Pump_Calibration_Loop()` : outil de calibration manuelle (non appelé automatiquement) affichant en continu le courant brut et lissé (moyenne glissante sur 50 échantillons) d'une pompe donnée (`CALIB_PUMP_ID`), pour déterminer les seuils `CURRENT_THRESHOLD_*` en tête de `feetech_Action.h`.

## Voir aussi

* [DRIVER_FEETECH](../../../opossum_common/IO_MANAGER/DRIVER_FEETECH/README.md) — driver bas niveau (protocole, bus série)
* [opossum_core1](../../README.md) — vue d'ensemble du projet CPU0
* [APP_COM](../../../opossum_common/APP_COM/README.md) — interpréteur et table des commandes
* [DRIVER_PS_GPIO](../../../opossum_common/IO_MANAGER/DRIVER_PS_GPIO/README.md) — source de `AU_state`
