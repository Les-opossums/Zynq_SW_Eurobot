# Timer Manager

Ce dossier contient le gestionnaire de base de temps du système ainsi que l'infrastructure de monitoring des temps d'exécution (marge temps-réel). Il exploite le timer privé du processeur Cortex-A9 (SCU Timer) pour la base milliseconde, et le Global Timer du Cortex-A9 (partagé entre les 2 cœurs) pour la mesure microseconde.

## Fichiers

*   `timer_manager.h` / `.c` : base de temps milliseconde (`Timer_ms1`, `Delay_ms`) et base de temps microseconde (`Timer_us1`, `Timer_us_Init`).
*   `timing_stats.h` : infrastructure générique de mesure de temps d'exécution (`TimingStats`, `T_START`/`T_STOP`, impression), utilisée par CPU0 et CPU1.

## Description

Le `Timer_Manager` configure le périphérique matériel `XScuTimer` en mode rechargement automatique (auto-reload) pour déclencher une interruption matérielle de façon périodique. À chaque expiration du timer, le gestionnaire d'interruption (`Timer_IntrHandler`) incrémente une variable globale. Cette variable sert de référence de temps absolue pour l'ensemble des cœurs et des périphériques (notamment pour les mises à jour non bloquantes comme le rafraîchissement des LEDs WS2812B).

## Fonctionnalités Principales

*   **Base de temps globale (ms) :** Entretien d'un compteur incrémenté chaque milliseconde (`Timer_ms1`).
*   **Base de temps microseconde :** Lecture du Global Timer du Cortex-A9 (`Timer_us1`), pour chronométrer précisément des portions de code (voir `timing_stats.h` ci-dessous).
*   **Attente active :** Fourniture d'une fonction d'attente bloquante (`Delay_ms`) pour des pauses précises.
*   **Intégration centralisée :** La connexion de la routine d'interruption au contrôleur générique GIC s'effectue directement via la fonction `IRQ_Manager_Connect` du système centralisé, évitant ainsi la duplication des configurations d'interruptions.

## Configuration Matérielle

Le timer 1ms est configuré avec les paramètres matériels suivants pour atteindre la période cible de 1 ms :
*   **Périphérique :** `XPAR_SCUTIMER_DEVICE_ID`.
*   **Interruption matérielle :** `XPAR_SCUTIMER_INTR`.
*   **Prédiviseur (Prescaler) :** Configuré à `0x0` via la constante `TIMER_1ms_PRESCALER`.
*   **Valeur de chargement (Load Value) :** Configurée à `332999` via la constante `TIMER_1ms_LOAD_VALUE` (soit 333000 ticks/ms → horloge du timer à 333 MHz).

Le timer microseconde utilise le **Global Timer** (registre unique, physiquement partagé par CPU0 et CPU1, cadencé à la même horloge de 333 MHz, cf `xtime_l.h`). Il n'est démarré/remis à zéro qu'**une seule fois**, côté CPU0, **avant le réveil de CPU1** (dans `main.c`), pour que les deux cœurs partagent la même base de temps.

## Fonctions de l'API

*   `int Timer_Manager_Init(void)` : Initialise l'instance `XScuTimer` (base 1ms), exécute un test de diagnostic (`SelfTest`), configure le rechargement automatique et la valeur de base, puis connecte l'interruption au système avant de démarrer le timer. Retourne `XST_SUCCESS` en cas de réussite ou `XST_FAILURE` en cas d'erreur. Appelé par les deux cœurs (chacun a son propre `XScuTimer` privé).
*   `void Delay_ms(int ms)` : Met en pause l'exécution du code appelant pendant le nombre de millisecondes spécifié. Il s'agit d'une attente active (boucle `while`) comparant la valeur actuelle de `Timer_ms1` avec une valeur de référence capturée au début de l'appel.
*   `extern volatile int Timer_ms1` : Variable globale qualifiée de `volatile` (car modifiée de manière asynchrone par l'interruption) contenant le nombre de millisecondes écoulées. Elle peut être lue librement par l'application pour implémenter des délais non bloquants.
*   `void Timer_us_Init(void)` : Démarre (remet à 0) le Global Timer. **À appeler une seule fois, côté CPU0 uniquement**, avant le réveil de CPU1 — déjà fait dans `main.c`, à ne pas dupliquer ailleurs.
*   `uint32_t Timer_us_Get(void)` / macro `Timer_us1` : Lit le nombre de microsecondes écoulées depuis `Timer_us_Init()`. Utilisable en lecture par les deux cœurs. Boucle (wrap) toutes les ~71 minutes (compteur 32 bits) : sans incidence pour des mesures de durées courtes (les soustractions `uint32_t` restent correctes même en cas de wrap, tant que la durée mesurée est trop courte pour multiple wraps).


## Monitoring des temps d'exécution (`timing_stats.h`)

### Objectif

Mesurer, pour n'importe quelle portion de code (sur CPU0 comme CPU1), le temps d'exécution **min / max / moyen / dernier** en microsecondes, afin de vérifier la marge disponible par rapport à un budget temps-réel (ex : boucle asserv à 1ms, boucle IO_Manager, réception UART, etc.).

### Activer / désactiver

Un seul interrupteur, commun aux deux cœurs, en haut de `timing_stats.h` :

```c
// #define TIMING_MEASURE
```

Décommenter cette ligne active les mesures **et** les impressions `xil_printf` périodiques sur CPU0 et CPU1 (il faut recompiler les deux projets). La laisser commentée désactive tout : aucun coût en flash/CPU (le code de mesure disparaît entièrement à la compilation).

### Ce qui est déjà instrumenté

| Où | Quoi | Cœur |
|---|---|---|
| `IO_Manager_Update()` (`IO_MANAGER/IO_manager.c`) | Chaque périphérique de `IO_DEVICE_TABLE` (GPIO_PS, WS2812B, IMU_BNO085, UART_COMM, CAN_MOTORS...) + le total | CPU0 et CPU1 (générique — tout nouveau driver ajouté à la table, ex. futurs actionneurs, est mesuré automatiquement sans code supplémentaire) |
| `Com_Interpreter_Update()` (`APP_COM/com_interpreter_loop.c`) | Réception + interprétation des commandes reçues par UART | CPU0 |
| `fast_loop()` / `slow_loop()` (`opossum_core2/src/ASSERV/asserv_loop.c`) | Odométrie, Kalman, IMU, FIFO (fast) ; consignes, contraintes, PID+CAN, IPC (slow) | CPU1 |

Impression automatique toutes les ~1 seconde sur chaque cœur (via `App_Loop()` côté CPU0, via `asserv_loop_update()` côté CPU1), sans action requise une fois `TIMING_MEASURE` activé.

Exemple de sortie :

```
--- Timing IO_Manager (CPU0) ---
  GPIO_PS      min=   2 max=   5 avg=   3 last=   3 us (n=1000)
  IMU_BNO085   min=  40 max= 120 avg=  55 last=  48 us (n=1000)
  UART_COMM    min=   1 max=   4 avg=   2 last=   2 us (n=1000)
  io_total     min=  45 max= 128 avg=  62 last=  55 us (n=1000)
  com_uart     min=   0 max=  10 avg=   1 last=   0 us (n=1000)
```

### Ajouter une mesure (ex : futur contrôle d'actionneurs)

Si le nouveau code passe par `IO_DEVICE_TABLE` (comme tous les drivers actuels), **rien à faire** : il sera chronométré automatiquement dès son ajout à la table dans `IO_config.h`.

Pour instrumenter une portion de code spécifique ailleurs (ex : une nouvelle étape dans `slow_loop()`, ou une future boucle de contrôle moteur autre que CAN) :

```c
#include "TIMER_MANAGER/timing_stats.h"

#if defined(TIMING_MEASURE)
static TimingStats ts_mon_etape;   // 1 par étape mesurée, static au fichier
#endif

void ma_fonction(void)
{
#if defined(TIMING_MEASURE)
    T_START(mon_etape);            // capture Timer_us1 dans une variable locale _t_mon_etape
#endif

    // ... code à mesurer ...

#if defined(TIMING_MEASURE)
    T_STOP(mon_etape, ts_mon_etape);  // calcule la durée et met à jour min/max/avg/last
#endif
}
```

Puis, dans une impression périodique (ex : basée sur `ts_trigger_ms`) :

```c
ts_print_one("mon_etape", &ts_mon_etape);
```

Points d'attention :
*   L'identifiant passé à `T_START`/`T_STOP` (ex : `mon_etape`) doit être **unique dans la fonction** : il sert à nommer la variable locale `_t_mon_etape`. Deux mesures séquentielles dans la même fonction doivent utiliser des identifiants différents.
*   Toujours entourer les appels de `#if defined(TIMING_MEASURE)` : les macros/fonctions de `timing_stats.h` n'existent pas quand le monitoring est désactivé (pour un coût nul en production).
*   Pour un module destiné à être appelé depuis un autre fichier (comme `IO_Manager_PrintTiming()` ou `Com_Interpreter_PrintTiming()`), préférer une fonction d'impression **toujours déclarée** dans le `.h`, mais dont le corps est vide (no-op) si `TIMING_MEASURE` est désactivé — cela évite de propager des `#ifdef` dans le code appelant.
*   Déclencher l'impression périodique avec `ts_trigger_ms(period_ms, &ma_variable_last_ms)` plutôt qu'un compteur de ticks manuel, pour rester cohérent entre les deux cœurs.
