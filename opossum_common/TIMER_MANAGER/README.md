# Timer Manager

Ce dossier contient le gestionnaire de base de temps du système. Il exploite le timer privé du processeur Cortex-A9 (SCU Timer) pour générer une horloge système globale avec une résolution d'une milliseconde.

## Description

Le `Timer_Manager` configure le périphérique matériel `XScuTimer` en mode rechargement automatique (auto-reload) pour déclencher une interruption matérielle de façon périodique. À chaque expiration du timer, le gestionnaire d'interruption (`Timer_IntrHandler`) incrémente une variable globale. Cette variable sert de référence de temps absolue pour l'ensemble des cœurs et des périphériques (notamment pour les mises à jour non bloquantes comme le rafraîchissement des LEDs WS2812B).

## Fonctionnalités Principales

*   **Base de temps globale :** Entretien d'un compteur incrémenté chaque milliseconde (`Timer_ms1`).
*   **Attente active :** Fourniture d'une fonction d'attente bloquante (`Delay_ms`) pour des pauses précises.
*   **Intégration centralisée :** La connexion de la routine d'interruption au contrôleur générique GIC s'effectue directement via la fonction `IRQ_Manager_Connect` du système centralisé, évitant ainsi la duplication des configurations d'interruptions.

## Configuration Matérielle

Le timer est configuré avec les paramètres matériels suivants pour atteindre la période cible de 1 ms :
*   **Périphérique :** `XPAR_SCUTIMER_DEVICE_ID`.
*   **Interruption matérielle :** `XPAR_SCUTIMER_INTR`.
*   **Prédiviseur (Prescaler) :** Configuré à `0x0` via la constante `TIMER_1ms_PRESCALER`.
*   **Valeur de chargement (Load Value) :** Configurée à `332999` via la constante `TIMER_1ms_LOAD_VALUE`.

## Fonctions de l'API

L'interface du Timer Manager expose les éléments suivants pour la couche applicative :

*   `int Timer_Manager_Init(void)` : Initialise l'instance `XScuTimer`, exécute un test de diagnostic (`SelfTest`), configure le rechargement automatique et la valeur de base, puis connecte l'interruption au système avant de démarrer le timer. Retourne `XST_SUCCESS` en cas de réussite ou `XST_FAILURE` en cas d'erreur.
*   `void Delay_ms(int ms)` : Met en pause l'exécution du code appelant pendant le nombre de millisecondes spécifié. Il s'agit d'une attente active (boucle `while`) comparant la valeur actuelle de `Timer_ms1` avec une valeur de référence capturée au début de l'appel.
*   `extern volatile int Timer_ms1` : Variable globale qualifiée de `volatile` (car modifiée de manière asynchrone par l'interruption) contenant le nombre de millisecondes écoulées. Elle peut être lue librement par l'application pour implémenter des délais non bloquants.