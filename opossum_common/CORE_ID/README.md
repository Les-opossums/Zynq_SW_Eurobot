# CORE_ID — Identification du cœur à la compilation

Ce dossier contient la seule brique qui rend possible le partage d'un même code source (`opossum_common`) entre les deux projets Vitis, chacun compilé pour un cœur différent du Zynq-7000 (architecture AMP, pas SMP : deux binaires indépendants, un par CPU).

## Fichier

* **`core_id.h`** (pas de `.c` — uniquement des macros)

## Fonctionnement

`THIS_CORE_ID` est injecté par les **Symbols** du projet Vitis (`-DTHIS_CORE_ID=0` pour le projet `opossum_core1` = CPU0, `-DTHIS_CORE_ID=1` pour `opossum_core2` = CPU1 — cf. étape 6 du [README racine](../../README.md)). Ce fichier :

* Vérifie que `THIS_CORE_ID` est bien défini (`#error` sinon, pour éviter un oubli de configuration silencieux).
* Définit `CORE_ID_CPU0` / `CORE_ID_CPU1` (constantes numériques pures, utilisables dans des `#if`/`#elif` au préprocesseur, avant même l'inclusion des en-têtes).
* Définit `THIS_CORE` (cast typé en `core_owner_t`, cf. [IO_MANAGER](../IO_MANAGER/README.md)), utilisé au runtime pour les comparaisons normales (`if (dev->owner == THIS_CORE)`).

## Deux usages distincts, à ne pas confondre

```c
// Au préprocesseur (avant compilation) : réserve du code entier à un cœur
#if THIS_CORE_ID == CORE_ID_CPU0
    #include "APP_ACTIONNEURS/feetech_Action.h"
#endif

// Au runtime (après compilation) : filtre l'exécution selon le propriétaire d'un device
if (dev->owner != THIS_CORE && dev->owner != CORE_BOTH) {
    continue;
}
```

Le premier évite que du code (ou même un simple `#include`) n'existe dans le binaire de l'autre cœur ; le second permet à une même table de périphériques (compilée dans les deux binaires) de ne s'activer que du côté qui la possède.

## Voir aussi

* [IO_MANAGER](../IO_MANAGER/README.md) — usage runtime (`core_owner_t`, filtrage par `owner`)
* [APP_COM](../APP_COM/README.md) — usage préprocesseur (commandes réservées à un cœur dans `command_list.c`)
* [IPC_MANAGER](../IPC_MANAGER/README.md) — communication entre les deux binaires ainsi séparés
* [opossum_common](../README.md) — vue d'ensemble de l'architecture
