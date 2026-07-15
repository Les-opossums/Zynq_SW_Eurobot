# Guide de Configuration : `IO_config.h`

Ce fichier est le **cœur névralgique** de la configuration matérielle de notre robot.

Il utilise une architecture par **macros empilées** qui permet de partager un seul et même fichier de configuration entre le **CPU0** et le **CPU1**, sans générer d'erreurs de compilation ou de variables fantômes (*linker errors*).

> ⚠️ **Règle d'or :** Vous ne devez jamais écrire de code matériel (`XGpioPs_...`) dans votre logique métier. Tout passe par ce fichier de configuration.

---

# 🧠 Comment ça marche ?

Le compilateur de chaque projet Vitis (CPU0 ou CPU1) possède une macro globale appelée `THIS_CORE` (définie dans les options du compilateur, **pas dans ce fichier**).

Grâce aux directives :

```c
#if THIS_CORE == CORE_...
```

le préprocesseur C lit ce fichier et ne conserve **que** les entrées et sorties qui concernent le processeur en cours de compilation.

Les IO appartenant à l'autre processeur sont ignorées et remplacées par du vide.

---

# 🛠️ Comment ajouter une nouvelle IO (pas à pas)

Imaginons que vous vouliez ajouter un capteur nommé `Capteur_Couleur` (connecté sur la broche **EMIO 60**) qui sera lu par le **CPU1**.

---

## Étape 1 : Localiser la bonne section

Ouvrez `IO_config.h` et descendez jusqu'à la section correspondant au cœur qui va gérer cette IO.

- `--- IO PARTAGÉES ---` (pour les IO utilisées par les deux CPU, comme l'arrêt d'urgence)
- `--- IO SPÉCIFIQUES AU CPU 0 ---`
- `--- IO SPÉCIFIQUES AU CPU 1 ---` ← **C'est ici dans notre exemple.**

---

## Étape 2 : Déclarer la variable et la macro-ligne

Dans le bloc :

```c
#if THIS_CORE == CORE_CPU1
```

ajoutez :

1. la déclaration `extern int`
2. la macro `ROW_...` qui décrit la configuration de la broche.

```c
#if THIS_CORE == CORE_CPU1
    // ... autres variables ...

    extern int Capteur_Couleur;

    #define ROW_CAPTEUR_COULEUR \
        {60, IO_DIR_INPUT, &Capteur_Couleur, CORE_CPU1},

#else

    // ... autres macros vides ...

    #define ROW_CAPTEUR_COULEUR

#endif
```

### Syntaxe d'une ligne de configuration

```c
{ PIN, DIRECTION, POINTEUR_VARIABLE, PROPRIETAIRE }
```

| Champ | Description |
|--------|-------------|
| `PIN` | Numéro de la broche EMIO (à partir de 54). |
| `DIRECTION` | `IO_DIR_INPUT` (capteur, bouton) ou `IO_DIR_OUTPUT` (LED, moteur...). |
| `POINTEUR_VARIABLE` | Adresse de la variable (`&MaVariable`). |
| `PROPRIETAIRE` | `CORE_CPU0`, `CORE_CPU1` ou `CORE_BOTH`. |

---

## Étape 3 : Ajouter la ligne au tableau final

En bas du fichier, trouvez la macro `IO_CONFIG_TABLE` et ajoutez simplement votre nouvelle macro.

L'ordre n'a pas d'importance.

```c
#define IO_CONFIG_TABLE { \
    ROW_AU \
    ROW_LEASH \
    ROW_TEAM \
    ROW_CAPTEUR_COULEUR \
}
```

---

## Étape 4 : Instancier la variable dans votre code métier

Le gestionnaire d'IO sait maintenant qu'il doit mettre à jour une variable.

Il faut cependant que cette variable existe réellement en mémoire.

Dans votre `main.c` (ou n'importe quel fichier `.c` de votre application) :

```c
#include "IO_config.h"

// Instanciation de la variable
int Capteur_Couleur = 0;

int main(void)
{
    // Initialisation...

    while (1)
    {
        IO_Manager_Update();

        if (Capteur_Couleur == 1)
        {
            // Logique du robot
        }
    }
}
```

---

# 🚨 Dépannage fréquent

## ❌ Erreur `multiple definition of ...`

Vous avez probablement :

- oublié le mot-clé `extern` dans `IO_config.h`, **ou**
- initialisé la variable (`= 0`) directement dans le fichier `.h`.

---

## ❌ Erreur `undefined reference to ...`

Vous avez bien déclaré :

```c
extern int MaVariable;
```

mais vous avez oublié de créer la variable dans un fichier `.c` :

```c
int MaVariable = 0;
```

---

## ❌ L'IO ne se met pas à jour

Vérifiez que :

```c
IO_Manager_Update();
```

est bien appelé régulièrement dans votre boucle principale (`while(1)`) ou dans votre timer système.

---

# Résumé

Pour ajouter une nouvelle IO :

1. Choisir le bon bloc (`CORE_CPU0`, `CORE_CPU1` ou partagé).
2. Ajouter un `extern int`.
3. Créer une macro `ROW_...`.
4. Ajouter cette macro dans `IO_CONFIG_TABLE`.
5. Créer la variable (`int ... = 0;`) dans un fichier `.c`.
6. Appeler `IO_Manager_Update()` régulièrement.

Une fois ces six étapes réalisées, votre nouvelle IO est automatiquement prise en charge par le gestionnaire.