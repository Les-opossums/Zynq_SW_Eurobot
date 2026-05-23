### README - Calibration du Feedforward (Moteurs M2006 / ESC C610)

Ce document détaille le principe et la procédure de calibration du modèle de friction pour la base holonome.

## Le Principe : Pourquoi un Feedforward ?

Les moteurs DJI M2006 équipés de réducteurs P36 présentent des frottements mécaniques non négligeables. Si l'on compte uniquement sur le correcteur PID pour vaincre cette résistance, le robot subira un retard au démarrage et des erreurs de traînage (lag) importantes lors des variations de consigne.

Le modèle de Feedforward (FF) anticipe l'effort nécessaire pour faire tourner la roue à une vitesse donnée, déchargeant ainsi le PID qui n'a plus qu'à corriger les petites perturbations externes (glissements, irrégularités du terrain).

Le modèle mathématique utilisé pour calculer le courant (PWM) à injecter est le suivant :
$$ I_{cmd} = I_{static} \cdot \text{sgn}(v) + B \cdot v $$

Les paramètres à identifier pour chaque roue sont :
- $I_{static}$ : Le frottement sec. C'est la commande minimale absolue nécessaire pour vaincre l'inertie et mettre la mécanique en mouvement.
- $B$ : Le frottement visqueux. C'est la proportionnalité entre la commande envoyée et la vitesse obtenue une fois le moteur en régime stabilisé.

La routine de calibration exécute une machine à états asynchrone sur le cœur 1 du Zynq. Elle applique différents paliers de courant (positifs et négatifs) sur chaque roue de manière séquentielle et enregistre la vitesse stabilisée mesurée par l'odométrie.

## Prérequis
- Le robot doit être placé sur cales. Les roues doivent tourner librement dans le vide pour isoler les frottements internes des motoréducteurs de l'interaction avec le tapis.
- Un terminal série doit être connecté au port de debug du Zynq pour capter les logs.

## Procédure de Calibration
1. Lancement de la séquence embarquéeDans le terminal de commande du Raspberry Pi, envoie la commande suivante pour déclencher la machine à états : `CALIBFF`  
Le Zynq va automatiquement passer en mode roue libre pour désactiver le PID et commencer à injecter des paliers de courant ciblés sur les roues 1 à 4. L'opération prend une dizaine de secondes par roue.
2. Récupération des donnéesPendant l'exécution, le cœur 1 va afficher sur le port série des lignes au format `FF_CAL wheel I_cmd v_ms`.
Copie l'intégralité de ces lignes de log, depuis le message de démarrage jusqu'à `FF_CALIBRATION_ALL_END`.
Sauvegarde ces données dans un fichier nommé `calib_log.txt` situé dans le dossier `motor/calib/`.
3. Analyse via le script PythonOuvre un terminal sur ton PC, navigue dans le bon répertoire et exécute le script. Ce dernier va parser le fichier texte et effectuer une régression linéaire (séparément pour la marche avant et arrière) pour extraire la pente et l'ordonnée à l'origine.
`cd motor_calib/`
`python calib_motor.py`
4. Mise à jour du firmware ZynqLe script générera une sortie prête à être copiée sous cette forme : `>>> wheel_ff[0] = {120.5f, 35.2f, 0.04f};` 
Copie ces résultats et remplace l'ancienne initialisation de ta structure wheel_ff dans le code d'asservissement.Recompile le projet (make) et flashe le nouveau .elf sur la carte.