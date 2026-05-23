import numpy as np
from scipy.stats import linregress
import sys
import os

# --- NOUVEAU : Calcul du chemin absolu du fichier log ---
# __file__ contient le chemin de calib_motor.py
script_dir = os.path.dirname(os.path.abspath(__file__))
log_file_path = os.path.join(script_dir, "calib_log.txt")

# Charger les données (parser le log série)
data = {}
with open(log_file_path) as f:  # On utilise le chemin absolu ici
    for line in f:
        if not line.startswith("FF_CAL "): continue
        parts = line.split()
        if len(parts) != 4: continue
        wheel_id = int(parts[1])
        I_cmd    = float(parts[2])
        v_ms     = float(parts[3])
        data.setdefault(wheel_id, []).append((I_cmd, v_ms))

for wid, pts in sorted(data.items()):
    pts = np.array(pts)
    I_cmd = pts[:, 0]
    v_ms  = pts[:, 1]

    # Séparer positif et négatif
    mask_pos = v_ms > 0.01
    mask_neg = v_ms < -0.01

    results = {}
    for mask, label in [(mask_pos, "pos"), (mask_neg, "neg")]:
        if mask.sum() < 3:
            continue
        I_fit = I_cmd[mask]
        v_fit = v_ms[mask]
        # Modèle : I = I_static + B * v  (pour v > 0)
        # Régression linéaire : I = B*v + I_static
        slope, intercept, r, _, _ = linregress(v_fit, I_fit)
        results[label] = {"B": slope, "I_static": abs(intercept)}
        print(f"Roue {wid} [{label}]: B={slope:.1f} I_static={abs(intercept):.1f} R²={r**2:.4f}")

    # Moyenne pos/neg pour les paramètres finaux
    if results: # Sécurité au cas où il manque des données
        B_mean = np.mean([r["B"] for r in results.values()])
        Is_mean = np.mean([r["I_static"] for r in results.values()])
        print(f">>> wheel_ff[{wid}] = {{{Is_mean:.1f}f, {B_mean:.1f}f, 0.04f}};")
        print()