import numpy as np
import matplotlib.pyplot as plt
from src.kinematics import forward_kinematics_3r, inverse_kinematics_3r

# 1. Arm-Konfiguration
link_lengths = [2.0, 1.5, 1.0]

# 2. Zielpunkt des Greifers
target_x = 2.5
target_y = 2.0
target_phi_deg = 0.0
target_phi_rad = np.radians(target_phi_deg)

# 3. Inverse Kinematik (Winkel berechnen)
calculated_angles = inverse_kinematics_3r(target_x, target_y, target_phi_rad, link_lengths, elbow_up=True)

# 4. Vorwaertskinematik (Punkte berechnen)
joints = forward_kinematics_3r(calculated_angles, link_lengths)

xs = joints[:, 0]
ys = joints[:, 1]

# 5. Debug-Ausgabe im Terminal
print("Gelenkpunkte (X, Y):")
for i, (x, y) in enumerate(joints):
    print(f"  Punkt {i}: ({x:.3f}, {y:.3f})")

print(f"\nSoll-Ziel:     ({target_x:.3f}, {target_y:.3f})")
print(f"Greifer (Ist): ({xs[-1]:.3f}, {ys[-1]:.3f})")

# 6. Plot erstellen
plt.figure(figsize=(7, 7))

# Blaue Armglieder mit dicken Gelenkpunkten
plt.plot(xs, ys, '-o', color='royalblue', linewidth=4, markersize=9, label='Roboterarm')

# Basis (schwarzes Viereck)
plt.plot(0, 0, 'ks', markersize=12, label='Basis (0,0)')

# Soll-Ziel (gruenes Kreuz) vs. Ist-Position (roter Stern)
plt.plot(target_x, target_y, 'gx', markersize=14, markeredgewidth=3, label='Soll-Ziel')
plt.plot(xs[-1], ys[-1], 'r*', markersize=12, label='End-Effektor')

# Plot-Grenzen und Gitternetz
max_reach = sum(link_lengths) + 0.5
plt.xlim(-max_reach, max_reach)
plt.ylim(-max_reach, max_reach)
plt.gca().set_aspect('equal')
plt.grid(True, linestyle='--', alpha=0.6)
plt.title("Inverse Kinematik: 3R Roboterarm", fontsize=12)
plt.legend(loc='upper left')

plt.show()