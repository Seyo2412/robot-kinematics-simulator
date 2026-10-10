import numpy as np
import matplotlib.pyplot as plt
from src.kinematics import forward_kinematics_3r

link_lengths = [2.0, 2.0, 2.0]
angles_deg = [30, 45, -45]
angles_rad = np.radians(angles_deg)

joint_positions = forward_kinematics_3r(angles_rad, link_lengths)

x_coords = joint_positions[:, 0]
y_coords = joint_positions[:, 1]

plt.figure(figsize = (6, 6))
plt.plot(x_coords, y_coords, "-o", linewidth = 3, markersize = 8, color = "royalblue", label = "Roboterarm")
plt.plot(0, 0, "ks", markersize = 10, label = "Basis")
plt.plot(x_coords[-1], y_coords[-1], "r*", markersize = 12, label = "End-Effektor" )

max_reach = sum(link_lengths) + 0.5
plt.xlim(-max_reach, max_reach)
plt.ylim(-max_reach, max_reach)
plt.grid(True, linestyle = "--", alpha = 0.6)
plt.gca().set_aspect("equal", adjustable = "box")
plt.title(f"3R Roboterarm Konfiguration\nWinkel: {angles_deg}°")
plt.legend()
plt.show()