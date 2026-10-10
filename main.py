import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as FuncAnimation
from src.kinematics import forward_kinematics_3r, inverse_kinematics_3r
from src.trajectory import plan_quintic_trajectory

link_lengths = [2.0, 1.5, 1.0]
start_angles = np.radians([10, 20, 10])

target_x = 1.5
target_y = 2.5
target_phi_rad = np.radians(45)

goal_angles = inverse_kinematics_3r(target_x, target_y, target_phi_rad, link_lengths, elbow_up=True)

trajectory_angles, time_vector = plan_quintic_trajectory(start_angles, goal_angles, duration=2.5, steps=80)

fig, ax = plt.subplots(figsize=(7, 7))
max_reach = sum(link_lengths) + 0.5
ax.set_xlim(-max_reach, max_reach)
ax.set_ylim(-max_reach, max_reach)
ax.set_aspect('equal')
ax.grid(True, linestyle='--', alpha=0.6)
ax.set_title("3R Roboterarm: Sanfte Zielanfahrt (Trajektorie)", fontsize=12)

line, = ax.plot([], [], 'o-', color='royalblue', linewidth=4,  markersize=8, label = "Roboterarm")
ax.plot(0, 0, "ks", markersize = 12, label = "Basis")
ax.plot(target_x, target_y, "gx", markersize = 14, markeredgewidth = 3, label = "Soll-Ziel")
trace_x, trace_y = [], []
trace_line, = ax.plot([], [], 'r--', alpha=0.5, linewidth=1.5, label = "Bahn Greiferspitze")
ax.legend(loc="upper left")

def init():
    line.set_data([], [])
    trace_line.set_data([], [])
    return line, trace_line

def update(frame):

    if frame == 0:
        trace_x.clear()
        trace_y.clear()

    current_q = trajectory_angles[frame]
    joints = forward_kinematics_3r(current_q, link_lengths)

    xs = joints[:, 0]
    ys = joints[:, 1]

    line.set_data(xs, ys)
    trace_x.append(xs[-1])
    trace_y.append(ys[-1])
    trace_line.set_data(trace_x, trace_y)

    return line, trace_line

ani = FuncAnimation.FuncAnimation(fig, update, frames = len(trajectory_angles), init_func=init, interval = 30, blit=True, repeat=True)

plt.show()