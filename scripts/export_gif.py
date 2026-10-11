import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from src.kinematics import RobotArm6DOF
from src.trajectory import plan_quintic_trajectory

os.makedirs("assets", exist_ok=True)

robot = RobotArm6DOF()

q_start = np.radians([0, -90, 0, -90, 0, 0])
q_goal  = np.radians([45, -45, 90, -135, -90, 30])

frames_count = 60
trajectory, _ = plan_quintic_trajectory(q_start, q_goal, duration=2.5, steps=frames_count)

fig = plt.figure(figsize=(7, 6), dpi=100)
ax = fig.add_subplot(111, projection='3d')

limit = 0.85
ax.set_xlim(-limit, limit)
ax.set_ylim(-limit, limit)
ax.set_zlim(0, limit * 1.1)

ax.set_xlabel('X [m]')
ax.set_ylabel('Y [m]')
ax.set_zlabel('Z [m]')
ax.set_title("3D 6-DOF Trajectory (UR5)", fontsize=11)

arm_line, = ax.plot([], [], [], '-o', color='royalblue', linewidth=3.5, markersize=6, label='UR5 Manipulator')
trace_line, = ax.plot([], [], [], 'r--', alpha=0.7, linewidth=1.5, label='End-Effector Path')
ax.scatter([0], [0], [0], color='black', s=80, marker='s')
ax.legend(loc='upper right', fontsize=8)

trace_x, trace_y, trace_z = [], [], []

def init():
    arm_line.set_data([], [])
    arm_line.set_3d_properties([])
    trace_line.set_data([], [])
    trace_line.set_3d_properties([])
    return arm_line, trace_line

def update(frame):
    current_q = trajectory[frame]
    pts = robot.forward_kinematics(current_q)
    
    xs, ys, zs = pts[:, 0], pts[:, 1], pts[:, 2]

    arm_line.set_data(xs, ys)
    arm_line.set_3d_properties(zs)

    trace_x.append(xs[-1])
    trace_y.append(ys[-1])
    trace_z.append(zs[-1])
    trace_line.set_data(trace_x, trace_y)
    trace_line.set_3d_properties(trace_z)

    return arm_line, trace_line

ani = FuncAnimation(fig, update, frames=frames_count, init_func=init, blit=False)

print("Exportiere GIF nach assets/demo_3d.gif ... (dauert ca. 10-15 Sekunden)")
writer = PillowWriter(fps=24)
ani.save("assets/demo_3d.gif", writer=writer)
print("Fertig! GIF erfolgreich erstellt.")