import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from src.kinematics import RobotArm6DOF
#from scr.trajectory import plan_quintic_trajectory

robot = RobotArm6DOF()

q_start_deg = [0, -90, 0, -90, 0, 0]
q_goal_deg  = [45, -45, 90, -135, -90, 30]

q_start = np.radians(q_start_deg)
q_goal  = np.radians(q_goal_deg)

trajectory, _ = plan_quintic_trajectory(q_start, q_goal, duration=3.0, steps=90)

fig = plt.figure(figsize=(9, 8))
ax = fig.add_subplot(111, projection='3d')

limit = 0.85
ax.set_xlim(-limit, limit)
ax.set_ylim(-limit, limit)
ax.set_zlim(0, limit * 1.1)

ax.set_xlabel('X [m]')
ax.set_ylabel('Y [m]')
ax.set_zlabel('Z [m]')
ax.set_title("3D 6-DOF Roboter: Dynamische Trajektorienfahrt", fontsize=12)

arm_line, = ax.plot([], [], [], '-o', color='royalblue', linewidth=4, markersize=7, label='Roboterarm')
trace_line, = ax.plot([], [], [], 'r--', alpha=0.6, linewidth=1.5, label='3D-Greiferspur')
ax.scatter([0], [0], [0], color='black', s=100, marker='s', label='Basis')
ax.legend(loc='upper right')

trace_x, trace_y, trace_z = [], [], []

def init():
    arm_line.set_data([], [])
    arm_line.set_3d_properties([])
    trace_line.set_data([], [])
    trace_line.set_3d_properties([])
    return arm_line, trace_line

def update(frame):
    global trace_x, trace_y, trace_z
    if frame == 0:
        trace_x.clear()
        trace_y.clear()
        trace_z.clear()

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

ani = FuncAnimation(fig, update, frames=len(trajectory), init_func=init, interval=33, blit=False, repeat=True)

plt.show()