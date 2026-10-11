import os
import sys
from pathlib import Path
import numpy as np
import pyvista as pv
from src.kinematics import RobotArm6DOF
from src.trajectory import plan_quintic_trajectory

sys.path.append(str(Path(__file__).resolve().parent.parent))

os.makedirs("assets", exist_ok=True)

robot = RobotArm6DOF()

waypoints_deg = [
    [  0,  -80,   90,  -100,  -90,   0],  # 0: Home / Ruhestellung
    [-40,  -45,  115,  -160,  -90,  15],  # 1: Pick-Position (tief links)
    [-10,  -70,   80,  -100,  -90,   0],  # 2: Clearance / Anheben
    [ 40,  -45,  115,  -160,  -90, -15],  # 3: Place-Position (tief rechts)
    [  0,  -80,   90,  -100,  -90,   0]   # 4: Zurueck zur Home-Stellung
]

trajectory_list = []
segment_frames = 22 

for k in range(len(waypoints_deg) - 1):
    q_start = np.radians(waypoints_deg[k])
    q_goal  = np.radians(waypoints_deg[k + 1])
    seg, _ = plan_quintic_trajectory(q_start, q_goal, duration=1.2, steps=segment_frames)
    
    if k > 0:
        trajectory_list.append(seg[1:])
    else:
        trajectory_list.append(seg)

trajectory = np.vstack(trajectory_list)

plotter = pv.Plotter(off_screen=True, window_size=[960, 720])
plotter.set_background("#18181b")

grid = pv.Plane(center=(0, 0, -0.01), direction=(0, 0, 1), i_size=1.6, j_size=1.6)
plotter.add_mesh(grid, color="#27272a", show_edges=True, edge_color="#3f3f46", opacity=0.8)

base_disc = pv.Cylinder(center=(0, 0, 0.02), direction=(0, 0, 1), radius=0.12, height=0.04)
plotter.add_mesh(base_disc, color="#52525b", pbr=True, roughness=0.3, metallic=0.7)

pts_initial = robot.forward_kinematics(trajectory[0])
link_actors, joint_actors = [], []
cyl_radius, joint_radius = 0.036, 0.046

for i in range(len(pts_initial) - 1):
    p1, p2 = pts_initial[i], pts_initial[i + 1]
    
    sphere = pv.Sphere(radius=joint_radius, center=p1)
    joint_actors.append(plotter.add_mesh(sphere, color="#0284c7", pbr=True, roughness=0.3, metallic=0.6))
    
    cyl = pv.Cylinder(center=(p1 + p2)/2.0, direction=p2 - p1, radius=cyl_radius, height=np.linalg.norm(p2 - p1))
    color = "#38bdf8" if i % 2 == 0 else "#f8fafc"
    link_actors.append(plotter.add_mesh(cyl, color=color, pbr=True, roughness=0.25, metallic=0.8))

tool_sphere = pv.Sphere(radius=0.035, center=pts_initial[-1])
tool_actor = plotter.add_mesh(tool_sphere, color="#f43f5e", pbr=True, roughness=0.2, metallic=0.9)

plotter.camera_position = [
    (1.45, -1.35, 1.15),
    (0.0, 0.0, 0.30),
    (0.0, 0.0, 1.0)
]

gif_path = "assets/demo_3d.gif"
print(f"Rendere {len(trajectory)} Frames für realistischen Arbeitszyklus ...")
plotter.open_gif(gif_path, fps=24)

trace_points = []
for q in trajectory:
    pts = robot.forward_kinematics(q)
    trace_points.append(pts[-1])
    
    for i in range(len(pts) - 1):
        p1, p2 = pts[i], pts[i + 1]
        new_sphere = pv.Sphere(radius=joint_radius, center=p1)
        joint_actors[i].mapper.SetInputData(new_sphere)
        
        vec = p2 - p1
        dist = np.linalg.norm(vec)
        if dist > 1e-4:
            new_cyl = pv.Cylinder(center=(p1 + p2)/2.0, direction=vec, radius=cyl_radius, height=dist)
            link_actors[i].mapper.SetInputData(new_cyl)
            
    new_tool = pv.Sphere(radius=0.035, center=pts[-1])
    tool_actor.mapper.SetInputData(new_tool)
    
    if len(trace_points) > 1:
        trace_poly = pv.Spline(np.array(trace_points), n_points=len(trace_points) * 2)
        plotter.add_mesh(trace_poly, color="#fb7185", line_width=4, name="trace")
        
    plotter.write_frame()

plotter.close()
print(f"Fertig! Neuer Bewegungszyklus unter '{gif_path}' gespeichert.")