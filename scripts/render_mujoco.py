import os
import sys
import numpy as np
import mujoco
from pathlib import Path
from PIL import Image
from src.trajectory import plan_quintic_trajectory

sys.path.append(str(Path(__file__).resolve().parent.parent))

os.makedirs("assets", exist_ok=True)

XML_MODEL = """
<mujoco model="ur5e_studio">
  <compiler angle="radian" coordinate="local"/>
  <option timestep="0.005" integrator="RK4"/>

  <visual>
    <headlight ambient="0.3 0.3 0.35" diffuse="0.8 0.8 0.8" specular="0.2 0.2 0.2"/>
    <rgba haze="0.15 0.15 0.18 1"/>
    <global offwidth="960" offheight="720"/>
    <quality shadowsize="4096"/>
  </visual>

  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.12 0.12 0.15" rgb2="0.05 0.05 0.07" width="512" height="512"/>
    <texture name="grid" type="2d" builtin="checker" width="512" height="512" rgb1="0.14 0.14 0.16" rgb2="0.18 0.18 0.21"/>
    <material name="grid_mat" texture="grid" texrepeat="6 6" reflectance="0.2" roughness="0.4"/>
    
    <material name="metal_dark" rgba="0.22 0.23 0.25 1" specular="0.8" shininess="0.7" reflectance="0.3"/>
    <material name="metal_silver" rgba="0.85 0.87 0.90 1" specular="0.9" shininess="0.9" reflectance="0.5"/>
    <material name="ur_blue" rgba="0.05 0.45 0.78 1" specular="0.6" shininess="0.5" reflectance="0.2"/>
    <material name="tool_accent" rgba="0.95 0.25 0.3 1" specular="0.8" shininess="0.6"/>
  </asset>

  <worldbody>
    <light directional="true" pos="-0.5 1.5 3.0" dir="0.2 -0.5 -1.0" castshadow="true" diffuse="0.9 0.9 0.92" specular="0.3 0.3 0.3"/>
    <light directional="false" pos="1.5 -1.5 2.0" diffuse="0.4 0.4 0.45" specular="0.1 0.1 0.1"/>

    <!-- Studio-Bodenplatte mit Reflexion -->
    <geom name="floor" type="plane" size="2 2 0.1" material="grid_mat"/>

    <!-- Roboter Basis-Sockel -->
    <body name="base" pos="0 0 0">
      <geom name="pedestal" type="cylinder" size="0.12 0.03" pos="0 0 0.03" material="metal_dark"/>
      <geom name="base_cap" type="cylinder" size="0.075 0.03" pos="0 0 0.09" material="metal_silver"/>

      <!-- Schulter -->
      <body name="shoulder_link" pos="0 0 0.12">
        <joint name="shoulder_pan" type="hinge" axis="0 0 1"/>
        <geom type="cylinder" size="0.06 0.06" pos="0 0 0.03" material="ur_blue"/>
        
        <!-- Oberarm -->
        <body name="upper_arm_link" pos="0 0.12 0.06">
          <joint name="shoulder_lift" type="hinge" axis="0 1 0"/>
          <geom type="capsule" size="0.05 0.20" pos="0 0 0.21" material="metal_silver"/>
          <geom type="cylinder" size="0.055 0.05" pos="0 0 0" quat="0.707 0.707 0 0" material="ur_blue"/>

          <!-- Unterarm -->
          <body name="forearm_link" pos="0 -0.09 0.425">
            <joint name="elbow" type="hinge" axis="0 1 0"/>
            <geom type="capsule" size="0.04 0.18" pos="0 0 0.19" material="metal_silver"/>
            <geom type="cylinder" size="0.048 0.04" pos="0 0 0" quat="0.707 0.707 0 0" material="ur_blue"/>

            <!-- Handgelenk 1 -->
            <body name="wrist_1_link" pos="0 0 0.392">
              <joint name="wrist_1" type="hinge" axis="0 1 0"/>
              <geom type="cylinder" size="0.038 0.04" pos="0 0.07 0" quat="0.707 0.707 0 0" material="metal_dark"/>

              <!-- Handgelenk 2 -->
              <body name="wrist_2_link" pos="0 0.095 0">
                <joint name="wrist_2" type="hinge" axis="0 0 1"/>
                <geom type="cylinder" size="0.035 0.04" pos="0 0 0.04" material="ur_blue"/>

                <!-- Handgelenk 3 & Flansch -->
                <body name="wrist_3_link" pos="0 0 0.082">
                  <joint name="wrist_3" type="hinge" axis="0 1 0"/>
                  <geom type="cylinder" size="0.032 0.02" pos="0 0.03 0" quat="0.707 0.707 0 0" material="metal_silver"/>
                  <!-- Tool / Greiferspitze -->
                  <geom type="cylinder" size="0.015 0.03" pos="0 0.06 0" quat="0.707 0.707 0 0" material="tool_accent"/>
                </body>
              </body>
            </body>
          </body>
        </body>
      </body>
    </body>
  </worldbody>
</mujoco>
"""

model = mujoco.MjModel.from_xml_string(XML_MODEL)
data = mujoco.MjData(model)
renderer = mujoco.Renderer(model, width=960, height=720)

camera = mujoco.MjvCamera()
camera.lookat = [0.0, 0.0, 0.35]
camera.distance = 1.9
camera.elevation = -22.0
camera.azimuth = 135.0

waypoints_deg = [
    [  0,  -80,   90,  -100,  -90,   0],  # Home
    [-45,  -45,  115,  -160,  -90,  15],  # Pick (links)
    [-15,  -70,   80,  -100,  -90,   0],  # Anheben
    [ 45,  -45,  115,  -160,  -90, -15],  # Place (rechts)
    [  0,  -80,   90,  -100,  -90,   0]   # Home
]

trajectory_list = []
segment_steps = 20

for k in range(len(waypoints_deg) - 1):
    q_start = np.radians(waypoints_deg[k])
    q_goal  = np.radians(waypoints_deg[k + 1])
    seg, _ = plan_quintic_trajectory(q_start, q_goal, duration=1.0, steps=segment_steps)
    if k > 0:
        trajectory_list.append(seg[1:])
    else:
        trajectory_list.append(seg)

trajectory = np.vstack(trajectory_list)

# 5. Rendern & GIF erzeugen
print(f"Rendere {len(trajectory)} Studio-Frames mit MuJoCo PBR-Shading...")
rendered_frames = []

for q in trajectory:
    # Gelenkpositionen in MuJoCo setzen
    data.qpos[:6] = q
    mujoco.mj_forward(model, data)
    
    # Offscreen-Frame rendern
    renderer.update_scene(data, camera=camera)
    frame_pixels = renderer.render()
    rendered_frames.append(Image.fromarray(frame_pixels))

# Speichern als hochoptimiertes GIF
gif_path = "assets/demo_3d.gif"
rendered_frames[0].save(
    gif_path,
    save_all=True,
    append_images=rendered_frames[1:],
    duration=40,  # ~25 FPS
    loop=0,
    optimize=True
)

print(f"Fertig! MuJoCo Studio-GIF erfolgreich unter '{gif_path}' gespeichert.")