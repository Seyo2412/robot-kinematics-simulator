# Robot Kinematics Simulator (2D & 3D)

 A modular kinematics and trajectory simulation tool for multi-joint robotic arms built with Python, NumPy, and Matplotlib.

 ![3D Robot Simulation Demo](assets/demo_3d.gif)

## Features
 - **Kinematics Engine:** Forward kinematics for planar 3R and spatial 6-DOF manipulators (UR5 parameters via standard Denavit-Hartenberg matrices).
 - **Inverse Kinematics:** Analytical IK solver for planar 3-DOF arms with geometric workspace boundary checks.
 - **Trajectory Planning:** Jerk-limited, C2-continuous motion profiles based on quintic polynomial interpolation.
 - **High-Fidelity Rendering:** Interactive 3D visualization and CAD-grade rendering with metallic shaders, volumetric links, and trace projection using PyVista & VTK.
 - **Automated Testing:** Fully automated unit test suite with `pytest` for kinematic precision verification.

## Installation

 Clone the repository and install dependencies:

 ```bash
 git clone https://github.com/Seyo2412/robot-kinematics-simulator.git
 cd robot-kinematics-simulator
 python3 -m venv venv
 source venv/bin/activate
 pip install -r requirements.txt
 ```

 ## Usage

 Run the 2D planar arm simulation:

 ```bash
 python main.py
 ```

 Run the interactive 3D 6-DOF robot animation:

 ```bash
 python main_3d.py
 ```

 Render the high-fidelity 3D simulation GIF (PyVista):

 ```bash
 python render_pyvista.py
 ```

 Run the automated test suite:

 ```bash
 pytest
 ```

## Roadmap

 - [x] Planar 3R Forward Kinematics
 - [x] Inverse Kinematics (IK) via Analytical Methods
 - [x] Trajectory Planning (quintic polynomial interpolation)
 - [x] 3D Kinematics for 6-DOF Manipulator (UR5 Industrial Arm)
 - [x] CAD-Grade 3D Volumetric Rendering (PyVista & VTK)
 - [x] Automated Unit Test Suite (pytest)