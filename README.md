# Robot Kinematics Simulator (2D & 3D)

 A modular kinematics and trajectory simulation tool for multi-joint robotic arms built with Python, NumPy, and Matplotlib.

 ![3D Robot Simulation Demo](assets/demo_3d.gif)

## Features
 - Forward Kinematics (FK) for both planar 3R and spatial 6-DOF manipulators (UR5 parameters via DH-matrices).
 - Analytical Inverse Kinematics (IK) for planar 3-DOF manipulators with workspace reachability checks.
 - Smooth trajectory planning via quintic polynomials (C2-continuous, jerk-limited motion profiles).
 - Interactive 2D and 3D animated visualization with real-time end-effector path tracing.
 - Automated unit testing suite with pytest verifying mathematical correctness.

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

 Run the automated test suite:

 ```bash
 pytest
 ```

## Roadmap

 - [x] Planar 3R Forward Kinematics
 - [x] Inverse Kinematics (IK) via Analytical Methods
 - [x] Trajectory Planning (quintic polynomial interpolation)
 - [x] 3D Kinematics for 6-DOF Manipulator (UR5 Industrial Arm)
 - [x] Automated Unit Test Suite (pytest)