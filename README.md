# 3R Robot Kinematics Simulator

 A modular kinematics and trajectory simulation tool for multi-joint robotic arms built with Python, Numpy and Matplotlib. 

## Features
 - Forward Kinematics (FK) based on standard Denavit-Hartenberg (DH) parameterization.
 - Interactive Matplotlib visualization of planar 3-DOF manipulators.
 - Clean separation between mathematical models and plotting routines.

## Installation

 Clone the repository and install the dependencies:

 ```bash
 git clone https://github.com/Seyo2412/robot-kinematics-simulator.git
 cd robot-kinematics-simulator
 python3 -m venv venv
 source venv/bin/activate
 pip install -r requirements.txt
 ```

## Usage

 Run the baseline forward kinematics demo:

 ```bash
 python main.py
 ```

## Roadmap

 - [x] Planar 3R Forward Kinematics (DH Matrix)
 - [ ] Inverse Kinematics (IK) via Analytical & Numeral Methods
 - [ ] Quintic Polynomial Trajectory Planning
 - [ ] 3D Kinematics for 6-DOF Manipulators (UR5 / PUMA)