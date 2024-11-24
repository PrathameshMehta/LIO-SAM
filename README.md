# LIO-SAM

## Overview

LIO-SAM provides a set of ROS 2 packages for:

- Controlling a robot in the Gazebo simulation environment.
- Performing SLAM (Simultaneous Localization and Mapping) using LIO-SAM.
- Visualizing and analyzing point cloud data.

These tools facilitate real-time mapping and localization, crucial for robot navigation in complex environments.

---

## Table of Contents

1. [Prerequisites](#prerequisites)
2. [Workspace Setup](#workspace-setup)
3. [How to Use](#how-to-use)
    - [Run the Robot Controller](#run-the-robot-controller)
    - [Launch SLAM](#launch-slam)
    - [Start the Gazebo Simulation](#start-the-gazebo-simulation)
    - [Control the Robot](#control-the-robot)
    - [Analyze Point Cloud Data](#analyze-point-cloud-data)
4. [Features](#features)
5. [Contributions](#contributions)
6. [License](#license)

---

## Prerequisites

Ensure the following software is installed on your system:

1. **ROS 2 Foxy:**
   ```bash
   sudo apt update
   sudo apt install ros-foxy-desktop

    Gazebo and RViz:

sudo apt install ros-foxy-gazebo-ros-pkgs ros-foxy-rviz2

Python 3 Colcon Extensions:

sudo apt install python3-colcon-common-extensions

GTSAM Libraries:

sudo add-apt-repository ppa:borglab/gtsam-release-4.0
sudo apt update
sudo apt install libgtsam-dev libgtsam-unstable-dev

CloudCompare:

    sudo snap install cloudcompare

Workspace Setup

    Clone this repository:

git clone git@github.com:PrathameshMehta/LIO-SAM.git ~/ros2_ws/src

Navigate to your ROS 2 workspace:

cd ~/ros2_ws

Build the workspace:

colcon build

Source the workspace environment:

    source install/setup.bash

How to Use
1. Run the Robot Controller

Start the robot control node:

ros2 launch robot_control robot_control.launch.py

2. Launch SLAM

In a new terminal, source the workspace and start the SLAM node:

source install/setup.bash
ros2 launch lio_sam run.launch.py

3. Start the Gazebo Simulation

In another terminal, source the workspace and launch the Gazebo simulation:

source install/setup.bash
ros2 launch robot_gazebo robot_sim.launch.py

4. Control the Robot

Run the teleoperation node to move the robot using your keyboard:

source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard

Use the following keys to navigate the robot:
Key	Action
i	Move forward
k	Stop
j	Turn left
l	Turn right
q	Increase speed
z	Decrease speed
5. Analyze Point Cloud Data

    After running the simulation, close all nodes and open CloudCompare:

    cloudcompare

    Navigate to File -> Open and locate the LOAM folder in your Downloads directory.

    Load the point cloud data files to visualize the generated map.

Features

    Real-time SLAM visualization in RViz.
    Keyboard-based robot navigation in Gazebo.
    Exported point cloud files for detailed analysis.

Contributions

Contributions are welcome! To contribute:

    Fork the repository:

git clone git@github.com:PrathameshMehta/LIO-SAM.git

Create a new branch for your feature:

git checkout -b feature-name

Commit your changes:

git add .
git commit -m "Description of changes"

Push to your branch:

git push origin feature-name

Open a pull request on GitHub.
