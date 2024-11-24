# LIO-SAM

## Overview
This repository contains ROS 2 packages for:
- Controlling a robot in simulation (Gazebo).
- Implementing SLAM (Simultaneous Localization and Mapping) using **LIO-SAM**.
- Visualizing point cloud data for analysis.

These tools enable real-time mapping and localization, which are crucial for robotic navigation in dynamic environments.

---

## Prerequisites
Ensure the following software is installed on your system:

1. **ROS 2 Foxy**:
   ```bash
   sudo apt install ros-foxy-desktop

    Gazebo and RViz:

sudo apt install ros-foxy-gazebo-ros-pkgs ros-foxy-rviz2

Python 3 Colcon Extensions:

sudo apt install python3-colcon-common-extensions

GTSAM Libraries:

sudo add-apt-repository ppa:borglab/gtsam-release-4.0
sudo apt update
sudo apt install libgtsam-dev libgtsam-unstable-dev

CloudCompare (for point cloud analysis):

sudo snap install cloudcompare

Workspace Setup

Follow these steps to set up your workspace:

    Clone the repository:

git clone git@github.com:PrathameshMehta/LIO-SAM.git

Navigate to your workspace:

cd ~/ros2_ws

Build the workspace:

colcon build

Source the workspace:

source install/setup.bash


Here is the complete README.md file written in markdown format for your repository:

# LIO-SAM

## Overview
This repository contains ROS 2 packages for:
- Controlling a robot in simulation (Gazebo).
- Implementing SLAM (Simultaneous Localization and Mapping) using **LIO-SAM**.
- Visualizing point cloud data for analysis.

These tools enable real-time mapping and localization, which are crucial for robotic navigation in dynamic environments.

---

## Prerequisites
Ensure the following software is installed on your system:

1. **ROS 2 Foxy**:
   ```bash
   sudo apt install ros-foxy-desktop

    Gazebo and RViz:

sudo apt install ros-foxy-gazebo-ros-pkgs ros-foxy-rviz2

Python 3 Colcon Extensions:

sudo apt install python3-colcon-common-extensions

GTSAM Libraries:

sudo add-apt-repository ppa:borglab/gtsam-release-4.0
sudo apt update
sudo apt install libgtsam-dev libgtsam-unstable-dev

CloudCompare (for point cloud analysis):

    sudo snap install cloudcompare

Workspace Setup

Follow these steps to set up your workspace:

    Clone the repository:

git clone git@github.com:PrathameshMehta/LIO-SAM.git

Navigate to your workspace:

cd ~/ros2_ws

Build the workspace:

colcon build

Source the workspace:

    source install/setup.bash

Usage Instructions
1. Robot Controller

Launch the robot control node:

ros2 launch robot_control robot_control.launch.py

2. SLAM Node

In a new terminal, launch the SLAM node:

source install/setup.bash
ros2 launch lio_sam run.launch.py

3. Gazebo Simulation

In another terminal, start the Gazebo simulation:

source install/setup.bash
ros2 launch robot_gazebo robot_sim.launch.py

4. Teleoperation

In a fourth terminal, run the teleoperation node to control the robot using your keyboard:

source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard

Use the following keys to navigate the robot:

    i: Move forward
    k: Stop
    j: Turn left
    l: Turn right
    q: Increase speed

    Analyzing Results

    After running the simulation, close all nodes and open CloudCompare:

cloudcompare

Navigate to File -> Open and locate the LOAM folder in your Downloads directory.

Load the point cloud data files to visualize the generated map.

Features

    Real-time SLAM visualization in RViz.
    Keyboard-based robot navigation in Gazebo.
    Exported point cloud data for advanced analysis in CloudCompare.


License

This project is licensed under the MIT License.
