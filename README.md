# Overview

This project has two main purposes:
1. Test how powerful cursor is as an AI-aided IDE (i.e. can it set up an entire simulation for me so that I can just do the fun parts)
2. Use my background in controls to implement sensor fusion and control for a drone that uses a remote observer and IR sensor to find "fires" and "put them out"

# Firefighting Drone Simulation

This ROS2 package provides a Gazebo simulation environment for a firefighting drone.

## Prerequisites

- Ubuntu 22.04
- ROS2 Humble
- Gazebo
- WSL2 (if running on Windows)

## Installation

1. Make sure you have ROS2 Humble installed:
```bash
sudo apt update
sudo apt install ros-humble-desktop-full
```

2. Install Gazebo and ROS2 Gazebo packages:
```bash
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros
```

3. Create a ROS2 workspace and clone this repository:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
# Clone this repository
cd ~/ros2_ws
colcon build
```

## Usage

1. Source your ROS2 workspace:
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

2. Launch the simulation:
```bash
ros2 launch firefighting_drone gazebo_sim.launch.py
```

## Project Structure

- `launch/`: Contains launch files for Gazebo and other nodes
- `models/`: Contains the drone model and map models
- `worlds/`: Contains Gazebo world files
- `config/`: Contains configuration files for the simulation

## Adding Your Drone Model

1. Place your drone model files in the `models/drone/` directory
2. Place your map DAE file in the `models/map/` directory
3. Update the launch file if necessary to match your model's file names

## Notes

- The simulation uses Gazebo's physics engine for realistic drone movement
- You can implement your own controller separately and interface with the simulation through ROS2 topics
- Make sure your drone model's SDF file includes proper physics properties for accurate simulation 
