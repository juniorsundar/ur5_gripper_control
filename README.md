# UR5 Control with Gripper (_Pick-and-Place with Octomap_)

____

## Introduction


## Prerequisite

- Ubuntu 20.04
- ROS Noetic Ninjemys
- MoveIt! Motion Planning Framework
- Gazebo 11

## Installation

Install prerequisite packages

```bash
sudo apt install ros-noetic-ros-numpy ros-noetic-ros-controllers ros-noetic-ros-control ros-noetic-cv-bridge 
```

Initialise the catkin workspace:

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
```

Clone prerequisite repositories:

```bash
# Clone this repository
git clone https://github.com/juniorsundar/ur5_gripper_control.git
# Cloning robot and gripper description files
git clone -b melodic-devel https://github.com/ros-industrial/universal_robot.git
git clone https://github.com/filesmuggler/robotiq.git
# Cloning Realsense Gazebo Plugin
git clone https://github.com/issaiass/realsense_gazebo_plugin
```

Build catkin workspace:

```bash
cd ~/catkin_ws
catkin_make
```

## Robot and Workspace

![workspace_and_robot](./img/workspace.jpg)

### Robot _(internal model)_

- UR5
- Robotiq 85 Gripper
- Raised Pedestal (20cm) as Base

refer to: ```urdf/ur5_robotiq85_gripper.urdf.xacro```

### Workspace _(environment)_

- Start and End Platform (1m x 1m x 20cm)
- Pillars at Corner of Platforms (20cm x 20cm x 75cm)
- Red Cube (5cm x 5cm x 5cm) (target of pick-and-place operation)

refer to: ```worlds/world.world```

## Perception and Octomap

Since planning is being managed by the MoveIt! Motion Planning Framework, we can rely on the Octomap plugin integrated into it. To facilitate it, we need a pointcloud.