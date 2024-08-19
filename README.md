# STERO2
==================

This repository contains the launch files to simulate the TIAGo robot in ROS 2.

## Setup

### Prerequisites

1. Install ROS 2 Humble by following the [installation instructions](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).

2. Update the apt package index and install needed packages

```console
sudo apt-get update

sudo apt-get install git python3-vcstool python3-rosdep python3-colcon-common-extensions
```

### Setting up the workspace

Create a workspace and clone all repositories:

```console
mkdir -p ~/tiago_public_ws/src
cd ~/tiago_public_ws
vcs import --input https://raw.githubusercontent.com/RCPRG-ros-pkg/STERO2/main/tiago_public_stero.repos src
```

Install dependencies using rosdep

```console
sudo rosdep init
rosdep update
rosdep install --from-paths src -y --ignore-src
```

Source the environment and build

```console
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

Finally, before running any application you have to source the workspace

```console
source ~/tiago_public_ws/install/setup.bash
```

Also you can add it to your .bashrc


## Simulation

### Nav2 + rviz2 + MoveIt!

Launch gazebo simulation:

```console
ros2 launch tiago_gazebo tiago_gazebo.launch.py navigation:=True moveit:=True is_public_sim:=True use_grasp_fix_plugin:=True
```
