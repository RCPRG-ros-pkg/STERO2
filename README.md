# STERO2

This repository contains the launch files to simulate the TIAGo robot in ROS 2.

## Setup

### Prerequisites

1. Install ROS 2 Iron by following the [installation instructions](https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debs.html).

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
vcs import --input https://raw.githubusercontent.com/RCPRG-ros-pkg/STERO2/refs/heads/iron/tiago_public_stero.repos src
```

Install dependencies using rosdep

```console
sudo rosdep init
rosdep update
rosdep install --from-paths src -y --ignore-src --rosdistro iron
```

Source the environment and build

```console
source /opt/ros/iron/setup.bash
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

## Dependencies

The rosdep tool should install all required dependencies. However, if something is missing, you can install it manually:
```console
sudo apt install ros-iron-desktop ros-iron-gazebo-dev ros-iron-gazebo-ros ros-iron-xacro ros-iron-backward-ros ros-iron-control-msgs ros-iron-controller-interface ros-iron-controller-manager ros-iron-controller-manager-msgs ros-iron-diagnostic-updater ros-iron-hardware-interface ros-iron-realtime-tools ros-iron-ros2-control-test-assets ros-iron-diff-drive-controller ros-iron-force-torque-sensor-broadcaster ros-iron-generate-parameter-library ros-iron-generate-parameter-library-py ros-iron-parameter-traits ros-iron-rsl ros-iron-tcb-span ros-iron-tl-expected ros-iron-joint-state-broadcaster ros-iron-imu-sensor-broadcaster ros-iron-control-toolbox ros-iron-filters ros-iron-joint-trajectory-controller ros-iron-ros2controlcli ros-iron-joint-state-publisher ros-iron-joint-state-publisher-gui ros-iron-launch-param-builder ros-iron-joy-linux ros-iron-joy-teleop ros-iron-teleop-tools-msgs ros-iron-twist-mux ros-iron-twist-mux-msgs ros-iron-eigen-stl-containers ros-iron-geometric-shapes ros-iron-google-benchmark-vendor ros-iron-moveit-common ros-iron-moveit ros-iron-moveit-core ros-iron-moveit-kinematics ros-iron-moveit-msgs ros-iron-moveit-ros-move-group ros-iron-moveit-ros-occupancy-map-monitor ros-iron-moveit-ros-planning ros-iron-moveit-ros-planning-interface ros-iron-moveit-ros-warehouse ros-iron-object-recognition-msgs ros-iron-octomap ros-iron-octomap-msgs ros-iron-octomap-server ros-iron-random-numbers ros-iron-ruckig ros-iron-srdfdom ros-iron-urdfdom-py ros-iron-warehouse-ros libgeographic-dev libgeographic19 ros-iron-behaviortree-cpp-v3 ros-iron-bond ros-iron-bondcpp ros-iron-camera-calibration-parsers ros-iron-camera-info-manager ros-iron-costmap-queue ros-iron-dwb-core ros-iron-dwb-critics ros-iron-dwb-msgs ros-iron-dwb-plugins ros-iron-gazebo-plugins ros-iron-gazebo-ros-pkgs ros-iron-geographic-msgs ros-iron-nav-2d-msgs ros-iron-nav-2d-utils ros-iron-nav2-amcl ros-iron-nav2-behavior-tree ros-iron-nav2-behaviors ros-iron-nav2-bringup ros-iron-nav2-bt-navigator ros-iron-nav2-collision-monitor ros-iron-nav2-common ros-iron-nav2-constrained-smoother ros-iron-nav2-controller ros-iron-nav2-core ros-iron-nav2-costmap-2d ros-iron-nav2-dwb-controller ros-iron-nav2-lifecycle-manager ros-iron-nav2-map-server ros-iron-nav2-mppi-controller ros-iron-nav2-msgs ros-iron-nav2-navfn-planner ros-iron-nav2-planner ros-iron-nav2-regulated-pure-pursuit-controller ros-iron-nav2-rotation-shim-controller ros-iron-nav2-rviz-plugins ros-iron-nav2-simple-commander ros-iron-nav2-smac-planner ros-iron-nav2-smoother ros-iron-nav2-theta-star-planner ros-iron-nav2-util ros-iron-nav2-velocity-smoother ros-iron-nav2-voxel-grid ros-iron-nav2-waypoint-follower ros-iron-navigation2 ros-iron-ompl ros-iron-robot-localization ros-iron-slam-toolbox ros-iron-smclib ros-iron-turtlebot3-gazebo ros-iron-gazebo-ros2-control ros-iron-moveit-configs-utils ros-iron-moveit-ros-control-interface ros-iron-moveit-simple-controller-manager ros-iron-moveit-planners-ompl ros-iron-moveit-ros-robot-interaction ros-iron-moveit-ros-visualization ros-iron-moveit-ros-perception ros-iron-forward-command-controller ros-iron-position-controllers ros-iron-rviz-visual-tools ros-iron-moveit-visual-tools ros-iron-moveit-py ros-iron-pilz-industrial-motion-planner mc python3-colcon-mixin python3-colcon-clean git python3-vcstool python3-rosdep python3-colcon-common-extensions ros-iron-ros2-controllers ros-iron-ros2-control ros-iron-rqt-controller-manager
```


## Launch description

Simulated Tiago robot is a complex system. The main launch file includes a lot of other launch files.

### Visualization of launch description

To get track of all nodes, executable, variables ,substitutions, etc., you can use show_launch_description.py tool.
It is much more powerful than **ros2 launch -p**.

```console
show_launch_description.py ~/tiago_public_ws/src/tiago_simulation/tiago_gazebo/launch/tiago_gazebo.launch.py
```
