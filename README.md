# EE631 Final Project — Human-Aware Navigation in Gazebo Sim / ROS 2 Jazzy

## Overview

This repository contains the code for the EE631 final project. The project addresses robot motion planning in a human environment, with navigation in Gazebo Sim, pedestrian-aware planning, and a benchmark comparison against standard planners.

The implementation is organized around the course assignment requirements:

- define a motion planning problem in a human environment;
- state the robot and environment assumptions;
- implement or adapt a planning algorithm in ROS 2;
- report simulation results and performance metrics;
- compare the planner with a standard baseline when possible.

For the detailed project write-up and implementation summary, see:

- [codefiles/README.md](codefiles/README.md)
- [codefiles/CONTRIBUTIONS.md](codefiles/CONTRIBUTIONS.md)

## Based on / 参考环境

本仓库基于 SIT Robotics 提供的室内人类交互环境（The Environment），并以其为基础进行修改和实验。参考仓库：

https://github.com/SIT-Robotics-and-Automation-Laboratory/CPE631-Navigation-ROS2

The Environment: An indoor human environment has been created, see the following
Github site on Human Robot Interaction Environment:
https://github.com/SIT-Robotics-and-Automation-Laboratory/CPE631-Navigation-ROS2

The Problem: You need to define your own motion planning problem as to navigate the
robot in the given environment. Clearly present your problem formulation in your final
report, state the assumptions on the robot (such as onboard sensors, etc.) and the
environment (such as map availability, etc.).

## What is in this repo

- `cpe631_ros2/` - ROS 2 Python nodes used in the simulator.
- `launch/` - launch files for mapping, navigation, and dynamic runs.
- `maps/`, `models/`, `worlds/` - the indoor cafe environment and assets.
- `param/` - Nav2 parameter files for default, dynamic, and social navigation modes.
- `src/`, `include/`, `cpe631_ros2_dstar_planner.xml` - the D* Lite Nav2 planner plugin.
- `codefiles/` - code copy and experiment notes used for the final submission package.

## Main project components

This project includes the following pieces:

- a social costmap node that reacts to pedestrians and their predicted motion;
- a Nav2 D* Lite global planner plugin for incremental replanning;
- dynamic navigation configurations using MPPI and conservative costmap settings;
- a standalone A* versus D* Lite benchmark for comparison;
- experiment scripts and CSV logging for result collection.

## Required performance metrics

The project is set up to report the metrics requested in the assignment:

- distance of the robot path;
- time for the robot to reach the goal;
- number of times the robot encounters people;
- whether the robot navigates in a natural and socially appropriate manner.

## Environment and dependencies

Recommended setup:

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Sim 8.x with `ros_gz`
- TurtleBot3 navigation packages

Useful packages:

```bash
sudo apt install \
  ros-jazzy-nav2-bringup \
  ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-image \
  ros-jazzy-turtlebot3-gazebo ros-jazzy-turtlebot3-navigation2 \
  ros-jazzy-slam-toolbox
```

## Build

```bash
cd <your_ros2_ws>
colcon build --packages-select cpe631_ros2 --symlink-install
source install/setup.bash
```

Open a new terminal and source `install/setup.bash` again before running ROS commands.

## How to run

### Mapping

```bash
ros2 launch cpe631_ros2 cafe.launch.py mapping:=true
```

Use teleoperation to drive the robot and build a map:

```bash
ros2 launch cpe631_ros2 teleop.launch.py model:=burger
```

### Save the map

```bash
mkdir -p maps
ros2 run nav2_map_server map_saver_cli -f maps/cafe
```

This creates `maps/cafe.yaml` and `maps/cafe.pgm`.

### Navigation

```bash
ros2 launch cpe631_ros2 cafe.launch.py navigation:=true map_file:=<path to map>/cafe.yaml enable_peds:=false
```

For dynamic runs with pedestrians, use:

```bash
ros2 launch cpe631_ros2 cafe_dynamic.launch.py navigation:=true map_file:=<path to map>/cafe.yaml enable_peds:=true
```

In RViz, use **2D Pose Estimate** to set the start pose, then **Nav2 Goal** to send a target.

### Standalone planner comparison

```bash
python3 codefiles/dstar_lite_planner.py --map maps/cafe.yaml --algorithm both
RUNS_PER_ALGO=25 CSV_FILE=a_dstar_results_25.csv ./codefiles/run_ad_experiments.sh
```

## Launch files and parameters

Important launch files:

- `launch/cafe.launch.py` - standard mapping and navigation flow;
- `launch/cafe_dynamic.launch.py` - more conservative dynamic-obstacle navigation.

Important parameter files:

- `param/nav2_burger.yaml` - baseline and dynamic modes (from base repo);
- `param/nav2_dynamic_conservative.yaml` - baseline with conservative costmap settings (no pedestrians);
- `param/nav2_dynamic_astar.yaml` and `param/nav2_dynamic_dstar.yaml` - comparison modes;
- `param/nav2_social_dynamic.yaml`, `param/nav2_social_astar.yaml`, `param/nav2_social_smac.yaml`, `param/nav2_social_dstar.yaml` - social navigation variants.

## Submission notes

The final project submission includes the report PDF, the zipped code package, and an optional video. This repository provides the code and run instructions that support that submission.
