# Project Guide

## Project Context

This repository contains the EE631 final project implementation for ROS 2 social navigation simulation in an indoor human-robot interaction environment.

Main project path:

```bash
/home/dx/CPE631-Navigation-ROS2-main
```

ROS 2 workspace path used during experiments:

```bash
/home/dx/ros2_ws/A/CPE631-Navigation-ROS2-main
```

Original backup path:

```bash
/home/dx/ros2_ws/or/CPE631-Navigation-ROS2-main
```

## Repository Layout

- `cpe631_ros2/`: main ROS 2 package used by the simulator.
- `codefiles/`: improved project code for submission and review.
- `param/`: Nav2 parameter files. `nav2_social_smac.yaml` is the added Smac Hybrid A* configuration with local/global social costmaps.
- `launch/`: launch files used by the experiment scripts.
- `A/`: experiment data, CSV results, and run logs.
- `FRC.CSV`: final consolidated experiment results.

As a rule, keep reusable improvements in `codefiles/` and avoid editing the original backup directory directly.

## Codefiles Overview

| File | Purpose | Requires ROS 2/Gazebo |
| --- | --- | --- |
| `ped_pose_extractor.py` | Extracts pedestrian poses from Gazebo model poses and publishes `PoseArray` data. | Yes |
| `social_nav_node.py` | Generates the anisotropic Gaussian predictive social costmap `/social_costmap`. | Yes |
| `data_collector.py` | Records path length, time, encounters, social-distance violations, and smoothness metrics. | Yes |
| `peds.py` | Creates and manages five pedestrians, including one random-walk pedestrian. | Yes |
| `goal_sender.py` | Sends a sequence of Nav2 `NavigateToPose` goals. | Yes |
| `integration_guide.yaml` | Documents how to connect the social costmap to Nav2 costmaps. | No |
| `run_all_experiments.sh` | Runs baseline, dynamic, social, social_astar, social_smac, and D* variants. | Yes |
| `run_sequential_reboot.sh` | Runs long experiments across reboot-separated modes. | Yes |
| `setup_and_start.sh` | Configures automatic reboot continuation for long experiments. | Yes, requires sudo |
| `convert_csv_time.py` | Converts simulation time to wall-clock estimates using measured RTF. | No |
| `point.py` | Checks route points and yaw values on the cafe map. | No |

## Using the Improved ROS 2 Nodes

To run the improved nodes from `codefiles/`, copy them into the main package and rebuild:

```bash
cd /home/dx/CPE631-Navigation-ROS2-main
cp codefiles/ped_pose_extractor.py cpe631_ros2/ped_pose_extractor.py
cp codefiles/social_nav_node.py    cpe631_ros2/social_nav_node.py
cp codefiles/data_collector.py     cpe631_ros2/data_collector.py
cp codefiles/peds.py               cpe631_ros2/peds.py
cp codefiles/goal_sender.py        cpe631_ros2/goal_sender.py

cd /home/dx/ros2_ws/A
colcon build --packages-select cpe631_ros2 --symlink-install
source install/setup.bash
```

The main experiment script also syncs the required launch, parameter, world, model, and core Python files into the active ROS 2 workspace.

## Core Components

### `ped_pose_extractor.py`

Extracts pedestrian poses from Gazebo `/world/cafe/dynamic_pose/info` or bridged model-pose topics. It publishes pedestrian pose arrays for `social_nav_node.py` and `data_collector.py`.

### `social_nav_node.py`

Implements an anisotropic Gaussian predictive social costmap:

```text
/pedestrian_poses
-> pedestrian velocity estimation
-> future pedestrian position prediction
-> anisotropic Gaussian social cost
-> /social_costmap
-> Nav2 local/global costmaps
```

The goal is to avoid not only current pedestrian positions, but also the likely near-future motion direction of pedestrians.

### `data_collector.py`

Records experiment metrics and writes CSV rows.

| Field | Meaning |
| --- | --- |
| `path_length_m` | Robot path length. |
| `time_s` | Simulation time from route start to result. |
| `encounters` | Number of entries into the encounter threshold. |
| `personal_violations` | Number of personal-space violations. |
| `intimate_violations` | Number of intimate-space violations. |
| `min_ped_dist_m` | Minimum distance to any pedestrian. |
| `mean_angular_acceleration` | Mean angular acceleration; lower values indicate smoother motion. |

Time convention:

- ROS 2/Nav2 experiments use `use_sim_time:=true`.
- CSV `time_s` is ROS/Gazebo simulation time, not wall-clock time.
- `goal_sender.py` simulation timeout uses ROS/Gazebo simulation time.
- Wall-clock runtime should be discussed separately using the measured real-time factor (RTF).

## Running Nav2 Experiment Modes

Main script:

```bash
cd /home/dx/CPE631-Navigation-ROS2-main
chmod +x codefiles/run_all_experiments.sh
./codefiles/run_all_experiments.sh
```

Supported modes:

- `baseline`: no pedestrians, no social costmap.
- `dynamic`: pedestrians enabled, standard dynamic obstacle avoidance, no social costmap.
- `social`: pedestrians enabled, NavFn/Dijkstra plus social costmap.
- `social_astar`: pedestrians enabled, A* plus social costmap.
- `social_smac`: pedestrians enabled, Smac Hybrid A* plus local/global social costmaps.
- `dynamic_dstar`: pedestrians enabled, D* planner, no social costmap.
- `social_dstar`: pedestrians enabled, D* planner plus social costmap.

Run selected modes:

```bash
MODES=dynamic,social RUNS_PER_MODE=5 ./codefiles/run_all_experiments.sh
```

Run 25 trials:

```bash
RUNS_PER_MODE=25 \
CSV_FILE=/home/dx/ros2_ws/A/experiment_results_25run.csv \
./codefiles/run_all_experiments.sh
```

## Final Results

The final consolidated result file is:

```text
FRC.CSV
```

It keeps all attempts, including retry attempts. It also includes synchronized wall-clock estimate columns:

- `actual_rtf`
- `time_wall_actual_s`
- `time_wall_actual_min`

Current row counts:

| Mode | Rows | Successful rows |
| --- | ---: | ---: |
| `social` | 29 | 20 |
| `social_astar` | 29 | 22 |
| `social_dstar` | 28 | 23 |
| `social_smac` | 39 | 19 |

Use `time_s` for algorithm comparison because it is simulation time. Use the wall-clock columns only when discussing how long the experiments took to run on the machine.

## Long Experiments and Reboot Workflow

`run_sequential_reboot.sh` runs modes sequentially with machine reboots between modes to reduce long-running Gazebo/Nav2 state contamination.

```bash
./codefiles/run_sequential_reboot.sh --status
./codefiles/run_sequential_reboot.sh --reset
```

`setup_and_start.sh` configures `@reboot` continuation and starts the first experiment. It modifies sudoers and crontab, so it requires sudo:

```bash
sudo bash codefiles/setup_and_start.sh
```

Monitor logs:

```bash
tail -f /home/dx/ros2_ws/A/run_logs/master.log
```

## Quick Review Checklist

```bash
ls codefiles
sed -n '1,20p' FRC.CSV
tail -n 30 FRC.CSV
ls A
ls A/run_logs
```
