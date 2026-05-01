# EE631 Final Project — Installation & Usage Guide

**Zhang, Shiyan · Spring 2026**

This folder contains all original code for this project.
For algorithmic details and what was built, see [CONTRIBUTIONS.md](CONTRIBUTIONS.md).

---

## Prerequisites

- Ubuntu 24.04, ROS2 Jazzy
- Gazebo Sim 8.x with `ros_gz` bridge
- Python: `numpy`, `scipy`, `pyyaml`, `matplotlib`

Install ROS2 packages:

```bash
sudo apt install \
  ros-jazzy-nav2-bringup \
  ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-image \
  ros-jazzy-turtlebot3-gazebo ros-jazzy-turtlebot3-navigation2 \
  ros-jazzy-slam-toolbox
```

---

## Step 1 — Clone the Base Repository

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/SIT-Robotics-and-Automation-Laboratory/CPE631-Navigation-ROS2 cpe631_ros2_pkg
cd cpe631_ros2_pkg
```

If you prefer a shorter directory name, you can replace `cpe631_ros2_pkg` with `cpe631_ros2`, but keep that name consistent in the later commands.

---

## Step 2 — Copy This Project's Files into the Base Repo

Run these commands from the **root of this project checkout**, then copy into the cloned base repo. In other words, keep both repositories available side by side: this repository provides the files under `codefiles/`, and the cloned base repository receives them.

If your checkout is named differently, replace `../CPE631-Navigation-ROS2-main` with the actual path to this project.

```bash
# Python nodes
cp ../CPE631-Navigation-ROS2-main/codefiles/social_nav_node.py     cpe631_ros2/social_nav_node.py
cp ../CPE631-Navigation-ROS2-main/codefiles/ped_pose_extractor.py  cpe631_ros2/ped_pose_extractor.py
cp ../CPE631-Navigation-ROS2-main/codefiles/peds.py                cpe631_ros2/peds.py
cp ../CPE631-Navigation-ROS2-main/codefiles/data_collector.py      cpe631_ros2/data_collector.py
cp ../CPE631-Navigation-ROS2-main/codefiles/goal_sender.py         cpe631_ros2/goal_sender.py

# C++ D* Lite plugin
cp ../CPE631-Navigation-ROS2-main/codefiles/plugin/dstar_planner.cpp  src/dstar_planner.cpp
cp ../CPE631-Navigation-ROS2-main/codefiles/plugin/cpe631_ros2_dstar_planner.xml  cpe631_ros2_dstar_planner.xml
mkdir -p include/cpe631_ros2
cp ../CPE631-Navigation-ROS2-main/codefiles/plugin/include/cpe631_ros2/dstar_planner.hpp  include/cpe631_ros2/dstar_planner.hpp

# Nav2 parameter files
cp ../CPE631-Navigation-ROS2-main/codefiles/param/*.yaml  param/
cp ../CPE631-Navigation-ROS2-main/codefiles/param/*.xml   param/

# Launch files
cp ../CPE631-Navigation-ROS2-main/codefiles/launch/cafe.launch.py          launch/cafe.launch.py
cp ../CPE631-Navigation-ROS2-main/codefiles/launch/cafe_dynamic.launch.py  launch/cafe_dynamic.launch.py

# Map (if not already present)
mkdir -p maps
cp ../CPE631-Navigation-ROS2-main/codefiles/maps/cafe.yaml  maps/cafe.yaml
cp ../CPE631-Navigation-ROS2-main/codefiles/maps/cafe.pgm   maps/cafe.pgm

# Updated build system files
cp ../CPE631-Navigation-ROS2-main/codefiles/build_files/CMakeLists.txt  CMakeLists.txt
cp ../CPE631-Navigation-ROS2-main/codefiles/build_files/package.xml     package.xml
```

---

## Step 3 — Build

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select cpe631_ros2 --symlink-install
source install/setup.bash
```

> Run `source install/setup.bash` in every new terminal.

---

## Running the Code

### Option A — Automated batch experiments (recommended)

Runs all 7 planner modes, 25 trials each, writes results to CSV:

```bash
cd ~/ros2_ws/src/cpe631_ros2_pkg
RUNS_PER_MODE=25 \
CSV_FILE=~/ros2_ws/experiment_results.csv \
./codefiles/run_all_experiments.sh
```

Run specific modes only:

```bash
MODES=social,social_astar RUNS_PER_MODE=5 ./codefiles/run_all_experiments.sh
```

Headless (no GUI, faster):

```bash
RUNS_PER_MODE=25 ENABLE_RVIZ=0 GZ_GUI=0 \
CSV_FILE=~/ros2_ws/experiment_results.csv \
./codefiles/run_all_experiments.sh
```

### Option B — Manual single run (demo / step-by-step)

```bash
# Terminal 1 — Gazebo + Nav2 + pedestrians
ros2 launch cpe631_ros2 cafe_dynamic.launch.py \
  navigation:=true map_file:=maps/cafe.yaml enable_peds:=true

# Terminal 2 — social costmap node
ros2 run cpe631_ros2 social_nav_node

# Terminal 3 — metrics recorder
ros2 run cpe631_ros2 data_collector \
  --ros-args -p mode:=social -p experiment_id:=1 -p csv_file:=results.csv

# Terminal 4 — goal sender (drives the robot)
ros2 run cpe631_ros2 goal_sender
```

Custom route (format: `x,y,yaw_rad` separated by `;`):

```bash
ros2 run cpe631_ros2 goal_sender --ros-args \
  -p route:="0.5,-6.5,2.08;-3.5,0.0,0.89;2.5,6.0,-2.75;-2.0,2.1,-1.10;2.0,-4.0,-2.16"
```

---

## Experiment Modes

| Mode | Global Planner | Local Controller | Social Costmap | Param file |
| --- | --- | --- | :---: | --- |
| `baseline` | NavFn (Dijkstra) | DWB | No | `nav2_burger.yaml` *(base repo)* |
| `dynamic` | NavFn (Dijkstra) | DWB | No | `nav2_burger.yaml` |
| `dynamic_astar` | NavFn (A\* flag) | DWB | No | `param/nav2_dynamic_astar.yaml` |
| `dynamic_dstar` | D\* Lite plugin | DWB | No | `param/nav2_dynamic_dstar.yaml` |
| `social` | NavFn (Dijkstra) | MPPI | Yes | `param/nav2_social_dynamic.yaml` |
| `social_astar` | NavFn (A\* flag) | MPPI | Yes | `param/nav2_social_astar.yaml` |
| `social_smac` | Smac Hybrid A\* | MPPI | Yes | `param/nav2_social_smac.yaml` |
| `social_dstar` | D\* Lite plugin | MPPI | Yes | `param/nav2_social_dstar.yaml` |

---

## Results Files

| File | Contents |
|---|---|
| `5.csv` | Main Nav2 simulation results — baseline, dynamic, dynamic_astar, dynamic_dstar, social, social_astar, social_dstar — 25 runs each |
| `social_smac_tuned_25.csv` | social_smac results — 25 runs, tuned parameters, EXPERIMENT_TIMEOUT=500 |

> All `time_s` values are **ROS/Gazebo simulation seconds**, not wall-clock time.
