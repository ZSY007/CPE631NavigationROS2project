# CPE631 Final Project — Human-Aware Social Navigation in ROS 2

**Zhang, Shiyan · Spring 2026**

## Overview

This project implements **predictive, socially-aware robot navigation** in a
simulated indoor cafe environment with moving pedestrians. Unlike standard Nav2
planners that treat pedestrians as static obstacles, our system anticipates
where pedestrians are heading and plans around their **predicted future
positions** before a conflict arises.

Key contributions:

1. **Anisotropic Gaussian Predictive Social Costmap** (`social_nav_node.py`)
   — publishes a `/social_costmap` OccupancyGrid that Nav2 consumes as a
   costmap layer, steering all planners around predicted pedestrian
   trajectories.
2. **D\* Lite Nav2 Global Planner Plugin** (`plugin/dstar_planner.cpp`)
   — C++ implementation of the incremental D\* Lite algorithm, registered as
   a Nav2 `GlobalPlanner` plugin.
3. **Automated Experiment Framework** (`run_all_experiments.sh`,
   `data_collector.py`, `goal_sender.py`) — batch-runs multiple planner
   modes, records per-run metrics to CSV, supports retry-on-abort logic.
4. **Multi-mode Benchmark** — 9 experiment configurations comparing
   Dijkstra, A\*, D\* Lite, and Smac Hybrid A\* with/without the social
   costmap, each with 20 repeated trials.

For algorithmic details and contribution breakdown, see
[CONTRIBUTIONS.md](CONTRIBUTIONS.md).

---

## File Structure

```
codefiles/
├── README.md                     # This file
├── CONTRIBUTIONS.md              # Detailed technical writeup
│
│  ── Core Python Nodes ──────────────────────────────────────
├── social_nav_node.py            # ★ Predictive social costmap (main algorithm)
├── data_collector.py             # ★ Navigation metrics recorder (CSV output)
├── goal_sender.py                # Nav2 action-client waypoint executor
├── peds.py                       # Gazebo pedestrian simulator (5 agents)
├── ped_pose_extractor.py         # Gazebo Pose_V → /pedestrian_poses bridge
├── replan_trigger.py             # Proximity-triggered global costmap clear
├── convert_csv_time.py           # Utility: sim-time → wall-clock converter
│
│  ── Automation ─────────────────────────────────────────────
├── run_all_experiments.sh        # Batch experiment runner (all modes)
│
│  ── D* Lite C++ Plugin ─────────────────────────────────────
├── plugin/
│   ├── dstar_planner.cpp         # D* Lite Nav2 global planner
│   └── dstar_planner.hpp         # Header
│
│  ── Build System ───────────────────────────────────────────
├── build_files/
│   ├── CMakeLists.txt            # ament_cmake build configuration
│   ├── package.xml               # ROS 2 package manifest
│   ├── setup.py                  # Python package setup
│   ├── setup.cfg                 # setuptools config
│   └── cpe631_ros2_dstar_planner.xml  # pluginlib registration
│
│  ── Launch Files ───────────────────────────────────────────
├── launch/
│   ├── cafe.launch.py            # Basic mapping / navigation launch
│   └── cafe_dynamic.launch.py    # Dynamic social navigation launch
│
│  ── Nav2 Parameter Files ───────────────────────────────────
├── param/
│   ├── nav2_burger.yaml          # Baseline (Dijkstra + DWB)
│   ├── nav2_dynamic_astar.yaml   # A* + DWB (no social)
│   ├── nav2_dynamic_dstar.yaml   # D* Lite + DWB (no social)
│   ├── nav2_dynamic_conservative.yaml
│   ├── nav2_social_dynamic.yaml  # Dijkstra + MPPI + social costmap
│   ├── nav2_social_astar.yaml    # A* + MPPI + social costmap
│   ├── nav2_social_dstar.yaml    # D* Lite + MPPI + social costmap
│   ├── nav2_social_smac.yaml     # Smac Hybrid A* + MPPI + social costmap
│   ├── nav2_dstar_plus.yaml      # D*+ enhanced config
│   ├── custom_bt.xml             # Custom Nav2 behavior tree
│   ├── custom_bt_dstar_plus.xml
│   ├── social_pose_bridge.yaml   # ROS-GZ bridge config
│   └── turtlebot3_burger_bridge_local.yaml
│
│  ── Sample Results ─────────────────────────────────────────
└── results/
    └── sample_results.csv        # Experiment output (244 rows, 8 modes)
```

---

## Environment Requirements

| Component | Version |
|-----------|---------|
| Ubuntu | 24.04 |
| ROS 2 | Jazzy |
| Gazebo Sim | 8.x with `ros_gz` |
| Python | 3.12+ with `numpy` |

Install required ROS 2 packages:

```bash
sudo apt install \
  ros-jazzy-nav2-bringup \
  ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-image \
  ros-jazzy-turtlebot3-gazebo ros-jazzy-turtlebot3-navigation2 \
  ros-jazzy-slam-toolbox
```

---

## Build Instructions

### Step 1 — Clone the Base Repository

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/SIT-Robotics-and-Automation-Laboratory/CPE631-Navigation-ROS2 cpe631_ros2
cd cpe631_ros2
```

### Step 2 — Copy This Project's Files into the Base Repo

```bash
# Python nodes → cpe631_ros2/ (the ROS 2 Python package directory)
cp codefiles/social_nav_node.py    cpe631_ros2/social_nav_node.py
cp codefiles/ped_pose_extractor.py cpe631_ros2/ped_pose_extractor.py
cp codefiles/peds.py               cpe631_ros2/peds.py
cp codefiles/data_collector.py     cpe631_ros2/data_collector.py
cp codefiles/goal_sender.py        cpe631_ros2/goal_sender.py
cp codefiles/replan_trigger.py     cpe631_ros2/replan_trigger.py

# C++ D* Lite plugin
cp codefiles/plugin/dstar_planner.cpp   src/dstar_planner.cpp
mkdir -p include/cpe631_ros2
cp codefiles/plugin/dstar_planner.hpp   include/cpe631_ros2/dstar_planner.hpp

# Build system files
cp codefiles/build_files/CMakeLists.txt              CMakeLists.txt
cp codefiles/build_files/package.xml                 package.xml
cp codefiles/build_files/setup.py                    setup.py
cp codefiles/build_files/setup.cfg                   setup.cfg
cp codefiles/build_files/cpe631_ros2_dstar_planner.xml  cpe631_ros2_dstar_planner.xml

# Launch files and Nav2 parameter files
cp codefiles/launch/*.py   launch/
cp codefiles/param/*.yaml  param/
cp codefiles/param/*.xml   param/
```

### Step 3 — Build

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select cpe631_ros2 --symlink-install
source install/setup.bash
```

> **Note:** Run `source install/setup.bash` in every new terminal.

---

## How to Run

### Run a Single Demo (Manual)

Open 4 terminals, each sourced with `install/setup.bash`:

```bash
# Terminal 1 — Launch Gazebo + Nav2 + pedestrians
ros2 launch cpe631_ros2 cafe_dynamic.launch.py \
  navigation:=true social_navigation:=true enable_peds:=true

# Terminal 2 — Social costmap node (the core contribution)
ros2 run cpe631_ros2 social_nav_node

# Terminal 3 — Data collector (records metrics to CSV)
ros2 run cpe631_ros2 data_collector \
  --ros-args -p mode:=social -p csv_file:=results.csv

# Terminal 4 — Goal sender (drives the robot through waypoints)
ros2 run cpe631_ros2 goal_sender
```

To visualize the social costmap in RViz: add a **Map** display subscribing
to `/social_costmap`. The anisotropic Gaussian cost field around each
pedestrian should be visible as colored regions.

### Run All Experiments (Automated Batch)

```bash
cd ~/ros2_ws/src/cpe631_ros2

# Run all 9 modes × 5 trials each (headless, no GUI):
RUNS_PER_MODE=5 ENABLE_RVIZ=0 GZ_GUI=0 \
  CSV_FILE=~/ros2_ws/experiment_results.csv \
  ./codefiles/run_all_experiments.sh
```

Run specific modes only:

```bash
MODES=social,baseline RUNS_PER_MODE=10 \
  ./codefiles/run_all_experiments.sh
```

Key environment variables for `run_all_experiments.sh`:

| Variable | Default | Description |
|----------|---------|-------------|
| `MODES` | all | Comma-separated mode names to run |
| `RUNS_PER_MODE` | 5 | Trials per mode |
| `CSV_FILE` | auto | Output CSV path |
| `ENABLE_RVIZ` | 1 | Show RViz (set 0 for headless) |
| `GZ_GUI` | 1 | Show Gazebo GUI (set 0 for headless) |
| `EXPERIMENT_TIMEOUT` | 450 | Max sim-seconds per trial |
| `SKIP_BUILD` | 0 | Set 1 to skip `colcon build` |

---

## Experiment Modes

| Mode | Global Planner | Local Controller | Social Costmap | Pedestrians |
|------|---------------|-----------------|:-:|:-:|
| `baseline` | NavFn (Dijkstra) | DWB | ✗ | ✗ |
| `dynamic` | NavFn (Dijkstra) | DWB | ✗ | ✓ |
| `dynamic_astar` | NavFn (A\*) | DWB | ✗ | ✓ |
| `dynamic_dstar` | D\* Lite | DWB | ✗ | ✓ |
| `social` | NavFn (Dijkstra) | MPPI | ✓ | ✓ |
| `social_astar` | NavFn (A\*) | MPPI | ✓ | ✓ |
| `social_smac` | Smac Hybrid A\* | MPPI | ✓ | ✓ |
| `social_dstar` | D\* Lite | MPPI | ✓ | ✓ |
| `social_dstar_plus` | D\* Lite + replan trigger | MPPI | ✓ | ✓ |

---

## Performance Metrics (CSV Columns)

`data_collector.py` writes one row per completed navigation run:

| Column | Definition |
|--------|-----------|
| `path_length_m` | Cumulative odometry distance traveled (m) |
| `time_s` | ROS/Gazebo simulation time elapsed (s) |
| `encounters` | Times robot came within 1.0 m of any pedestrian |
| `personal_violations` | Times robot entered personal space (< 1.2 m, Proxemics) |
| `intimate_violations` | Times robot entered intimate space (< 0.45 m, Proxemics) |
| `min_ped_dist_m` | Global minimum distance to any pedestrian (m) |
| `mean_angular_acceleration` | Mean |Δω/Δt| (rad/s²) — lower = smoother path |

> All `time_s` values are **simulation seconds**, not wall-clock time.
> Use `convert_csv_time.py` to estimate wall-clock equivalents given a
> measured real-time factor.

---

## Results Output Location

- **Automated runs:** CSV file at the path specified by `CSV_FILE`
  (default: `~/ros2_ws/experiment_results_v3.csv`)
- **Sample results:** `results/sample_results.csv` contains 244 rows
  across all 8 experiment modes with 20 trials each.

---

## Nav2 Integration: How the Social Costmap Works

The social costmap is integrated into Nav2 as a **StaticLayer** plugin in
both the local and global costmap configurations:

```yaml
# In any social nav2_*.yaml param file:
local_costmap:
  local_costmap:
    ros__parameters:
      plugins: ["obstacle_layer", "social_layer", "inflation_layer"]
      social_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        enabled: true
        map_topic: /social_costmap        # Published by social_nav_node
        subscribe_to_updates: true
```

The pipeline:

```
Gazebo pedestrian models
    │
    ▼
/pedestrian_poses (PoseArray)  ←── peds.py publishes directly
    │
    ▼
social_nav_node.py
  ├── PedestrianTracker (velocity estimation, 3-mode classification)
  ├── Trajectory prediction (adaptive horizon, non-linear uncertainty)
  ├── Anisotropic Gaussian cost (asymmetric, group cost, approach boost)
  └── Interactive prediction (game-theoretic pedestrian deflection)
    │
    ▼
/social_costmap (OccupancyGrid)  ──► Nav2 costmap stack
    │
    ▼
Nav2 global + local planners avoid predicted pedestrian zones
```

---

## License

Course project for CPE631/EE631, Stevens Institute of Technology.
Based on: https://github.com/SIT-Robotics-and-Automation-Laboratory/CPE631-Navigation-ROS2
