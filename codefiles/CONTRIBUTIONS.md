# EE631 Final Project — What Was Built

Zhang, Shiyan · Spring 2026

For installation and how to run, see [README.md](README.md).

---

## Project Goal

Standard Nav2 planners (NavFn, A\*) treat pedestrians as static obstacles — they react only after a pedestrian blocks the path. This project adds predictive, socially-aware behavior: the robot anticipates where pedestrians are *going* and plans around their future positions before a conflict arises.

Contributions implemented and benchmarked:

1. An **Anisotropic Gaussian Predictive Social Costmap** (Python ROS2 node)
2. A **D\* Lite Nav2 global planner plugin** (C++)
3. A **Smac Hybrid A\* + MPPI** experiment configuration
4. A **standalone A\* vs D\* Lite map-level benchmark** (no Gazebo)
5. An **automated experiment framework** with per-run metrics collection

---

## Contribution 1 — Social Costmap Node (`social_nav_node.py`)

### Pipeline

Subscribes to `/pedestrian_poses` (PoseArray), runs a predictive model for each pedestrian, and publishes `/social_costmap` (nav_msgs/OccupancyGrid) at 5 Hz. Nav2 loads this as an extra layer in both the local and global costmap stacks, so all planners automatically route around predicted pedestrian trajectories.

```text
/pedestrian_poses  ──►  PedestrianTracker (per pedestrian)
                              │
                              ▼
                    velocity estimation
                    trajectory prediction
                    anisotropic Gaussian cost
                              │
                              ▼
                   /social_costmap  ──►  Nav2 local/global costmap
```

### Pedestrian Tracker

Each pedestrian gets a `PedestrianTracker` instance. The tracker estimates velocity using **exponential smoothing** with a configurable time constant τ:

```text
α_t  = 1 − exp(−Δt / τ)
v_new = α_t · v_instant + (1 − α_t) · v_prev
```

Two protection mechanisms prevent bad estimates:

- **Jump filter**: position jumps > 1.0 m between frames are discarded (Gazebo teleport artifacts)
- **Speed cap**: estimated velocity is clamped to 1.8 m/s

### 3-Mode Motion Classification

The shape of the cost ellipse adapts to how the pedestrian is moving:

| Mode | Condition | Gaussian shape |
| --- | --- | --- |
| Stationary / wandering | speed < 0.05 m/s, or net displacement / path length < 0.3 | Circular (symmetric) |
| Slow | 0.05 – 0.3 m/s | Weak ellipse (wider laterally) |
| Fast | > 0.3 m/s | Narrow forward ellipse (direction is reliable) |

Wandering is detected by comparing net displacement to total path length over the history window.

### Adaptive Prediction Horizon

Lookahead time scales with pedestrian speed so fast pedestrians are predicted further ahead:

```text
lookahead = clip(1.5 + 0.7 · speed,  min=1.5 s,  max=3.5 s)
```

### Non-Linear Uncertainty Growth

Variance at prediction step k grows as k^1.3 rather than linearly:

```text
σ_long(k) = √( σ_base² + q · k^1.3 )
```

### Asymmetric Gaussian

The hazard zone in front of a moving pedestrian is larger than behind.
The rear sigma is scaled by `behind_sigma_ratio = 0.5`, so the robot can pass safely behind a pedestrian without triggering avoidance.

### Group / Crowd Cost

When two pedestrians are within 1.5 m of each other, an extra Gaussian is placed at their midpoint. This discourages the robot from attempting to pass between them.

### Robot Direction-Aware Decay

Pedestrians behind the robot pose no immediate threat. Their cost amplitude is multiplied by `behind_ped_weight = 0.15`, preventing unnecessary detours for pedestrians the robot has already passed.

### Interactive Prediction (Lightweight Game-Theoretic Term)

The node generates 3 candidate robot trajectory hypotheses (varying yaw rate). Pedestrians are predicted to slightly repel away from the nearest robot trajectory, modelling the social expectation that a pedestrian will step aside when a robot is approaching on a collision course.

### Lethal Core

A hard obstacle circle of radius 0.25 m is stamped at each pedestrian's current position, ensuring Nav2 treats the pedestrian's body as a hard constraint regardless of predictive cost.

### ROS2 Interface

| Topic | Direction | Type |
| --- | --- | --- |
| `/pedestrian_poses` | In | `geometry_msgs/PoseArray` |
| `/amcl_pose` | In | `geometry_msgs/PoseWithCovarianceStamped` |
| `/odom` | In | `nav_msgs/Odometry` |
| `/social_costmap` | Out | `nav_msgs/OccupancyGrid` |

---

## Contribution 2 — D\* Lite Nav2 Plugin (`plugin/dstar_planner.cpp`)

### Algorithm

A C++ `nav2_core::GlobalPlanner` plugin implementing the D\* Lite incremental replanning algorithm. Registered as `"cpe631_ros2/DStarPlanner"` via pluginlib and loaded by `planner_server` when any `*_dstar*.yaml` param file is used.

Standard Nav2 global planners (NavFn, A\*) replan from scratch every time the costmap changes. D\* Lite maintains a consistent solution and only re-expands cells affected by the change — making it more efficient for environments with moving obstacles.

### Implementation

- Reads the Nav2 `Costmap2D` directly; no separate map file needed
- Priority queue keyed on `(k1, k2)` pairs as defined in the D\* Lite algorithm
- `makePlan()`: runs full D\* Lite from start to goal, extracts path by gradient descent on `g`-values
- Costmap update: marks changed cells inconsistent, calls `computeShortestPath()` to repair only the affected region
- Returns `nav_msgs/Path` compatible with Nav2's `FollowPath` action

The plugin is compiled as a shared library (`libcpe631_ros2_dstar_planner.so`) and registered via `plugin/cpe631_ros2_dstar_planner.xml`.

---

## Contribution 3 — Smac Hybrid A\* + MPPI Configuration (`social_smac`)

The `social_smac` mode pairs **Nav2 SmacPlannerHybrid** with the **MPPI local controller** and the social costmap. SmacPlannerHybrid generates SE(2)-aware paths that respect the robot's turning radius, producing more natural arcs in human spaces compared to grid-only planners. MPPI then optimises its local trajectories against the social costmap at each control step.

This represents the most sophisticated planner combination tested: kinodynamically feasible global paths + real-time trajectory optimisation + predictive social awareness.

Param file: `param/nav2_social_smac.yaml`

---

## Contribution 4 — Standalone A\* vs D\* Lite Benchmark (`dstar_lite_planner.py`)

A pure-Python, Gazebo-free comparison that runs both algorithms on `maps/cafe.yaml`.

- Pedestrian positions are projected onto the occupancy grid as circular static obstacles
- A\* plans on the final obstacle map; D\* Lite plans on the clean map then receives the obstacles as dynamic updates and replans
- Reports: path length (m), planning time (s), pedestrian proximity violations

This provides a controlled comparison of algorithmic behavior independent of Nav2 and simulation variability.

---

## Contribution 5 — Experiment Framework

### `data_collector.py`

ROS2 node that subscribes to `/amcl_pose`, `/pedestrian_poses`, `/odom`, and `/cmd_vel`, and writes one CSV row per completed navigation goal.

| Metric | Definition |
| --- | --- |
| `path_length_m` | Cumulative distance traveled (m) |
| `time_s` | ROS/Gazebo simulation time elapsed (s) |
| `encounters` | Times robot came within 1.0 m of a pedestrian |
| `personal_violations` | Times robot entered personal space (< 1.2 m) |
| `intimate_violations` | Times robot entered intimate space (< 0.45 m) |
| `min_ped_dist_m` | Minimum distance to any pedestrian during the run (m) |
| `mean_angular_acceleration` | Mean absolute angular acceleration (rad/s²) — lower = smoother |

### `run_all_experiments.sh`

Orchestrates full experiment sessions:

1. Writes the appropriate Nav2 param file for the current mode
2. Copies updated Python nodes to the installed package location
3. Launches Gazebo + Nav2 + all nodes as background processes
4. Waits for goal completion or timeout, then kills everything cleanly
5. Repeats for N runs, appending rows to the CSV

Supports environment variable overrides: `MODES`, `RUNS_PER_MODE`, `CSV_FILE`, `EXPERIMENT_TIMEOUT`, `ENABLE_RVIZ`, `GZ_GUI`.

### Other Scripts

| Script | Purpose |
| --- | --- |
| `run_ad_experiments.sh` | Batch runner for the standalone A\*/D\* Lite benchmark |
| `run_sequential_reboot.sh` | Runs multiple modes with automatic machine reboots between them to prevent Gazebo state corruption on overnight sessions |
| `setup_and_start.sh` | Configures `@reboot` cron to auto-resume experiments after reboot (requires sudo) |
| `convert_csv_time.py` | Off-line utility: converts sim `time_s` to wall-clock time given a measured RTF |
| `point.py` | Checks whether route waypoints fall in free space on `maps/cafe.yaml`, computes yaw between goals |
| `integration_guide.yaml` | Documents how `/social_costmap` is wired into Nav2 as a costmap plugin layer |

---

## Files Added / Modified vs Base Repository

| File | Status | Description |
| --- | --- | --- |
| `cpe631_ros2/social_nav_node.py` | **New** | Social costmap ROS2 node |
| `cpe631_ros2/ped_pose_extractor.py` | **New** | Gazebo → `/pedestrian_poses` bridge |
| `cpe631_ros2/peds.py` | Modified | Added random-walk pedestrian + direct pose publishing |
| `cpe631_ros2/data_collector.py` | **New** | Per-run metrics recorder |
| `cpe631_ros2/goal_sender.py` | **New** | Nav2 goal sequence sender |
| `src/dstar_planner.cpp` | **New** | D\* Lite Nav2 plugin (C++) |
| `include/cpe631_ros2/dstar_planner.hpp` | **New** | D\* Lite plugin header |
| `cpe631_ros2_dstar_planner.xml` | **New** | pluginlib registration |
| `CMakeLists.txt` | Modified | Added D\* Lite plugin build targets + new node installs |
| `package.xml` | Modified | Added C++ build dependencies |
| `launch/cafe_dynamic.launch.py` | **New** | Launch file for dynamic/social experiment modes |
| `launch/cafe.launch.py` | Modified | Added social node and data collector launch args |
| `param/nav2_dynamic_astar.yaml` | **New** | NavFn A\* mode config |
| `param/nav2_dynamic_conservative.yaml` | **New** | NavFn + MPPI conservative safety config |
| `param/nav2_dynamic_dstar.yaml` | **New** | D\* Lite mode config |
| `param/nav2_social_dynamic.yaml` | **New** | NavFn Dijkstra + social costmap |
| `param/nav2_social_astar.yaml` | **New** | NavFn A\* + social costmap |
| `param/nav2_social_smac.yaml` | **New** | Smac Hybrid A\* + MPPI + social costmap |
| `param/nav2_social_dstar.yaml` | **New** | D\* Lite + social costmap |
| `param/social_pose_bridge.yaml` | **New** | ROS-GZ bridge config for social costmap topic |
| `param/turtlebot3_burger_bridge_local.yaml` | **New** | Local ROS-GZ bridge config |
| `param/custom_bt.xml` | **New** | Custom Nav2 behavior tree XML |
| `codefiles/social_nav_node.py` | **New** | (same as above — kept here for standalone reference) |
| `codefiles/dstar_lite_planner.py` | **New** | Standalone A\* vs D\* Lite benchmark |
| `codefiles/run_all_experiments.sh` | **New** | Batch Nav2 experiment automation |
| `codefiles/run_ad_experiments.sh` | **New** | Standalone A\*/D\* batch runner |
| `codefiles/run_sequential_reboot.sh` | **New** | Multi-mode runner with auto-reboot |
| `codefiles/setup_and_start.sh` | **New** | Auto-resume cron setup |
| `codefiles/convert_csv_time.py` | **New** | Sim-time → wall-clock converter |
| `codefiles/point.py` | **New** | Route waypoint checker |
| `codefiles/integration_guide.yaml` | **New** | Social costmap Nav2 integration guide |
