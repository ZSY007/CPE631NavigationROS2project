# cpe631_ros2 (Gazebo Sim / ROS2 Jazzy)

## Overview
This project runs a Gazebo Sim cafe environment with a TurtleBot3 Burger, supports SLAM (mapping) and Nav2 navigation, and can optionally spawn moving pedestrians.

Workflow:
1) Mapping (SLAM) in a static environment, save a map.
2) Navigation with Nav2 using the saved map (dynamic pedestrians optional).

## Dependencies
ROS2 Jazzy and these packages:
- ros-jazzy-ros-gz-sim
- ros-jazzy-ros-gz-bridge
- ros-jazzy-ros-gz-image
- ros-jazzy-slam-toolbox
- ros-jazzy-nav2-bringup
- ros-jazzy-turtlebot3-gazebo
- ros-jazzy-turtlebot3-navigation2
- ros-jazzy-rviz2

## Build
```bash
cd <your_ros2_ws>
colcon build --packages-select cpe631_ros2
source install/setup.bash
```
Source install/setup.bash every time when you open a new terminal

## Run
### 1) Mapping (static scene, no pedestrians)
From this package directory (so relative paths work):
```bash
cd <your_ros2_ws>
```
```bash
ros2 launch cpe631_ros2 cafe.launch.py mapping:=true
```
- Use teleop to drive and build the map:
```bash
ros2 launch cpe631_ros2 teleop.launch.py model:=burger
```

### 2) Save the map
Relative output location (recommended):
```bash
mkdir -p maps
ros2 run nav2_map_server map_saver_cli -f maps/cafe
```
This creates:
- `maps/cafe.yaml`
- `maps/cafe.pgm`

### 3) Navigation (load map, Nav2)
Example (navigation without pedestrians):
```bash
ros2 launch cpe631_ros2 cafe.launch.py navigation:=true map_file:=<path to map>/cafe.yaml enable_peds:=false
```
In RViz:
- Click **2D Pose Estimate** to set the initial pose.
- Click **Nav2 Goal** to send a goal.

## Launch arguments
`launch/cafe.launch.py` supports:
- `mapping` (true/false): enable SLAM (no pedestrians)
- `navigation` (true/false): enable Nav2
- `enable_peds` (true/false): enable pedestrians (ignored when mapping=true)
- `map_file` (path): map YAML for navigation
- `model` (burger/waffle/waffle_pi)
- `use_sim_time` (true/false)

Example (navigation with pedestrians):
```bash
ros2 launch cpe631_ros2 cafe.launch.py navigation:=true map_file:=<path to map>/cafe.yaml enable_peds:=true
```

## Configurable parameters
### Robot initial position
- Set in `worlds/cafe.world` under the TurtleBot3 include:
  - `<pose>1.0 0.0 0.05 0 0 0</pose>`

### Pedestrian positions and paths
- Static standing human pose in `launch/cafe.launch.py`.
- Moving actor paths in:
  - `models/person_walking_actor_1/model.sdf`
  - `models/person_walking_actor_2/model.sdf`
  - `models/person_walking_actor_3/model.sdf`

### Sensor configuration
- TurtleBot3 model and sensor settings:
  - `models/turtlebot3_burger/model.sdf`
- Lidar resolution, FOV, range, noise are under `<sensor name="hls_lfcd_lds">`.
- IMU noise parameters are under `<sensor name="tb3_imu">`.

### Topics used
- Command velocity: `/cmd_vel`
- Lidar: `/scan`
- Odometry: `/odom`
- TF: `/tf`, `/tf_static`
- Map: `/map`
- Initial pose: `/initialpose`
- AMCL pose: `/amcl_pose`
- Goal: `/goal_pose`

## Nav2 navigation configuration
Nav2 is launched via `nav2_bringup` and uses this package's parameter file:
- Params file in use: `param/nav2_burger.yaml` (passed as `params_file` in `launch/cafe.launch.py`)

### Which config is used?
This repo supports two separate Nav2 configurations:
- **Default navigation**: `launch/cafe.launch.py` + `param/nav2_burger.yaml` (DWB local controller).
- **Dynamic conservative navigation**: `launch/cafe_dynamic.launch.py` + `param/nav2_dynamic_conservative.yaml` (NavFn global planner + MPPI local controller + conservative safety settings).

### Dynamic obstacle avoidance (conservative)
For a more conservative setup that better handles dynamic obstacles (e.g., pedestrians), use the dedicated launch + params (does not modify `param/nav2_burger.yaml`):
```bash
ros2 launch cpe631_ros2 cafe_dynamic.launch.py navigation:=true map_file:=<path to map>/cafe.yaml enable_peds:=true
```
This launch uses:
- Params: `param/nav2_dynamic_conservative.yaml` (NavFn global planner + MPPI local controller)
- RViz: `rviz/navigation.rviz` (same RViz config as `cafe.launch.py`)
- Map for RViz: it runs `cpe631_map_republisher` and remaps RViz to `/map_viz` to avoid missing a transient-local map at startup.

What changes vs the default “static” navigation config (`param/nav2_burger.yaml`):
- **Local controller**: MPPI (`nav2_mppi_controller::MPPIController`) replaces DWB (`dwb_core::DWBLocalPlanner`).
- **Max speed**: `controller_server.ros__parameters.FollowPath.vx_max = 0.5`.
- **More reactive costmaps**: higher `local_costmap.*.update_frequency` / `publish_frequency` and higher `global_costmap.*.update_frequency` / `publish_frequency`.
- **More conservative safety**: `collision_monitor` stop/slow polygons enabled with a lower slowdown ratio.
- **Startup robustness**: AMCL is configured to publish an initial `map->odom` transform, and RViz map is fed via `/map_viz`.

Key knobs (in `param/nav2_dynamic_conservative.yaml`):
- Max linear speed: `controller_server.ros__parameters.FollowPath.vx_max` (default `0.5 m/s`)
- Costmap update rates: `local_costmap.local_costmap.ros__parameters.update_frequency` / `global_costmap.global_costmap.ros__parameters.update_frequency`
- Safety: `collision_monitor.ros__parameters.PolygonStop.radius`, `PolygonSlow.slowdown_ratio`

Default stack when `navigation:=true` (high-level):
- Localization: `nav2_amcl` (AMCL particle filter)
- Map: `nav2_map_server` loads the map YAML (CLI `map_file:=...` overrides `map_server.yaml_filename`)
- Global planner: `planner_server` loads a *planner plugin* (default in this repo: `nav2_navfn_planner::NavfnPlanner`)
- Local controller: `controller_server` loads a *controller plugin* (default: `dwb_core::DWBLocalPlanner`, dynamic conservative: `nav2_mppi_controller::MPPIController`)
- Navigator / BT: `bt_navigator` provides the `NavigateToPose` action used by RViz’s **Nav2 Goal** tool

Planner / controller plugins are not separate ROS nodes; they are classes provided by ROS packages (e.g., `nav2_navfn_planner`, `nav2_smac_planner`, `nav2_mppi_controller`) and loaded into `planner_server` / `controller_server` via `pluginlib`.

### How to swap planners / controllers (dynamic environments)
You can replace plugins in `param/nav2_burger.yaml` (default) or `param/nav2_dynamic_conservative.yaml` (dynamic conservative):
- Global planner options:
  - `nav2_smac_planner/SmacPlannerHybrid`
  - `nav2_smac_planner/SmacPlannerLattice`
- Local controller options:
  - `nav2_regulated_pure_pursuit_controller/RegulatedPurePursuitController`
  - `nav2_mppi_controller::MPPIController`

#### Parameters to change (planner/controller)
Global planner (`planner_server`):
- `planner_server.ros__parameters.planner_plugins`: list of enabled planner IDs (default: `["GridBased"]`)
- `planner_server.ros__parameters.<PlannerID>.plugin`: plugin class for that ID (default: `GridBased.plugin: "nav2_navfn_planner::NavfnPlanner"`)
- Common planner knobs in this repo: `tolerance`, `use_astar`, `allow_unknown` under the selected planner ID

Local controller (`controller_server`):
- `controller_server.ros__parameters.controller_plugins`: list of enabled controller IDs (default: `["FollowPath"]`)
- `controller_server.ros__parameters.<ControllerID>.plugin`: plugin class for that ID (default: `FollowPath.plugin: "dwb_core::DWBLocalPlanner"`)

Behavior tree:
- `bt_navigator.ros__parameters.default_nav_to_pose_bt_xml` / `default_nav_through_poses_bt_xml`

### What `planner_server` / `controller_server` expose
- `bt_navigator` action server: `/navigate_to_pose` (`nav2_msgs/action/NavigateToPose`) — RViz **Nav2 Goal** sends goals here.
- `planner_server` action servers: `/compute_path_to_pose`, `/compute_path_through_poses` — returns `nav_msgs/Path` (often visualized in RViz on `/plan`).
- `controller_server` action server: `/follow_path` (`nav2_msgs/action/FollowPath`) — consumes the path and produces velocity commands.

### How the plan becomes `/cmd_vel`
Nav2 does not have the robot "subscribe to `/plan` and execute it". Instead:
1) RViz sends a `nav2_msgs/action/NavigateToPose` goal to `bt_navigator` (action server: `/navigate_to_pose`).
2) `bt_navigator` (behavior tree) calls `planner_server` actions to compute a `nav_msgs/Path` (also published on `/plan` for RViz).
3) `bt_navigator` sends that `Path` to `controller_server` using the `nav2_msgs/action/FollowPath` action.
4) The controller plugin runs in a loop, using TF + `local_costmap` to generate velocity commands.
5) `controller_server` publishes `cmd_vel` (this repo uses stamped cmd_vel). `velocity_smoother` / `collision_monitor` may post-process it before the sim consumes it.

### Dynamic obstacles in Nav2 (what to use)
Nav2 doesn’t have a special “dynamic obstacle global planner” in the sense of predicting moving pedestrians in the global plan. In practice, dynamic obstacle avoidance is handled by:
- **Local costmap** obstacle/voxel layers (marking + clearing from `/scan`).
- **Local controller** (e.g., MPPI) optimizing trajectories against the local costmap.
- **Safety layers** like `nav2_collision_monitor` (stop/slowdown zones) and `nav2_velocity_smoother`.

### Nav2 docs
- Main docs: https://docs.nav2.org/
- Configuration guide: https://docs.nav2.org/configuration/index.html
- Planner plugins: https://docs.nav2.org/configuration/packages/configuring-planner-server.html
- Controller plugins: https://docs.nav2.org/configuration/packages/configuring-controller-server.html
- Behavior trees (BT Navigator): https://docs.nav2.org/configuration/packages/configuring-bt-navigator.html
- MPPI controller: https://docs.nav2.org/configuration/packages/configuring-mppi-controller.html
- Nav2 actions: https://github.com/ros-navigation/navigation2/tree/main/nav2_msgs/action
- `FollowPath` action: https://github.com/ros-navigation/navigation2/blob/main/nav2_msgs/action/FollowPath.action

## Notes
- Pedestrians are disabled in mapping mode by default.
- RViz configuration lives at `rviz/navigation.rviz`.
