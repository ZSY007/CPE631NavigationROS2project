# Experiment Commands

Run commands from the active experiment workspace unless noted otherwise:

```bash
cd /home/dx/ros2_ws/A/CPE631-Navigation-ROS2-main
```

If `colcon build` cannot find the package, move or link the project under a ROS 2 workspace source directory such as `~/ros2_ws/src/`.

## Build

```bash
colcon build --packages-select cpe631_ros2 --symlink-install
source install/setup.bash
```

Each new terminal must source the workspace:

```bash
source /home/dx/ros2_ws/A/CPE631-Navigation-ROS2-main/install/setup.bash
```

## Use the Improved Code in `codefiles/`

Copy the improved files into the ROS 2 package and rebuild:

```bash
cp codefiles/ped_pose_extractor.py cpe631_ros2/ped_pose_extractor.py
cp codefiles/social_nav_node.py    cpe631_ros2/social_nav_node.py
cp codefiles/data_collector.py     cpe631_ros2/data_collector.py
cp codefiles/peds.py               cpe631_ros2/peds.py

colcon build --packages-select cpe631_ros2 --symlink-install
source install/setup.bash
```

## Mapping

Terminal 1:

```bash
ros2 launch cpe631_ros2 cafe.launch.py mapping:=true
```

Terminal 2:

```bash
ros2 launch cpe631_ros2 teleop.launch.py model:=burger
```

Save the map:

```bash
ros2 run nav2_map_server map_saver_cli -f maps/cafe
```

## Navigation Experiments

Baseline, without pedestrians:

```bash
ros2 launch cpe631_ros2 cafe_dynamic.launch.py navigation:=true social_navigation:=false enable_peds:=false
```

Dynamic navigation, with pedestrians but without social costmap:

```bash
ros2 launch cpe631_ros2 cafe_dynamic.launch.py navigation:=true social_navigation:=false enable_peds:=true
```

Social navigation, with anisotropic Gaussian social costmap:

```bash
ros2 launch cpe631_ros2 cafe_dynamic.launch.py navigation:=true social_navigation:=true enable_peds:=true
```

Social Smac, with Smac Hybrid A* and local/global social costmap:

```bash
ros2 launch cpe631_ros2 cafe_dynamic.launch.py navigation:=true social_navigation:=true enable_peds:=true planner_variant:=smac
```

## Data Collection

Open a separate terminal for the data collector. Change `mode` to match the active experiment:

```bash
source /home/dx/ros2_ws/A/CPE631-Navigation-ROS2-main/install/setup.bash
ros2 run cpe631_ros2 data_collector --ros-args -p mode:=baseline
```

Open another terminal for route goals:

```bash
source /home/dx/ros2_ws/A/CPE631-Navigation-ROS2-main/install/setup.bash
ros2 run cpe631_ros2 goal_sender
```

Use a shorter dwell time:

```bash
ros2 run cpe631_ros2 goal_sender --ros-args -p dwell_time:=0.5
```

Use a custom route:

```bash
ros2 run cpe631_ros2 goal_sender --ros-args \
  -p route:="0.55,-6.34,-1.06;2.8,-3.0,0.0;2.5,5.8,3.14;-3.5,2.6,1.57;-1.0,0.5,-1.57"
```

Improved data collector with explicit thresholds:

```bash
ros2 run cpe631_ros2 data_collector --ros-args \
  -p mode:=social \
  -p encounter_threshold:=1.0 \
  -p personal_zone:=1.2 \
  -p intimate_zone:=0.45 \
  -p timestamp_filename:=true
```

## CSV Columns

| Column | Meaning |
| --- | --- |
| `encounters` | Number of entries within 1.0 m of a pedestrian. |
| `personal_violations` | Number of entries within 1.2 m personal space. |
| `intimate_violations` | Number of entries within 0.45 m intimate space. |
| `min_ped_dist_m` | Minimum observed distance to a pedestrian. |
| `mean_angular_acceleration` | Smoothness metric in rad/s^2; lower is smoother. |

## Node and Topic Checks

```bash
ros2 node list | grep -E "ped_pose|social_nav|gz_social"
ros2 topic hz /pedestrian_poses
ros2 topic hz /social_costmap
ros2 param get /local_costmap/local_costmap plugins
```

## Experiment Matrix

| Experiment | Launch arguments | Data collector mode |
| --- | --- | --- |
| Baseline | `social_navigation:=false enable_peds:=false` | `mode:=baseline` |
| Dynamic | `social_navigation:=false enable_peds:=true` | `mode:=dynamic` |
| Social | `social_navigation:=true enable_peds:=true` | `mode:=social` |
| Social A* | `social_navigation:=true enable_peds:=true planner_variant:=astar` | `mode:=social_astar` |
| Social Smac | `social_navigation:=true enable_peds:=true planner_variant:=smac` | `mode:=social_smac` |
| D* Social | `social_navigation:=true enable_peds:=true planner_variant:=dstar` | `mode:=social_dstar` |

## Batch Runs

Run selected modes:

```bash
MODES=dynamic,social RUNS_PER_MODE=5 ./codefiles/run_all_experiments.sh
```

Run a 25-trial batch:

```bash
RUNS_PER_MODE=25 \
CSV_FILE=/home/dx/ros2_ws/A/experiment_results_25run.csv \
./codefiles/run_all_experiments.sh
```

Run the final Social Smac tuned batch:

```bash
RUNS_PER_MODE=25 \
MODES=social_smac \
MAX_RETRY=1 \
CSV_FILE=/home/dx/CPE631-Navigation-ROS2-main/A/social_smac_tuned_25.csv \
./codefiles/run_all_experiments.sh
```

## Final Result File

The final consolidated result file is:

```text
/home/dx/CPE631-Navigation-ROS2-main/FRC.CSV
```

It preserves all attempts and adds wall-clock estimate fields:

```text
actual_rtf,time_wall_actual_s,time_wall_actual_min
```
