# Social Navigation Optimization Summary

## Background

This project is an EE631 ROS2 social navigation simulation based on TurtleBot3 + Nav2 in the cafe human environment. The main improvement is an Anisotropic Gaussian Predictive Social Costmap that publishes `/social_costmap` and injects continuous social costs into the Nav2 local costmap.

The first full experiment produced:

- `baseline`: 10/10 succeeded
- `dynamic`: 7/10 succeeded
- `social`: 4/10 succeeded

The social mode improved pedestrian clearance but caused many timeouts. Most failures reached goal 4/5 and timed out at goal 5/5, which suggested that the social costmap was too conservative rather than completely broken.

This document summarizes the optimization work done after analyzing `2.md`.

## Files Modified

Main modified files:

- `codefiles/social_nav_node.py`
- `codefiles/peds.py`
- `codefiles/data_collector.py`
- `launch/cafe_dynamic.launch.py`

The same updated files were also synchronized to the working copy under:

- `/home/dx/CPE631-Navigation-ROS2-main/`

The actual runnable project path is:

- `/home/dx/ros2_ws/A/CPE631-Navigation-ROS2-main/`

## 1. Social Costmap Tuning

File:

- `codefiles/social_nav_node.py`
- `launch/cafe_dynamic.launch.py`

Problem:

The original social costmap was too conservative. Long-horizon constant-velocity prediction and large Gaussian cost fields caused the robot to avoid pedestrians even when they were not strongly relevant to the current route.

Changes:

- Reduced future cost decay factor:
  - `gamma: 0.80 -> 0.68`
- Reduced social cost strength:
  - `peak_cost: 95 -> 70`
- Reduced Gaussian influence range:
  - `cutoff_sigma: 3.0 -> 2.5`
  - `longitudinal_sigma: 0.90 -> 0.70`
  - `speed_sigma_scale: 0.45 -> 0.35`
- Reduced group cost:
  - `group_cost_ratio: 0.70 -> 0.45`
- Shortened prediction horizon:
  - `base_lookahead: 2.0 -> 1.5`
  - `speed_lookahead_gain: 1.0 -> 0.7`
  - `min_lookahead: 2.0 -> 1.5`
  - `max_lookahead: 5.0 -> 3.5`
- Reduced cost from pedestrians behind the robot:
  - `behind_ped_weight: 0.30 -> 0.15`

Expected effect:

The robot should remain socially aware near pedestrians, but should be less likely to get stuck or time out because of distant or far-future predicted costs.

## 2. Robot-Relevance Weighting

File:

- `codefiles/social_nav_node.py`

Problem:

Some social-mode runs timed out even when the nearest pedestrian distance was large. This indicated that distant pedestrians were still producing enough cost to affect planning.

Changes:

Added relevance weighting:

- Full cost when a predicted pedestrian position is within `4.0 m` of the robot or its predicted short-term trajectory.
- Linearly reduced cost between `4.0 m` and `6.0 m`.
- Minimum retained cost weight: `0.15`.

New parameters:

- `social_relevance_distance`
- `social_relevance_soft_margin`
- `min_relevance_weight`

Expected effect:

Pedestrians that are unlikely to interact with the robot should no longer dominate the local costmap.

## 3. Time-Normalized Velocity Smoothing

File:

- `codefiles/social_nav_node.py`

Problem:

The old velocity smoother used a fixed `alpha`, even though pedestrian pose messages may not arrive at perfectly uniform intervals.

Old behavior:

```python
velocity = alpha * instant_velocity + (1.0 - alpha) * velocity
```

New behavior:

```python
alpha_t = 1.0 - exp(-dt / velocity_tau)
velocity = alpha_t * instant_velocity + (1.0 - alpha_t) * velocity
```

New parameter:

- `velocity_tau`

Expected effect:

Pedestrian speed estimation becomes more physically consistent when message timing varies.

## 4. Better Pedestrian Identity Matching

File:

- `codefiles/social_nav_node.py`

Problem:

Greedy nearest-neighbor matching can switch pedestrian IDs when pedestrians cross paths.

Change:

Replaced greedy matching with a small-scale global optimal assignment. Since this project only tracks around five pedestrians, the assignment can be solved using standard-library combinations and permutations without adding a SciPy dependency.

Expected effect:

Pedestrian identity and velocity tracking should remain more stable during path crossings.

## 5. Longer Wandering Detection Window

File:

- `codefiles/social_nav_node.py`

Problem:

The original wandering detection history window was too short. A 5-frame window at around 10 Hz only captures about 0.5 seconds of motion, which is not enough to identify the random-walk pedestrian.

Change:

- Tracker history window increased:
  - `window: 5 -> 25`

Expected effect:

The random-walk pedestrian is more likely to be treated as wandering or uncertain, instead of being assigned an overconfident directional Gaussian.

## 6. Use Pedestrian Orientation From PoseArray

Files:

- `codefiles/peds.py`
- `codefiles/social_nav_node.py`
- `launch/cafe_dynamic.launch.py`

Problem:

`peds.py` internally knew each pedestrian yaw, but the published `PoseArray` originally did not include orientation. The social costmap then had to infer heading only from position differences.

Changes:

- `peds.py` now publishes pedestrian orientation in each `Pose`.
- `social_nav_node.py` reads pose orientation and converts it to heading.
- The observed heading is used to initialize trackers and assist heading estimation during slow or stopped frames.

New parameters:

- `use_ped_orientation`
- `ped_orientation_offset_deg`

Current launch value:

```python
use_ped_orientation: True
ped_orientation_offset_deg: -90.0
```

Expected effect:

Pedestrian heading estimation should be more stable, especially when the pedestrian is moving slowly or has just started moving.

## 7. Lightweight Interaction / Game-Style Prediction

File:

- `codefiles/social_nav_node.py`
- `launch/cafe_dynamic.launch.py`

Problem:

The earlier version only used a single robot trajectory extrapolated from odometry. This was interaction-aware, but still weaker than the game-style idea described in `2.md`.

Change:

Added multiple short-term robot trajectory hypotheses. The node now generates several possible robot trajectories, such as left-turn, straight, and right-turn hypotheses. During pedestrian prediction, each pedestrian reacts to the nearest robot trajectory hypothesis at each future step.

New parameters:

- `robot_traj_hypotheses`
- `robot_traj_yaw_rate_span`
- `robot_prediction_min_speed`

Current launch values:

```python
robot_traj_hypotheses: 5
robot_traj_yaw_rate_span: 0.8
robot_prediction_min_speed: 0.12
```

Important note:

This is not a full Nav2 MPPI critic implementation that evaluates every sampled MPPI trajectory. It is a lightweight trajectory-dependent interaction model inside the social costmap node. For a course project, it is a practical and explainable approximation of implicit game-style prediction.

## 8. Pedestrian Motion Update Fixes

File:

- `codefiles/peds.py`

Problems:

- Pedestrian step size was hardcoded as `speed / 10.0`.
- This assumes the timer always runs exactly at 10 Hz.

Changes:

- Step size now uses actual elapsed time:

```python
step_size = speed * dt
```

- The node also stores and publishes each pedestrian yaw.

Expected effect:

Pedestrian motion is more consistent when timer timing varies.

## 9. Data Collection Improvements

File:

- `codefiles/data_collector.py`

Problems:

- Angular acceleration used receive time instead of the message timestamp.
- Pedestrian distance checking was too slow at `0.5 s`, which could miss short personal-space violations.
- After adding the faster timer, the old timer had to be removed to avoid double counting.

Changes:

- `cmd_vel` angular acceleration now uses `msg.header.stamp`.
- Mean angular acceleration is averaged by sample count.
- Distance checking period changed to a configurable value:

```python
distance_check_period: 0.1
```

- The old `create_timer(0.5, self.record_loop)` was removed.

Expected effect:

Metrics should better reflect actual navigation behavior, especially for proxemics violations and smoothness.

## 10. Launch Integration Fixes

File:

- `launch/cafe_dynamic.launch.py`

Problems:

- `ped_pose_extractor` and `cpe631_peds` both published to `/pedestrian_poses`.
- Launch parameters were overriding the optimized defaults in `social_nav_node.py`.

Changes:

- `cpe631_peds` is now the only authoritative publisher to `/pedestrian_poses`.
- `ped_pose_extractor` now publishes only to a debug topic:

```python
/pedestrian_poses_gz_debug
```

- Social navigation launch parameters were updated to match the optimized code defaults.

Expected effect:

The social costmap should receive a single consistent pedestrian stream, and launch should no longer accidentally restore old conservative parameters.

## Remaining Work Not Implemented

The following ideas from `2.md` are still not fully implemented:

1. Replace `peds.py` subprocess-based `gz service` calls with a true Gazebo Transport or ROS service client.
   - Current version still uses subprocess for spawning and pose updates.
   - This is the largest remaining engineering improvement.

2. Full MPPI candidate-trajectory game planning.
   - Current version uses multiple trajectory hypotheses inside `social_nav_node.py`.
   - Full implementation would require writing or modifying a Nav2 MPPI critic/plugin to evaluate pedestrian interaction for every sampled trajectory.

3. True probabilistic CV + Random Walk mixture.
   - Current version has stationary / slow / fast / wandering modes plus uncertainty growth.
   - It is not yet a formal probability mixture model.

## Verification Performed

Python syntax checks were run successfully on the modified files:

```bash
python3 -m py_compile \
  codefiles/social_nav_node.py \
  codefiles/peds.py \
  codefiles/data_collector.py \
  launch/cafe_dynamic.launch.py
```

## Suggested Next Experiment

Run a new 10-run social experiment using the updated code and compare against the previous result:

- Previous social success rate: `4/10`
- Target after optimization: `6/10` to `8/10`

Key metrics to compare:

- Success rate
- Timeout count
- `min_ped_dist_m`
- `personal_violations`
- `intimate_violations`
- `mean_angular_acceleration`
- Average navigation time

The ideal outcome is higher success rate and lower timeout count while keeping intimate-space violations near zero.
