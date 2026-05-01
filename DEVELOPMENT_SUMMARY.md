# Development Summary

## Background

This project extends the original EE631 ROS 2 navigation project for social navigation, human-robot interaction, and quantitative performance comparison. The improved implementation is kept under `codefiles/`, while the original ROS 2 package is preserved as much as possible for simulation compatibility.

## 1. Code Review and Bug Fixes

### `codefiles/ped_pose_extractor.py`

| Issue | Severity | Fix |
| --- | --- | --- |
| Unused imports such as `StringVec` and `GzPoseArray` could break startup in some environments. | High | Removed invalid or unused imports. |
| Hard-coded slicing with `poses[1:4]` only captured three pedestrians and silently ignored `ped_4`. | High | Added the explicit `ped_indices` parameter to support any configured pedestrian list. |
| `self.ped_indices = None` was declared but never used. | Medium | Removed dead code. |

Added behavior:

- `ped_indices` parameter, defaulting to `"1,2,3,4,5"`.
- First-frame debug output showing the incoming pose-array length.
- Clear out-of-range warnings when a configured pedestrian index is invalid.
- Backward-compatible fallback slicing when `ped_indices` is empty.

### `codefiles/social_nav_node.py`

| Issue | Severity | Fix |
| --- | --- | --- |
| First-frame pedestrian velocity was exponentially smoothed from zero, underestimating early speed and shortening predicted trajectories. | Medium | Added an `_initialized` flag so the first observation is assigned directly. |
| `grid.reshape(-1).tolist()` was used at 10 Hz on a large array. | Low | Replaced it with `grid.flatten().tolist()`. |

### `codefiles/integration_guide.yaml`

| Issue | Severity | Fix |
| --- | --- | --- |
| `subscribe_to_updates: false` caused the social costmap layer to load only once at startup. | Medium | Changed it to `subscribe_to_updates: true` so pedestrian updates propagate to Nav2. |

## 2. Feature Expansion

### `codefiles/data_collector.py`

| Metric | Description |
| --- | --- |
| `personal_violations` | Number of entries into personal space, using a 1.2 m proxemic threshold. |
| `intimate_violations` | Number of entries into intimate space, using a 0.45 m proxemic threshold. |
| `min_ped_dist_m` | Minimum pedestrian distance observed during the run. |
| `mean_angular_acceleration` | Smoothness metric in rad/s^2; lower is smoother. |

Other improvements:

- CSV output can use timestamped filenames to avoid overwriting old experiments.
- `/cmd_vel` is subscribed to for angular acceleration estimation.
- Manual mode selection includes `dynamic`.
- Social-space event counting is edge-triggered, so each entry is counted once.

### `codefiles/peds.py`

The original setup used four pedestrians moving on simple back-and-forth paths. The improved version adds `ped_5`, a random-walk pedestrian that selects waypoints inside:

```text
(-4, 4) x (-3, 7)
```

This makes the scene less deterministic and closer to a real indoor human environment.

## 3. Planner Comparison Configuration

| Parameter file | Global planner | Local controller | Purpose |
| --- | --- | --- | --- |
| `nav2_burger.yaml` | Dijkstra (`use_astar: false`) | DWB | Baseline. |
| `nav2_dynamic_conservative.yaml` | A* (`use_astar: true`) | MPPI | Dynamic-navigation comparison. |
| `nav2_social_dynamic.yaml` | A* (`use_astar: true`) | MPPI + social costmap | Social navigation. |
| `nav2_social_smac.yaml` | Smac Hybrid A* | MPPI + local/global social costmaps | Tuned social-Smac experiment. |

The social layer was changed to subscribe to updates so moving pedestrians affect the Nav2 costmap continuously.

## 4. Documentation Updates

The project documentation was reorganized into clearer English files:

- `PROJECT_GUIDE.md`: full project guide, experiment modes, final result file, and review workflow.
- `DEVELOPMENT_SUMMARY.md`: implementation changes, fixes, and feature summary.
- `EXPERIMENT_COMMANDS.md`: build, launch, data collection, and validation commands.

## 5. Submitted Code Files

| File | Description |
| --- | --- |
| `ped_pose_extractor.py` | Pedestrian pose extraction with explicit indexing and bounds checks. |
| `social_nav_node.py` | Anisotropic Gaussian predictive social costmap. |
| `data_collector.py` | Experiment metric collection with proxemic and smoothness metrics. |
| `peds.py` | Five-pedestrian scenario with one random-walk pedestrian. |
| `integration_guide.yaml` | Nav2 social costmap integration reference. |

## 6. Metric Coverage

| Required metric | Implementation | Quantification |
| --- | --- | --- |
| Path distance | `path_length_m` | Accumulated odometry path length with jump filtering. |
| Arrival time | `time_s` | Simulation-time duration from goal dispatch to result. |
| Human encounters | `encounters`, `personal_violations`, `intimate_violations` | Edge-triggered threshold crossings. |
| Social naturalness | `mean_angular_acceleration` | Mean angular acceleration; lower indicates smoother motion. |

## 7. Final Data Products

- `FRC.CSV` is the final consolidated results file.
- `A/` contains raw experiment CSV files, run logs, and supporting experiment artifacts.
- `FRC.CSV` keeps all attempts rather than compressing each experiment to one final row.

| Mode | Rows | Successful rows |
| --- | ---: | ---: |
| `social` | 29 | 20 |
| `social_astar` | 29 | 22 |
| `social_dstar` | 28 | 23 |
| `social_smac` | 39 | 19 |
