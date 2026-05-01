# 基于 A* 与 Dijkstra 的 TurtleBot3 社交导航运动规划报告

## 1. 问题描述

本项目研究给定 cafe human environment 中的移动机器人导航问题。机器人为 TurtleBot3 Burger，目标是在已知二维地图中依次到达多个目标点，同时避开静态障碍物和动态行人。与普通最短路径导航不同，本项目关注 social navigation：机器人不仅要避免碰撞，还要尽量减少进入行人个人空间和亲密空间的次数，使导航行为更自然、更符合人类环境中的社交距离要求。

本报告只比较两种全局规划算法：

- A*：Nav2 NavFn planner 中 `use_astar: true`
- Dijkstra：Nav2 NavFn planner 中 `use_astar: false`

这里的 “D 算法” 指 Dijkstra algorithm，不引入 D*、RRT 或其他规划算法。局部控制器、行人模型、社交代价图和实验路线保持一致，因此对比重点是 A* 与 Dijkstra 在同一社交导航框架下的表现差异。

## 2. 假设条件

机器人假设：

- TurtleBot3 Burger 采用差速驱动模型。
- 机器人搭载 2D LiDAR、IMU 和里程计，可提供 `/scan`、`/odom` 和 TF 信息。
- 机器人通过 AMCL 在 `map` 坐标系中定位。
- 机器人可接收 Nav2 `NavigateToPose` 目标，并由 MPPI local controller 生成 `/cmd_vel`。

环境假设：

- cafe 地图已通过 SLAM 建图并保存为 `maps/cafe.yaml` 与 `maps/cafe.pgm`。
- 静态障碍物由全局/局部 costmap 表示。
- 行人为动态障碍物，位置由 `/pedestrian_poses` 发布。
- 行人运动包含直线往返和随机游走，代表 cafe 环境中的普通人流干扰。
- 仿真在 Gazebo + ROS2 Jazzy + Nav2 中运行。

## 3. 算法设计

系统总体流程如下：

`/pedestrian_poses` -> 行人速度估计与轨迹预测 -> 各向异性高斯社交代价 -> `/social_costmap` -> Nav2 costmap -> A* 或 Dijkstra 全局规划 -> MPPI 局部控制

核心改进是 Anisotropic Gaussian Predictive Social Costmap。该模块根据行人当前位置、估计速度和朝向预测未来短时间内的行人位置，并在预测位置周围生成各向异性高斯代价。行人前方和运动方向上的代价更高，侧向和后方代价较低。这样机器人会提前绕开行人未来可能经过的位置，而不是只对当前障碍物作反应。

A* 与 Dijkstra 的配置差异只在 NavFn planner：

| 模式 | 全局规划器 | Nav2 参数 | 局部控制器 | 社交代价图 |
| --- | --- | --- | --- | --- |
| Dijkstra | NavFn Dijkstra expansion | `use_astar: false` | MPPI | 开启 |
| A* | NavFn A* expansion | `use_astar: true` | MPPI | 开启 |

Dijkstra 通过代价扩展搜索整张 costmap，通常更稳定但可能扩展更多节点。A* 在搜索中加入启发式方向引导，理论上能更快向目标扩展，但在动态行人和高代价社交区域中可能更依赖启发式路径质量。

## 4. 性能指标

本项目使用 `data_collector.py` 记录以下指标：

| 指标 | CSV 字段 | 含义 |
| --- | --- | --- |
| 路径距离 | `path_length_m` | 机器人实际行驶距离，越短越高效 |
| 到达时间 | `time_s` | 从任务开始到完成所有目标点的时间 |
| 与人接触/近距离次数 | `encounters`、`personal_violations`、`intimate_violations` | 分别记录接近行人、进入个人空间、进入亲密空间的次数 |
| 最小行人距离 | `min_ped_dist_m` | 全程与行人的最近距离，越大越安全 |
| 社交自然性 | `mean_angular_acceleration` | 平均角加速度，越小代表转向越平滑 |

社交自然性的判断不只看是否到达目标，也看是否能保持较大行人间距、减少个人空间侵犯，并避免突然转向或贴近行人。

## 5. 仿真结果

实验数据来自 `three_modes_final_rows.csv`，其中 A* 与 Dijkstra 均为 25 次重复实验。下表只保留 A* 与 Dijkstra 两种算法的结果。

| 算法 | 成功率 | 平均路径距离 | 平均到达时间 | 平均 encounters | 平均 personal violations | 平均 intimate violations | 平均最小行人距离 | 平均角加速度 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Dijkstra | 25/25 = 100% | 32.33 m | 182.04 s | 1.76 | 1.84 | 0.96 | 8.50 m | 0.289 rad/s² |
| A* | 25/25 = 100% | 32.48 m | 151.07 s | 1.56 | 1.88 | 0.60 | 8.72 m | 0.258 rad/s² |

结果说明：

- 路径距离：A* 和 Dijkstra 的平均路径长度非常接近，说明两者都能找到可行且相似的全局路线。
- 到达时间：A* 平均用时更短，约比 Dijkstra 少 31 s，说明启发式搜索在该地图和目标路线中更有利于快速完成任务。
- 与人接触次数：A* 的 encounters 和 intimate violations 更低，说明在本组实验中 A* 与社交代价图结合后能更好地避免非常近距离的人机接触。
- 自然性：A* 的平均角加速度更低，轨迹更平滑；Dijkstra 稍慢且转向变化略大，但成功率同样稳定。

## 6. 结论

在本 cafe human environment 中，A* 与 Dijkstra 都能完成 TurtleBot3 的社交导航任务。加入预测式各向异性高斯社交代价图后，机器人会倾向于避开行人当前和未来可能出现的位置，因此导航行为比普通动态避障更符合社交距离要求。

从实验结果看，A* 在到达时间、亲密空间侵犯次数和轨迹平滑度上略优于 Dijkstra；Dijkstra 的优点是稳定性强，25 次实验全部成功。综合四类指标，本项目中 A* 更适合作为最终社交导航规划器，Dijkstra 可作为可靠 baseline。

## 7. 参考文献

1. E. W. Dijkstra, “A note on two problems in connexion with graphs,” Numerische Mathematik, 1959. https://doi.org/10.1007/BF01386390
2. P. E. Hart, N. J. Nilsson, and B. Raphael, “A Formal Basis for the Heuristic Determination of Minimum Cost Paths,” IEEE Transactions on Systems Science and Cybernetics, 1968. https://doi.org/10.1109/TSSC.1968.300136
3. Nav2 Documentation, “NavFn Planner.” https://docs.nav2.org/configuration/packages/configuring-navfn.html
4. Nav2 Documentation, “Costmap 2D.” https://docs.nav2.org/configuration/packages/configuring-costmaps.html
5. Nav2 Documentation, “Model Predictive Path Integral Controller.” https://docs.nav2.org/configuration/packages/configuring-mppic.html
6. E. T. Hall, The Hidden Dimension, 1966.
