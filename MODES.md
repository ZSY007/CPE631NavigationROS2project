# 实验模式说明

本项目在 ROS 2 Jazzy + Nav2 + Gazebo Sim 环境下，对比九种导航配置在动态行人场景（4 个行人）中的表现。

---

## 公共基础设施

### 机器人与仿真

- 平台：TurtleBot3 Burger，差速驱动，半径 ≈ 105 mm
- 仿真：Gazebo Sim（gz-sim 8），cafe 场景，10 m × 22 m
- 定位：AMCL（自适应蒙特卡洛定位）
- 局部控制器：全模式统一使用 **MPPI Controller**
  - 配置：[param/nav2_dynamic_conservative.yaml](param/nav2_dynamic_conservative.yaml) `plugin: nav2_mppi_controller::MPPIController`

### 行人管理

- 行人模型由 `cpe631_peds` 节点生成并直接发布 `/pedestrian_poses`（`geometry_msgs/PoseArray`）
  - 代码：[cpe631_ros2/peds.py](cpe631_ros2/peds.py)
  - 启动条件：[launch/cafe_dynamic.launch.py L385–398](launch/cafe_dynamic.launch.py)

### 数据采集

- `data_collector` 节点记录每轮：路径长度、耗时、行人相遇次数、个人空间侵犯（< 1.2 m）、亲密空间侵犯（< 0.45 m）、最近距离
  - 代码：[cpe631_ros2/data_collector.py](cpe631_ros2/data_collector.py)

---

## 模式一览

| 模式 | 全局规划器 | 局部代价图社会层 | 全局代价图社会层 | BT 重规划频率 | 近距离触发清图 |
|------|-----------|----------------|----------------|-------------|-------------|
| baseline | NavFn | ✗ | ✗ | 1 Hz | ✗ |
| dynamic | NavFn | ✗ | ✗ | 1 Hz | ✗ |
| dynamic_astar | A\* (NavFn) | ✗ | ✗ | 1 Hz | ✗ |
| dynamic_dstar | D\* Lite | ✗ | ✗ | 1 Hz | ✗ |
| social | NavFn | ✓ | ✗ | 1 Hz | ✗ |
| social_astar | A\* (NavFn) | ✓ | ✗ | 1 Hz | ✗ |
| social_smac | SMAC (Hybrid-A\*) | ✓ | ✗ | 1 Hz | ✗ |
| social_dstar | D\* Lite | ✓ | ✗ | 1 Hz | ✗ |
| **social_dstar_plus** | D\* Lite | ✓ | **✓** | **2 Hz** | **✓** |

---

## 各模式详细说明

### 1. baseline

**目的**：无行人基准，测量纯导航耗时与路径长度的理论下界。

- 行人：禁用（`enable_peds:=false`）
- 规划器：`nav2_navfn_planner::NavfnPlanner`（Dijkstra）
- 参数文件：[param/nav2_dynamic_conservative.yaml](param/nav2_dynamic_conservative.yaml)

---

### 2. dynamic

**目的**：有行人但无任何社会感知的基准，评估传统导航在动态场景的表现。

- 规划器：`nav2_navfn_planner::NavfnPlanner`（Dijkstra）
- 行人处理：仅通过 `ObstacleLayer` / `VoxelLayer` 识别行人为普通障碍物
- 参数文件：[param/nav2_dynamic_conservative.yaml](param/nav2_dynamic_conservative.yaml)
- BT：[param/custom_bt.xml](param/custom_bt.xml) — `<RateController hz="1.0">`

---

### 3. dynamic_astar

**目的**：用 A\* 替换 Dijkstra，对比启发式搜索在动态场景的路径质量差异。

- 规划器：`nav2_navfn_planner::NavfnPlanner`，**`use_astar: true`**
  - 配置：[param/nav2_dynamic_astar.yaml](param/nav2_dynamic_astar.yaml)
- 改进点：A\* 使用欧氏距离启发函数，倾向于生成更直接的路径；在障碍物稀疏场景比 Dijkstra 快，但在拥挤场景中路径质量与 Dijkstra 相近

---

### 4. dynamic_dstar

**目的**：引入增量式规划器 D\* Lite，利用其"逆向搜索 + 增量更新"特性应对动态障碍。

- 规划器：**`cpe631_ros2::DStarPlanner`**（自定义 D\* Lite 插件）
  - 插件声明：[include/cpe631_ros2/dstar_planner.hpp](include/cpe631_ros2/dstar_planner.hpp) — `class DStarPlanner : public nav2_core::GlobalPlanner`
  - 实现：[src/dstar_planner.cpp](src/dstar_planner.cpp)
  - 插件注册：`src/dstar_planner.cpp` 末行 `PLUGINLIB_EXPORT_CLASS`
- 参数文件：[param/nav2_dynamic_dstar.yaml](param/nav2_dynamic_dstar.yaml)
- D\* Lite 原理：从目标逆向构建搜索树，当代价图更新时只重算受影响的节点（而非全图重算），适合代价频繁变化的场景

---

### 5. social

**目的**：基准社会导航，验证将行人社会代价注入**局部代价图**是否改善行人空间指标。

- 规划器：`nav2_navfn_planner::NavfnPlanner`
- **新增**：`social_nav_node` 发布 `/social_costmap`（`OccupancyGrid`），局部代价图通过 `StaticLayer` 订阅该话题
  - 核心代码：[cpe631_ros2/social_nav_node.py](cpe631_ros2/social_nav_node.py) — `class SocialNavNode`
  - 模型：对每个行人建立**各向异性高斯代价**，沿速度方向拉伸（预测行人未来位置），横向收窄
  - 关键参数（在 launch 文件中配置）：
    - `prediction_dt = 0.5 s`：速度预测步长
    - `peak_cost = 85`：代价峰值（0–100 占用网格）
    - `longitudinal_sigma = 0.80 m` / `lateral_sigma = 0.35 m`：纵/横向高斯宽度
    - `lethal_core_radius = 0.25 m`：行人中心致死区半径
  - 启动条件：[launch/cafe_dynamic.launch.py L145–154](launch/cafe_dynamic.launch.py) `social_condition`
- 局部代价图社会层配置：[param/nav2_social_dynamic.yaml](param/nav2_social_dynamic.yaml)
- 参数文件：[param/nav2_social_dynamic.yaml](param/nav2_social_dynamic.yaml)

---

### 6. social_astar

**目的**：A\* + 社会局部代价图的组合，测试启发式规划器能否从社会代价中获益。

- 规划器：`nav2_navfn_planner::NavfnPlanner`，`use_astar: true`
- 社会层：同 social 模式（局部代价图）
- 参数文件：[param/nav2_social_astar.yaml](param/nav2_social_astar.yaml)
- 实验结果：personal violations 反而最高（13.9 均值），说明 A\* 的直线偏好与社会代价的配合不佳

---

### 7. social_smac

**目的**：引入 SMAC Hybrid-A\* 规划器，该规划器支持运动学约束，路径更平滑。

- 规划器：`nav2_smac_planner::SmacPlannerHybrid`（Nav2 内置）
- 社会层：局部代价图，同 social 模式
- 参数文件：[param/nav2_social_smac.yaml](param/nav2_social_smac.yaml)
- SMAC 特点：基于 Hybrid-A\*，考虑车辆转弯半径，生成曲率连续的路径；在社会代价场景下 intimate violations 最低（2.1 均值），但成功率较低（14/17）

---

### 8. social_dstar

**目的**：D\* Lite + 社会局部代价图，测试增量规划器能否利用社会代价避让行人。

- 规划器：`cpe631_ros2::DStarPlanner`
- 社会层：局部代价图，同 social 模式
- 参数文件：[param/nav2_social_dstar.yaml](param/nav2_social_dstar.yaml)
- 问题：出现"卡死"事件（行人群包围，D\* 等待下次 BT tick 才重规划，最长卡 400 s），intimate violations 均值高达 7.9

---

### 9. social_dstar_plus ⭐

**目的**：针对 social_dstar 的两个架构缺陷进行改进。

#### Feature 1：社会代价注入**全局**代价图

- **问题**：social_dstar 的社会代价仅在局部代价图，D\* Lite 全局路径规划时不感知行人，路径会穿越高行人密度区
- **改进**：全局代价图新增 `social_layer`（StaticLayer 订阅 `/social_costmap`），D\* 从规划阶段就绕开行人预测位置
- 配置：[param/nav2_dstar_plus.yaml](param/nav2_dstar_plus.yaml)
  ```yaml
  global_costmap:
    plugins: ["static_layer", "social_layer", "inflation_layer"]
    social_layer:
      plugin: "nav2_costmap_2d::StaticLayer"
      map_topic: /social_costmap
      subscribe_to_updates: true
      trinary_costmap: false
  ```
- 全局代价图更新频率提升：`update_frequency: 10.0 Hz`（默认 5 Hz）

#### Feature 2：2 Hz BT 重规划 + 近距离触发清图

- **问题**：BT `RateController` 默认 1 Hz，行人逼近时最多等 1 s 才重规划，在拥挤场景导致卡死
- **改进 A**：BT 重规划频率提升到 **2 Hz**（每 0.5 s 重算一次全局路径）
  - BT 文件：[param/custom_bt_dstar_plus.xml L12](param/custom_bt_dstar_plus.xml) — `<RateController hz="2.0">`
  - 对比默认：[param/custom_bt.xml L13](param/custom_bt.xml) — `<RateController hz="1.0">`
- **改进 B**：`replan_trigger` 节点，当任意行人距机器人 < 1.5 m 时**立即**调用 `ClearEntireCostmap` 服务，强制全局代价图刷新，触发即时重规划（不等下次 BT tick）
  - 代码：[cpe631_ros2/replan_trigger.py](cpe631_ros2/replan_trigger.py)
    - `TRIGGER_DIST = 1.5 m`（第 26 行）
    - `COOLDOWN_S = 2.0 s`（第 27 行，避免频繁清图）
  - 订阅：`/pedestrian_poses`、`/amcl_pose`
  - 发布：`/near_pedestrian`（Bool，供监控）
  - 服务调用：`/global_costmap/clear_entirely_global_costmap`
  - 启动条件：[launch/cafe_dynamic.launch.py L456–468](launch/cafe_dynamic.launch.py) — `dstar_plus_condition`（navigation=true AND planner_variant=dstarplus）

---

## 实验结果摘要（n ≈ 20，final attempt only）

| 模式 | 成功率 | time 中位(s) | personal 中位 | **intimate 中位** |
|------|--------|-------------|--------------|-----------------|
| baseline | 20/20 | 100.3 | 0 | 0 |
| dynamic | 18/20 | 98.1 | 12.0 | 3.5 |
| dynamic_astar | 18/20 | 96.5 | 11.0 | 4.0 |
| dynamic_dstar | 20/20 | 99.6 | 12.0 | 3.0 |
| social | 15/15 | 102.7 | 9.0 | 4.0 |
| social_astar | 14/15 | 113.5 | 13.5 | 5.0 |
| social_smac | 14/17 | 117.1 | 11.0 | 3.0 |
| social_dstar | 19/20 | 103.9 | 11.0 | 5.0 |
| **social_dstar_plus** | **19/20** | **101.1** | **9.0** | **2.0 ✓** |

social_dstar_plus intimate violations 中位数 **2.0**，为所有动态模式最优，同时时间中位数（101.1 s）优于所有其他 social 模式。
