# 实验模式说明

本项目在 ROS 2 Jazzy + Nav2 + Gazebo Sim 环境下，对比九种导航配置在动态行人场景中的表现。

---

## 公共基础设施

### 机器人与仿真

- 平台：TurtleBot3 Burger，差速驱动，半径 ≈ 105 mm
- 仿真：Gazebo Sim（gz-sim 8），cafe 场景，约 10 m × 22 m
- 定位：AMCL（自适应蒙特卡洛定位）
- 局部控制器：全模式统一使用 **MPPI Controller**（`nav2_mppi_controller::MPPIController`）

---

### 行人管理模块：`cpe631_peds`

**代码：** [cpe631_ros2/peds.py](cpe631_ros2/peds.py) — `PedestrianManager`

启动条件：[launch/cafe_dynamic.launch.py L385–398](launch/cafe_dynamic.launch.py)（`peds_condition`: mapping=false AND enable_peds=true）

**5 个行人的轨迹配置：**

| 行人 | 路线 | 速度 | 模式 |
|------|------|------|------|
| ped_1 | (3.0, 5.0) ↔ (3.0, −8.0) | 1.2 m/s | 线性往返 |
| ped_2 | (−3.0, 5.0) ↔ (−3.0, 0.0) | 0.8 m/s | 线性往返 |
| ped_3 | (2.0, −4.5) ↔ (−3.0, −4.0) | 0.8 m/s | 线性往返 |
| ped_4 | (0.0, 6.0) ↔ (3.5, 6.5) | 0.5 m/s | 线性往返 |
| ped_5 | (−4.0~4.0, −3.0~7.0) | 0.6 m/s | 随机游走 |

**更新逻辑（`_update_positions`）：**
- 以 `update_period = 0.2 s`（5 Hz）步进各行人位置，线性模式到达终点后反向，随机模式到达随机航点后重新采样
- 每次更新后向 `/pedestrian_poses`（`geometry_msgs/PoseArray`）发布全部行人位姿（含朝向四元数）
- 提供 `/reset_pedestrians` 服务，每轮实验开始前将所有行人复位到初始位置（`_reset_pedestrians_cb`）

---

### 目标发送模块：`goal_sender`

**代码：** [cpe631_ros2/goal_sender.py](cpe631_ros2/goal_sender.py) — `GoalSender`

顺序向 Nav2 `NavigateToPose` action 发送 5 个路点，同时向 `/goal_pose` 发布（触发 `data_collector` 自动开始计时）。

**默认路线（覆盖高行人密度区）：**

| 路点 | 坐标 (x, y, yaw) | 行人区域 |
|------|-----------------|---------|
| G1 | (0.50, −6.50, −1.06) | ped_1 / ped_3 走廊 |
| G2 | (2.00, −2.50,  0.00) | ped_1 附近 |
| G3 | (0.50,  6.50,  3.14) | ped_4 区域 |
| G4 | (−3.50, 3.50,  1.57) | ped_2 走廊 |
| G5 | (−1.50, 1.50, −1.57) | ped_5 随机区 |

**关键参数：**
- `dwell_time = 1.5 s`：到达后停留时间
- `experiment_timeout = 720 s`（仿真时间看门狗）
- `experiment_wall_timeout`：真实时间看门狗（可选）

退出码：0=全部成功，2=aborted，4=超时

---

### 数据采集模块：`data_collector`

**代码：** [cpe631_ros2/data_collector.py](cpe631_ros2/data_collector.py) — `DataCollector`

以 `distance_check_period = 0.1 s`（10 Hz）轮询所有行人距离，记录以下指标：

| 指标 | 计算方式 | 阈值 |
|------|---------|------|
| `path_length_m` | 累计里程计位移（跳变 > 0.5 m 过滤） | — |
| `time_s` | stop_time − start_time（ROS 时钟） | — |
| `encounters` | **进入式计数**：行人首次进入 1.0 m 圆加 1 | 1.0 m |
| `personal_violations` | **进入式计数**：行人首次进入个人空间加 1 | 1.2 m（Proxemics） |
| `intimate_violations` | **进入式计数**：行人首次进入亲密空间加 1 | 0.45 m（Proxemics） |
| `min_ped_dist_m` | 整轮机器人中心到行人中心最小值（center-to-center） | — |
| `mean_angular_acceleration` | `/cmd_vel` 角速度变化率均值 \|Δω/Δt\|，越小越平滑 | — |

**进入式计数说明：** 使用持久化集合 `close_ped_indices` / `personal_ped_indices` / `intimate_ped_indices` 跟踪当前在区内的行人，只有**新进入**的行人才计数，持续处于区内的行人不重复计数。

---

## 社会代价图模块：`social_nav_node`

**代码：** [cpe631_ros2/social_nav_node.py](cpe631_ros2/social_nav_node.py)

这是所有 `social_*` 模式的核心，包含两个类：

### `PedestrianTracker`（行人状态跟踪器）

每个行人对应一个独立实例，负责：

**速度估计（`update`）：**
- 指数平滑：`alpha_t = 1 − exp(−dt / velocity_tau)`（`velocity_tau = 0.35 s`）
- 跳变滤波：单步位移 > `max_jump_dist = 1.0 m` 时忽略
- 速度上限：`max_ped_speed = 1.8 m/s`
- 朝向融合：速度方向 80% + 观测朝向 20%

**游走检测（`is_wandering`）：**
- 净位移 / 轨迹总长 < 0.3 → 判定为随机游走

**三模式轨迹预测（`predict`）：**

| 模式 | 触发条件 | 高斯形状 |
|------|---------|---------|
| A 静止/游走 | speed < 0.05 m/s 或 is_wandering() | 圆形（sigma = 0.45 m） |
| B 慢速 | 0.05 ≤ speed < 0.3 m/s | 弱各向异性（sigma_long 轻度拉长） |
| C 快速 | speed ≥ 0.3 m/s | 强各向异性（速度方向拉长，侧向收窄） |

**自适应预测步长：**
```
lookahead = clip(base_lookahead + speed_lookahead_gain × speed,
                 min_lookahead, max_lookahead)
= clip(1.5 + 0.7 × speed, 1.5, 3.5)  秒
```

**不确定性增长（非线性）：**
```
sigma_long[k] = sqrt(sigma_long_base² + q_scale × k^1.3 × vel_uncertainty)
sigma_short[k] = sqrt(sigma_short_base² + lateral_q_scale × k^1.3)
```
上限：`max_uncertainty_scale = 1.6`

**朝向惯性约束：** 每步最大转向 30°，避免预测出不合理的急转弯路径

**交互预测（`_nearest_robot_prediction`）：** 机器人轨迹假设存在时，行人会向远离机器人的方向偏移（`ped_react_gain = 0.15`）

---

### `SocialNavNode`（社会代价图发布器）

**完整处理流程（`_publish`，5 Hz）：**

```
1. TF 查询机器人当前位姿
2. 清除超时 > 1.0 s 的失活轨迹
3. 生成 3 条机器人轨迹假设（直行/左转/右转，yaw_rate ∈ [−0.8, +0.8] rad/s）
4. 对每个行人：
   a. 计算相关性权重（距机器人 < 3.5 m → 1.0；> 5.5 m → 0.10；中间线性）
   b. 计算接近系数（approaching boost，最多 +30%）
   c. 写入即时代价（当前位置的高斯，instant_cost_ratio = 0.85）
   d. 可选标记致死核心（radius = 0.25 m，value = 90）
   e. 逐步叠加预测代价（gamma^k 衰减，k=1..N 步）
5. 叠加组团代价（相互距离 < 1.5 m 的行人组，peak × 0.45）
6. 发布 OccupancyGrid（189 × 447 cells @ 0.05 m，topic: /social_costmap）
```

**各向异性高斯（`_accumulate_gaussian`）：**
```
long_axis  = Δx·cos(θ) + Δy·sin(θ)   # 沿速度方向投影
short_axis = −Δx·sin(θ) + Δy·cos(θ)  # 垂直方向投影

# 不对称：行人身后衰减更快
sigma_l = sigma_long  if long_axis ≥ 0
        = behind_sigma_ratio × sigma_long  otherwise (behind_sigma_ratio=0.5)

cost += peak × exp(−0.5 × [(long/sigma_l)² + (short/sigma_short)²])
```
使用 `numpy.maximum` 叠加（保留各 Gaussian 最大值，避免求和溢出）

**完整参数列表（launch 文件注入）：** [launch/cafe_dynamic.launch.py L400–452](launch/cafe_dynamic.launch.py)

| 类别 | 参数 | 默认值 | 含义 |
|------|------|--------|------|
| 高斯形状 | `longitudinal_sigma` | 0.80 m | 速度方向初始标准差 |
| 高斯形状 | `lateral_sigma` | 0.35 m | 垂直方向初始标准差 |
| 高斯形状 | `peak_cost` | 85 | 代价峰值（0–100） |
| 高斯形状 | `cutoff_sigma` | 2.5 | 截断半径倍数 |
| 时间衰减 | `gamma` | 0.72 | 预测步代价衰减系数 |
| 时间衰减 | `prediction_dt` | 0.5 s | 预测时间步长 |
| 速度估计 | `velocity_alpha` | 0.35 | 平滑系数 |
| 速度估计 | `velocity_tau` | 0.35 s | 指数平滑时间常数 |
| 不确定性 | `q_scale` | 0.08 | 纵向不确定性增长率 |
| 不确定性 | `lateral_q_scale` | 0.02 | 横向不确定性增长率 |
| 不确定性 | `uncertainty_exponent` | 1.3 | 增长非线性指数 |
| 即时代价 | `instant_cost_ratio` | 0.85 | 当前位置代价占峰值比 |
| 即时代价 | `instant_sigma` | 0.75 m | 即时代价高斯宽度 |
| 即时代价 | `approach_cost_gain` | 0.3 | 逼近方向代价增益 |
| 致死核心 | `lethal_core_radius` | 0.25 m | 致死区半径 |
| 相关性 | `social_relevance_distance` | 3.5 m | 全权重距离 |
| 相关性 | `min_relevance_weight` | 0.10 | 远距最小权重 |
| 组团代价 | `group_cost_ratio` | 0.45 | 组团代价占峰值比 |
| 机器人假设 | `robot_traj_hypotheses` | 3 | 机器人轨迹候选数 |
| 不对称 | `behind_ped_weight` | 0.15 | 行人身后代价缩放 |

---

## D\* Lite 规划器

**代码：** [src/dstar_planner.cpp](src/dstar_planner.cpp) — 插件类声明 [include/cpe631_ros2/dstar_planner.hpp](include/cpe631_ros2/dstar_planner.hpp)

实现 `nav2_core::GlobalPlanner` 接口，注册为 `cpe631_ros2::DStarPlanner`。

### 算法核心（`createPlan`）

**D\* Lite 原理：** 从**目标反向**构建最短路径树（与 Dijkstra 正向不同），当代价图更新时只重算受影响的节点，而非全图重算，适合代价频繁变化的动态场景。

**关键数据：**
- `g[cell]`：当前估计的从 cell 到目标的代价
- `rhs[cell]`：当前最优后继计算出的单步更新值
- `theta_next[cell]`：当前最优后继节点（any-angle 路径用）
- 优先队列按 `(k1=min(g,rhs)+h, k2=min(g,rhs))` 排序

**主循环（`computeShortestPath`）：**
```
while top_key < calc_key(start)  or  |g[start] - rhs[start]| > ε:
  pop (key, index) from open
  if g > rhs:
    g = rhs  // 局部一致
    update_vertex 所有前驱
  else:
    g = ∞    // 重置，触发传播
    update_vertex(index)
    update_vertex 所有前驱
```

**代价计算：**
```
traversalCost(from, to) = distance × (neutral_cost + cost_factor × map_cost_normalized)
= distance × (1.0 + 3.0 × cell_cost / 252)
```
`theta_any_angle = true` 时额外尝试跨多格的视线捷径（`lineTraversalCost`，Bresenham 平均代价）

**路径平滑（`smoothPath`）：** 从起点出发逐锚点跳过视线可达的中间节点，减少转折。

**配置参数：** [param/nav2_dynamic_dstar.yaml](param/nav2_dynamic_dstar.yaml)

| 参数 | 值 | 含义 |
|------|------|------|
| `tolerance` | 0.75 m | 目标容忍半径 |
| `theta_any_angle` | true | 允许非格点方向捷径 |
| `smooth_path` | true | 启用路径平滑 |
| `neutral_cost` | 1.0 | 空白格基础代价 |
| `cost_factor` | 3.0 | 代价图权重倍数 |
| `lethal_cost_threshold` | 253 | 致死障碍阈值 |
| `max_iterations` | 2 000 000 | 防死循环上限 |
| `max_pose_spacing` | 0.10 m | 输出路径点间距 |

---

## 模式一览

| 模式 | 全局规划器 | 局部代价图社会层 | 全局代价图社会层 | BT 重规划频率 | 近距离触发清图 |
|------|-----------|----------------|----------------|-------------|-------------|
| baseline | NavFn（Dijkstra） | ✗ | ✗ | 1 Hz | ✗ |
| dynamic | NavFn（Dijkstra） | ✗ | ✗ | 1 Hz | ✗ |
| dynamic_astar | NavFn（A\*） | ✗ | ✗ | 1 Hz | ✗ |
| dynamic_dstar | D\* Lite | ✗ | ✗ | 1 Hz | ✗ |
| social | NavFn（Dijkstra） | ✓ | ✗ | 1 Hz | ✗ |
| social_astar | NavFn（A\*） | ✓ | ✗ | 1 Hz | ✗ |
| social_smac | SMAC Hybrid-A\* | ✓ | ✗ | 1 Hz | ✗ |
| social_dstar | D\* Lite | ✓ | ✗ | 1 Hz | ✗ |
| **social_dstar_plus** | D\* Lite | ✓ | **✓** | **2 Hz** | **✓** |

---

## 各模式详细说明

### 1. baseline

无行人基准，测量纯导航耗时与路径长度的理论下界。

- 规划器：`nav2_navfn_planner::NavfnPlanner`（Dijkstra）
- 行人：禁用（`enable_peds:=false`）
- 参数文件：[param/nav2_dynamic_conservative.yaml](param/nav2_dynamic_conservative.yaml)

---

### 2. dynamic

有行人但无任何社会感知，评估传统导航在动态场景的基准表现。

- 规划器：`nav2_navfn_planner::NavfnPlanner`（Dijkstra）
- 行人处理：仅通过 `ObstacleLayer` / `VoxelLayer` 识别为普通障碍
- 参数文件：[param/nav2_dynamic_conservative.yaml](param/nav2_dynamic_conservative.yaml)
- BT：[param/custom_bt.xml L13](param/custom_bt.xml) — `<RateController hz="1.0">`

---

### 3. dynamic_astar

用 A\* 替换 Dijkstra，对比启发式搜索在动态场景的差异。

- 规划器：`nav2_navfn_planner::NavfnPlanner` + **`use_astar: true`**
- A\* 相比 Dijkstra：使用欧氏距离启发函数，倾向生成更直接的路径
- 参数文件：[param/nav2_dynamic_astar.yaml](param/nav2_dynamic_astar.yaml)

---

### 4. dynamic_dstar

引入增量式规划器，利用 D\* Lite 代价变化时局部重算的特性应对动态障碍。

- 规划器：**`cpe631_ros2::DStarPlanner`**（自定义实现，详见上方 D\* Lite 章节）
- 参数文件：[param/nav2_dynamic_dstar.yaml](param/nav2_dynamic_dstar.yaml)
- 局部代价图：无社会层，仅 `ObstacleLayer + VoxelLayer + InflationLayer`

---

### 5. social

基准社会导航模式，验证行人社会代价注入**局部**代价图的效果。

- 规划器：`nav2_navfn_planner::NavfnPlanner`（Dijkstra）
- 新增：`social_nav_node` 发布 `/social_costmap`，局部代价图通过 `StaticLayer`（`subscribe_to_updates: true`）订阅动态更新
  - `social_nav_node` 启动条件：[launch/cafe_dynamic.launch.py L145–154](launch/cafe_dynamic.launch.py) `social_condition`
- **局部代价图层（`local_costmap`）：** `ObstacleLayer + VoxelLayer + social_layer + InflationLayer`
- **全局代价图层（`global_costmap`）：** `StaticLayer + InflationLayer`（无社会层）
- 参数文件：[param/nav2_social_dynamic.yaml](param/nav2_social_dynamic.yaml)

---

### 6. social_astar

A\* + 社会局部代价图，测试启发式规划器能否从社会代价中获益。

- 规划器：NavFn + `use_astar: true`
- 代价图配置：同 social（局部有社会层，全局无）
- 参数文件：[param/nav2_social_astar.yaml](param/nav2_social_astar.yaml)
- 实验发现：personal violations 反而最高（中位 13.5），说明 A\* 的直线偏好在社会代价场景下适得其反

---

### 7. social_smac

引入运动学约束规划器，生成曲率连续的平滑路径。

- 规划器：**`nav2_smac_planner::SmacPlannerHybrid`**（Nav2 内置 Hybrid-A\*）
  - 考虑差速机器人转弯半径，生成 Reeds-Shepp 曲线路径
- 代价图配置：同 social
- 参数文件：[param/nav2_social_smac.yaml](param/nav2_social_smac.yaml)
- 实验发现：intimate violations 均值 2.8（与 dstar_plus 持平），但成功率仅 82%（14/17），且耗时最长（中位 117 s）

---

### 8. social_dstar

D\* Lite + 社会局部代价图，测试增量规划器与社会代价的组合效果。

- 规划器：`cpe631_ros2::DStarPlanner`
- 代价图配置：同 social（局部有社会层，**全局无**）
- 参数文件：[param/nav2_social_dstar.yaml](param/nav2_social_dstar.yaml)
- BT：[param/custom_bt.xml](param/custom_bt.xml)（1 Hz 重规划）
- **问题：** 出现"卡死"事件（行人群包围，全局路径未感知社会代价，D\* 等待下次 BT tick 才重规划，最长卡 400 s），导致 intimate violations 均值高达 7.9

---

### 9. social_dstar_plus ⭐

针对 social_dstar 的两个架构缺陷的改进版本。

#### Feature 1：社会代价注入全局代价图

**问题：** social_dstar 的社会代价仅在局部代价图，D\* Lite 全局路径规划时完全不感知行人位置，路径规划会穿越高行人密度区，路径设计阶段就注定要靠近行人。

**改进：** 全局代价图新增 `social_layer`，D\* Lite 在规划阶段即可绕开行人预测位置：

```yaml
# param/nav2_dstar_plus.yaml — global_costmap
plugins: ["static_layer", "social_layer", "inflation_layer"]
social_layer:
  plugin: "nav2_costmap_2d::StaticLayer"
  map_topic: /social_costmap
  subscribe_to_updates: true   # 接收动态 OccupancyGrid 更新
  map_subscribe_transient_local: false
  trinary_costmap: false        # 保留梯度代价而非三值化
update_frequency: 10.0  # 原 5 Hz 提升为 10 Hz，更快同步社会代价
```

参数文件：[param/nav2_dstar_plus.yaml](param/nav2_dstar_plus.yaml)

#### Feature 2：2 Hz BT 重规划 + 近距离触发清图

**问题：** BT `RateController` 默认 1 Hz，行人逼近时最多等 1 s 才触发重规划。在拥挤场景（行人从侧面快速逼近）中，这 1 s 的延迟导致机器人陷入局部极小无法脱身。

**改进 A — BT 频率加倍：**

BT 文件 [param/custom_bt_dstar_plus.xml L12](param/custom_bt_dstar_plus.xml)：
```xml
<RateController hz="2.0">   <!-- 原来是 1.0 Hz -->
  <RecoveryNode number_of_retries="2" name="ComputePathToPose">
    <ComputePathToPose .../>
    ...
  </RecoveryNode>
</RateController>
```
对比原始 BT：[param/custom_bt.xml L13](param/custom_bt.xml) — `hz="1.0"`

**改进 B — 近距离触发即时清图：**

新节点 `replan_trigger`（[cpe631_ros2/replan_trigger.py](cpe631_ros2/replan_trigger.py)）：

```
订阅 /pedestrian_poses, /amcl_pose
每帧：
  min_dist = min(distance(robot, ped_i) for ped_i)
  发布 /near_pedestrian (Bool)
  if min_dist < TRIGGER_DIST(1.5 m) AND elapsed > COOLDOWN(2.0 s):
    调用 /global_costmap/clear_entirely_global_costmap 服务
    → 强制代价图刷新 → D* 立即以新的社会代价重规划
    （不等下次 2 Hz BT tick）
```

关键常量（[replan_trigger.py L26–27](cpe631_ros2/replan_trigger.py)）：
- `TRIGGER_DIST = 1.5 m`
- `COOLDOWN_S = 2.0 s`（防止连续触发抖动）

启动条件：[launch/cafe_dynamic.launch.py L456–468](launch/cafe_dynamic.launch.py)，仅当 `navigation=true AND planner_variant=dstarplus` 时启动。

---

## 代价图层配置对比

| 模式 | local_costmap 层 | global_costmap 层 |
|------|----------------|-----------------|
| dynamic* | Obstacle + Voxel + Inflation | Static + Inflation |
| social* (非 plus) | Obstacle + Voxel + **Social** + Inflation | Static + Inflation |
| social_dstar_plus | Obstacle + Voxel + **Social** + Inflation | Static + **Social** + Inflation |

---

## 实验结果摘要（n ≈ 20，final attempt only）

| 模式 | 成功/总 | time 均/中(s) | personal 均/中 | **intimate 均/中** | min_dist 均(m) |
|------|---------|-------------|--------------|-------------------|--------------|
| baseline | 20/20 | 100.5/100.3 | 0/0 | 0/0 | — |
| dynamic | 18/20 | 108.2/98.1 | 12.5/12.0 | 3.6/3.5 | 0.21 |
| dynamic_astar | 18/20 | 115.3/96.5 | 12.5/11.0 | 3.7/4.0 | 0.25 |
| dynamic_dstar | 20/20 | 130.2/99.6 | 15.8/12.0 | 6.4/3.0 | 0.08 |
| social | 15/15 | 110.8/102.7 | 11.0/9.0 | 4.5/4.0 | 0.17 |
| social_astar | 14/15 | 122.8/113.5 | 13.9/13.5 | 5.6/5.0 | 0.14 |
| social_smac | 14/17 | 148.4/117.1 | 15.4/11.0 | 3.2/3.0 | 0.20 |
| social_dstar | 19/20 | 131.2/103.9 | 12.4/11.0 | 7.9/5.0 | 0.14 |
| **social_dstar_plus** | **19/20** | **115.4/101.1** | **11.6/9.0** | **2.6/2.0** ✓ | 0.21 |

social_dstar_plus intimate violations 中位数 **2.0**，所有动态模式最优；time 中位数 101.1 s 优于所有其他 social 模式；相比 social_dstar intimate 均值下降 **67%**（7.9 → 2.6）。
