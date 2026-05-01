0:43] 等待 data_collector 写入 CSV (最多 30s) ...
[19:10:43] ✔ data_collector 已退出
[19:10:44] ✘ 实验 dynamic_astar #5 异常退出 (exit=1, attempt=0)
[19:10:44] ⚠ 实验 dynamic_astar #5 最终失败，已跳过

[19:10:49] ═══ dynamic_astar 实验 6/25 ═══
[19:10:49] 尝试 0/1 ...
[19:10:51] 重置行人到初始位置...
[19:10:52] ✔ 行人已重置
[19:10:54] 启动 data_collector (mode=dynamic_astar, exp=6, attempt=0) ...
[19:10:54]   data_collector PID = 6209
[19:10:56] 启动 goal_sender (timeout=720s) ...
[19:10:57]   goal_sender 退出码 = 1
[19:10:57]   result = error (写入 /tmp/result_dynamic_astar_6_0.txt)
[19:10:57] 发送 SIGUSR1 给 data_collector ...
[19:10:57] 等待 data_collector 写入 CSV (最多 30s) ...
[19:10:57] ✔ data_collector 已退出
[19:10:58] ✘ 实验 dynamic_astar #6 异常退出 (exit=1, attempt=0)
[19:10:58] ⚠ 实验 dynamic_astar #6 最终失败，已跳过
^C[19:10:58] 清理所有后台进程...

2,social_dstar,succeeded,1,36.683,109.88,8,13,3,0.191,0.5782

[04:08:33] ═══ dynamic_astar 实验 25/25 ═══
[04:08:33] 尝试 0/1 ...
[04:08:35] 重置行人到初始位置...
[04:08:36] ✔ 行人已重置
[04:08:38] 启动 data_collector (mode=dynamic_astar, exp=25, attempt=0) ...
[04:08:38]   data_collector PID = 118156
[04:08:40] 启动 goal_sender (timeout=720s) ...
[04:14:13]   goal_sender 退出码 = 0
[04:14:13]   result = succeeded (写入 /tmp/result_dynamic_astar_25_0.txt)
[04:14:13] 发送 SIGUSR1 给 data_collector ...
[04:14:13] 等待 data_collector 写入 CSV (最多 30s) ...
[04:14:14] ✔ 数据已落盘
[04:14:16] ✔ 实验 dynamic_astar #25 成功 (attempt=0)
[04:14:21] ✔ 模式 dynamic_astar 全部实验完成，清理...
[04:14:21] 清理所有后台进程...
The daemon has been stopped
The daemon has been started
[04:14:35] 清理完成
[04:14:35] Mode 间冷却 15s ...

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  开始模式: dynamic_dstar
  navigation=true  social_navigation=false  enable_peds=true  planner=dstar
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
[04:14:50] 启动 cafe_dynamic.launch.py (mode=dynamic_dstar, planner=dstar) ...
[04:14:50]   launch PID = 122660
[04:14:50] 等待 45s 让仿真完全启动...
[04:15:35] ✔ 仿真已启动
[04:15:40] 验证 Nav2 节点就绪状态...
[04:15:42] ✔ Nav2 bt_navigator 已激活

[04:15:42] ═══ dynamic_dstar 实验 1/25 ═══
[04:15:42] 尝试 0/1 ...
[04:15:43] 重置行人到初始位置...
[04:15:44] ✔ 行人已重置
[04:15:46] 启动 data_collector (mode=dynamic_dstar, exp=1, attempt=0) ...
[04:15:46]   data_collector PID = 123935
[04:15:48] 启动 goal_sender (timeout=720s) ...
[04:21:49]   goal_sender 退出码 = 2
[04:21:49]   result = aborted (写入 /tmp/result_dynamic_dstar_1_0.txt)
[04:21:49] 发送 SIGUSR1 给 data_collector ...
[04:21:49] 等待 data_collector 写入 CSV (最多 30s) ...
[04:21:50] ✔ data_collector 已退出
[04:21:51] ⚠ 实验 dynamic_dstar #1 被 Nav2 abort (attempt=0)
[04:21:51] 重置机器人位姿并重试...
[04:21:51] 重置机器人位姿 → (1.0, 0.0, yaw=0.0)
data: true

publisher: beginning loop
publishing #1: geometry_msgs.msg.PoseWithCovarianceStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=0, nanosec=0), frame_id='map'), pose=geometry_msgs.msg.PoseWithCovariance(pose=geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point(x=1.0, y=0.0, z=0.0), orientation=geometry_msgs.msg.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)), covariance=array([0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
       0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
       0., 0.])))

requester: making request: nav2_msgs.srv.ClearEntireCostmap_Request(request=std_msgs.msg.Empty())

response:
nav2_msgs.srv.ClearEntireCostmap_Response(response=std_msgs.msg.Empty())

waiting for service to become available...
requester: making request: nav2_msgs.srv.ClearEntireCostmap_Request(request=std_msgs.msg.Empty())

response:
nav2_msgs.srv.ClearEntireCostmap_Response(response=std_msgs.msg.Empty())

[04:22:03] ✔ 机器人位姿已重置
[04:22:08] 尝试 1/1 ...
[04:22:09] 重置行人到初始位置..



:34:16]   goal_sender 退出码 = 4
[04:34:16]   result = timeout (写入 /tmp/result_dynamic_dstar_1_1.txt)
[04:34:16] 发送 SIGUSR1 给 data_collector ...
[04:34:16] 等待 data_collector 写入 CSV (最多 30s) ...
[04:34:17] ✔ data_collector 已退出
[04:34:18] ⚠ 实验 dynamic_dstar #1 超时 (attempt=1)
[04:34:18] 重置机器人位姿 → (1.0, 0.0, yaw=0.0)
data: true

publisher: beginning loop
publishing #1: geometry_msgs.msg.PoseWithCovarianceStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=0, nanosec=0), frame_id='map'), pose=geometry_msgs.msg.PoseWithCovariance(pose=geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point(x=1.0, y=0.0, z=0.0), orientation=geometry_msgs.msg.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)), covariance=array([0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
       0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
       0., 0.])))

requester: making request: nav2_msgs.srv.ClearEntireCostmap_Request(request=std_msgs.msg.Empty())

response:
nav2_msgs.srv.ClearEntireCostmap_Response(response=std_msgs.msg.Empty())

requester: making request: nav2_msgs.srv.ClearEntireCostmap_Request(request=std_msgs.msg.Empty())

response:
nav2_msgs.srv.ClearEntireCostmap_Response(response=std_msgs.msg.Empty())

[04:34:31] ✔ 机器人位姿已重置
[04:34:31] ⚠ 实验 dynamic_dstar #1 最终失败，已跳过

[04:34:36] ═══ dynamic_dstar 实验 2/25 ═══
[04:34:36] 尝试 0/1 ...
[04:34:37] 重置行人到初始位置...
[04:34:38] ✔ 行人已重置
[04:34:40] 启动 data_collector (mode=dynamic_dstar, exp=2, attempt=0) ...
[04:34:40]   data_collector PID = 138310
[04:34:42] 启动 goal_sender (timeout=720s) ...


dx@dx-VMware-Virtual-Platform:~/CPE631-Navigation-ROS2-main$ grep "peak_cost" ~/CPE631-Navigation-ROS2-main/codefiles/social_nav_node.py
grep "plugins:" ~/CPE631-Navigation-ROS2-main/param/nav2_social_dynamic.yaml | grep -v dock | grep -v progress | grep -v goal | grep -v controller | grep -v behavior | grep -v planner
        self.declare_parameter("peak_cost", 55)
        self.peak_cost = float(self.get_parameter("peak_cost").value)
                amplitude = self.peak_cost * (self.gamma ** (k + 1))
        amplitude = self.peak_cost * self.instant_cost_ratio
                        self.peak_cost
      plugins: ["obstacle_layer", "voxel_layer", "social_layer", "inflation_layer"]
      plugins: ["static_layer", "inflation_layer"]
dx@dx-VMware-Virtual-Platform:~/CPE631-Navigation-ROS2-main$ 

dx@dx-VMware-Virtual-Platform:~$ cd /home/dx/CPE631-Navigation-ROS2-main/
dx@dx-VMware-Virtual-Platform:~/CPE631-Navigation-ROS2-main$ ODES=social RUNS_PER_MODE=25 \
EXPERIMENT_TIMEOUT=300 \
CSV_FILE=/home/dx/ros2_ws/A/social_fixed_25.csv \
./codefiles/run_all_experiments.sh
[18:59:45] ✔ 包目录: /home/dx/CPE631-Navigation-ROS2-main
[18:59:45] 同步 launch/param/worlds/models 到 workspace 包 ...
[18:59:45] ✔ 资源文件已同步
[18:59:45] 复制 codefiles/ → cpe631_ros2/ ...
'/home/dx/CPE631-Navigation-ROS2-main/codefiles/ped_pose_extractor.py' -> '/home/dx/CPE631-Navigation-ROS2-main/cpe631_ros2/ped_pose_extractor.py'
'/home/dx/CPE631-Navigation-ROS2-main/codefiles/social_nav_node.py' -> '/home/dx/CPE631-Navigation-ROS2-main/cpe631_ros2/social_nav_node.py'
'/home/dx/CPE631-Navigation-ROS2-main/codefiles/data_collector.py' -> '/home/dx/CPE631-Navigation-ROS2-main/cpe631_ros2/data_collector.py'
'/home/dx/CPE631-Navigation-ROS2-main/codefiles/goal_sender.py' -> '/home/dx/CPE631-Navigation-ROS2-main/cpe631_ros2/goal_sender.py'
'/home/dx/CPE631-Navigation-ROS2-main/codefiles/peds.py' -> '/home/dx/CPE631-Navigation-ROS2-main/cpe631_ros2/peds.py'
[18:59:45] ✔ 改进版代码已复制
[18:59:45] 在 /home/dx/CPE631-Navigation-ROS2-main 执行 colcon build ...
Starting >>> cpe631_ros2
Finished <<< cpe631_ros2 [7.83s]                     

Summary: 1 package finished [8.03s]
[18:59:53] ✔ 编译完成
[18:59:54] ✔ 已 source workspace
[18:59:54] 启动实验前清理残留 ROS/Gazebo/DDS 进程...
[18:59:54] 清理所有后台进程...
The daemon is not running
The daemon has been started
[19:00:07] 清理完成
[19:00:07] ⚠ 已有 CSV 已备份到 /home/dx/ros2_ws/A/social_fixed_25.csv.bak_20260430_190007，本次将新建 /home/dx/ros2_ws/A/social_fixed_25.csv

╔═══════════════════════════════════════════════════════╗
║     ROS2 Social Navigation 三组对比实验 自动脚本 v3  ║
╠═══════════════════════════════════════════════════════╣
║  工作区: /home/dx/CPE631-Navigation-ROS2-main
║  路线:   0.5,-6.5,2.08;-3.5,0.0,0.89;2.5,6.0,-2.75;-2.0,2.1,-1.10;2.0,-4.0,-2.16
║  每组重复: 25 次  MAX_RETRY: 1
║  总计: 200 次实验
╚═══════════════════════════════════════════════════════╝


━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  开始模式: baseline
  navigation=true  social_navigation=false  enable_peds=false  planner=navfn
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
[19:00:07] 启动 cafe_dynamic.launch.py (mode=baseline, planner=navfn) ...
[19:00:07]   launch PID = 2948
[19:00:07] 等待 45s 让仿真完全启动...
[19:00:52] ✔ 仿真已启动
[19:00:57] 验证 Nav2 节点就绪状态...
[19:00:58] ✔ Nav2 bt_navigator 已激活

[19:00:58] ═══ baseline 实验 1/25 ═══
[19:00:58] 尝试 0/1 ...
[19:01:00] 启动 data_collector (mode=baseline, exp=1, attempt=0) ...
[19:01:00]   data_collector PID = 3404
[19:01:02] 启动 goal_sender (sim-timeout=300s, wall-watchdog=0.0s) ...
^C^C[19:02:25] 清理所有后台进程...
^C[19:02:25] 清理所有后台进程...
^C[19:02:26] 清理所有后台进程...
The daemon has been stopped
The daemon has been started
[19:02:39] 清理完成
The daemon has been stopped
^AThe daemon has been started
[19:02:49] 清理完成

dx@dx-VMware-Virtual-Platform:~$ ls -la ~/CPE631-Navigation-ROS2-main/param/nav2_social_dynamic.yaml
ls -la ~/CPE631-Navigation-ROS2-main/param/nav2_social_astar.yaml
-rw-rw-r-- 1 dx dx 13568  5月  1 00:35 /home/dx/CPE631-Navigation-ROS2-main/param/nav2_social_dynamic.yaml
-rw-rw-r-- 1 dx dx 13567  5月  1 00:35 /home/dx/CPE631-Navigation-ROS2-main/param/nav2_social_astar.yaml
dx@dx-VMware-Virtual-Platform:~$ s





x@dx-VMware-Virtual-Platform:~$ # 1. D* start cell fix（C++）
grep "reachable_start" ~/CPE631-Navigation-ROS2-main/src/dstar_planner.cpp

# 2. global costmap 有 social_layer
grep -A2 "plugins:" ~/CPE631-Navigation-ROS2-main/param/nav2_social_dstar.yaml | grep -v dock | grep -v progress | grep -v goal | grep -v controller | grep -v behavior | grep -v planner

# 3. local social_layer 有 subscribe_to_updates 和 trinary_costmap
grep -A15 "social_layer:" ~/CPE631-Navigation-ROS2-main/param/nav2_social_dstar.yaml | head -30

# 4. peak_cost = 55
grep "peak_cost" ~/CPE631-Navigation-ROS2-main/codefiles/social_nav_node.py
  GridNode reachable_start = start_node;
    if (!findReachableGoal(start_node, reachable_start)) {
  const unsigned int start_index = toIndex(reachable_start.x, reachable_start.y);
--
    use_realtime_priority: false

--
      plugins: ["obstacle_layer", "voxel_layer", "social_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
--
      plugins: ["static_layer", "social_layer", "inflation_layer"]
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
--
    GridBased:
      plugin: "cpe631_ros2::DStarPlanner"
--
    spin:
      social_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        enabled: true
        map_topic: /social_costmap
        map_subscribe_transient_local: false
        subscribe_to_updates: true
        trinary_costmap: false
      always_send_full_costmap: true

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: map
      robot_base_frame: base_link
--
      social_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        enabled: true
        map_topic: /social_costmap
        map_subscribe_transient_local: false
        subscribe_to_updates: true
        use_maximum: true
        trinary_costmap: false
        lethal_cost_threshold: 255
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        inflation_radius: 0.5
        cost_scaling_factor: 5.0
        self.declare_parameter("peak_cost", 55)
        self.peak_cost = float(self.get_parameter("peak_cost").value)
                amplitude = self.peak_cost * (self.gamma ** (k + 1))
        amplitude = self.peak_cost * self.instant_cost_ratio
                        self.peak_cost
dx@dx-VMware-Virtual-Platform:~$ 

x@dx-VMware-Virtual-Platform:~$ # 看哪个 goal 卡住
tail -50 /tmp/sender_dstar_social.log

# 看 Nav2 报什么错
grep -E "timeout|Timeout|failed|Failed|ABORT|0 poses|not traversable|Start" /tmp/launch_dstar_social.log | tail -40
tail: 无法以读模式打开 '/tmp/sender_dstar_social.log': 没有那个文件或目录
[lifecycle_manager-7] [INFO] [1777611247.701810028] [lifecycle_manager_localization]: Starting managed nodes bringup...
[lifecycle_manager-18] [INFO] [1777611247.861451240] [lifecycle_manager_navigation]: Starting managed nodes bringup...
[planner_server-10] [WARN] [1777611351.840879674] [planner_server]: GridBased plugin failed to plan from (-0.18, -5.80) to (-3.50, 0.00): "D* planner could not extract a valid path"
[planner_server-10] [WARN] [1777611424.933805366] [planner_server]: GridBased plugin failed to plan from (1.09, 6.66) to (2.50, 6.00): "D* planner could not extract a valid path"
[planner_server-10] [WARN] [1777611462.099991686] [planner_server]: GridBased plugin failed to plan from (2.22, 7.00) to (2.50, 6.00): "D* planner could not extract a valid path"
[planner_server-10] [WARN] [1777611501.513666608] [planner_server]: GridBased plugin failed to plan from (2.34, 7.73) to (2.50, 6.00): "D* planner could not extract a valid path"
[planner_server-10] [WARN] [1777611525.514733855] [planner_server]: GridBased plugin failed to plan from (2.34, 7.73) to (2.50, 6.00): "D* planner could not extract a valid path"
[controller_server-8] [ERROR] [1777611542.583559351] [controller_server]: Failed to make progress
[planner_server-10] [WARN] [1777611553.709538897] [planner_server]: GridBased plugin failed to plan from (2.34, 7.73) to (2.50, 6.00): "D* planner could not extract a valid path"
[amcl-6] [WARN] [1777611613.349800206] [amcl]: Failed to transform initial pose in time (Lookup would require extrapolation into the future.  Requested time 203.486000 but the latest data is at time 203.480000, when looking up transform from frame [base_footprint] to frame [odom])
[planner_server-10] [WARN] [1777611654.620647181] [planner_server]: GridBased plugin failed to plan from (0.47, -3.12) to (0.50, -6.50): "D* planner could not extract a valid path"
[planner_server-10] [WARN] [1777611654.909522941] [planner_server]: GridBased plugin failed to plan from (0.47, -3.17) to (0.50, -6.50): "D* planner could not extract a valid path"
[planner_server-10] [WARN] [1777611682.793802136] [planner_server]: GridBased plugin failed to plan from (-0.75, -4.97) to (-3.50, 0.00): "D* planner could not extract a valid path"
[planner_server-10] [WARN] [1777611684.147934825] [planner_server]: GridBased plugin failed to plan from (-0.85, -4.87) to (-3.50, 0.00): "D* planner could not extract a valid path"
[planner_server-10] [WARN] [1777611722.316908152] [planner_server]: GridBased plugin failed to plan from (-0.57, 2.92) to (2.50, 6.00): "D* planner could not extract a valid path"
[planner_server-10] [WARN] [1777611754.930251109] [planner_server]: GridBased plugin failed to plan from (0.23, 4.79) to (-2.00, 2.10): "D* planner could not extract a valid path"
[controller_server-8] [ERROR] [1777611939.840230382] [controller_server]: Failed to make progress
[controller_server-8] [ERROR] [1777612005.809440965] [controller_server]: Failed to make progress
[controller_server-8] [ERROR] [1777612069.022114404] [controller_server]: Failed to make progress
[bt_navigator-13] [ERROR] [1777612129.736875712] [bt_navigator_navigate_to_pose_rclcpp_node]: Failed to get result for follow_path in node halt!
[amcl-6] [WARN] [1777612133.969743136] [amcl]: Failed to transform initial pose in time (Lookup would require extrapolation into the future.  Requested time 520.170000 but the latest data is at time 520.160000, when looking up transform from frame [base_footprint] to frame [odom])
[planner_server-10] [WARN] [1777612228.790131075] [planner_server]: GridBased plugin failed to plan from (-3.47, 0.04) to (2.50, 6.00): "D* planner could not extract a valid path"
[planner_server-10] [WARN] [1777612245.329157182] [planner_server]: GridBased plugin failed to plan from (-0.28, 3.29) to (2.50, 6.00): "D* planner could not extract a valid path"
[planner_server-10] [WARN] [1777612254.098957433] [planner_server]: GridBased plugin failed to plan from (1.33, 4.64) to (2.50, 6.00): "D* planner could not extract a valid path"
[planner_server-10] [WARN] [1777612283.374188038] [planner_server]: GridBased plugin failed to plan from (2.50, 5.94) to (-2.00, 2.10): "D* planner could not extract a valid path"
[planner_server-10] [WARN] [1777612319.101420284] [planner_server]: GridBased plugin failed to plan from (0.38, -0.93) to (2.00, -4.00): "D* planner could not extract a valid path"
[planner_server-10] [WARN] [1777612393.499025001] [planner_server]: GridBased plugin failed to plan from (-2.92, -1.81) to (-3.50, 0.00): "D* planner could not extract a valid path"
[controller_server-8] [ERROR] [1777612484.451949114] [controller_server]: Failed to make progress
[controller_server-8] [ERROR] [1777612628.790884711] [controller_server]: Failed to make progress
[bt_navigator-13] [ERROR] [1777612653.981728659] [bt_navigator_navigate_to_pose_rclcpp_node]: Failed to get result for follow_path in node halt!
[amcl-6] [WARN] [1777612657.848946281] [amcl]: Failed to transform initial pose in time (Lookup would require extrapolation into the future.  Requested time 839.626000 but the latest data is at time 839.620000, when looking up transform from frame [base_footprint] to frame [odom])
[planner_server-10] [WARN] [1777612687.129846806] [planner_server]: GridBased plugin failed to plan from (1.01, -0.13) to (0.50, -6.50): "D* planner could not extract a valid path"
[planner_server-10] [WARN] [1777612723.874939372] [planner_server]: GridBased plugin failed to plan from (-1.02, -4.18) to (-3.50, 0.00): "D* planner could not extract a valid path"
[planner_server-10] [WARN] [1777612741.674676451] [planner_server]: GridBased plugin failed to plan from (-3.49, 0.02) to (2.50, 6.00): "D* planner could not extract a valid path"
[planner_server-10] [WARN] [1777612776.113579002] [planner_server]: GridBased plugin failed to plan from (1.12, 6.48) to (2.50, 6.00): "D* planner could not extract a valid path"
dx@dx-VMware-Virtual-Platform:~$ 



grep -n "subscribe_to_updates\|trinary_costmap\|plugins" ~/CPE631-Navigation-ROS2-main/param/nav2_social_dynamic.yaml | head -20
grep -n "subscribe_to_updates\|trinary_costmap\|plugins" ~/CPE631-Navigation-ROS2-main/param/nav2_social_dynamic.yaml | head -20
dx@dx-VMware-Virtual-Platform:~/CPE631-Navigation-ROS2-main$ grep -n "subscribe_to_updates\|trinary_costmap\|plugins" ~/CPE631-Navigation-ROS2-main/param/nav2_social_dynamic.yaml | head -20
85:    dock_plugins: ['nova_carter_dock']
111:    progress_checker_plugins: ["progress_checker"]
112:    goal_checker_plugins: ["general_goal_checker"]
113:    controller_plugins: ["FollowPath"]
224:      trinary_costmap: false
228:      plugins: ["obstacle_layer", "voxel_layer", "social_layer", "inflation_layer"]
268:        subscribe_to_updates: true
269:        trinary_costmap: false
283:      trinary_costmap: false
285:      plugins: ["static_layer", "inflation_layer"]
319:    planner_plugins: ["GridBased"]
333:    behavior_plugins: ["spin", "backup", "drive_on_heading", "wait", "assisted_teleop"]
dx@dx-VMware-Virtual-Platform:~/CPE631-Navigation-ROS2-main$ ^C
dx@dx-VMware-Virtual-Platform:~/CPE631-Navigation-ROS2-main$ ^C



dx@dx-VMware-Virtual-Platform:~/CPE631-Navigation-ROS2-main$ grep -n "plugins" ~/CPE631-Navigation-ROS2-main/param/nav2_social_dstar.yaml
85:    dock_plugins: ['nova_carter_dock']
111:    progress_checker_plugins: ["progress_checker"]
112:    goal_checker_plugins: ["general_goal_checker"]
113:    controller_plugins: ["FollowPath"]
228:      plugins: ["obstacle_layer", "voxel_layer", "social_layer", "inflation_layer"]
285:      plugins: ["static_layer", "inflation_layer"]
319:    planner_plugins: ["GridBased"]
340:    behavior_plugins: ["spin", "backup", "drive_on_heading", "wait", "assisted_teleop"]
dx@dx-VMware-Virtual-Platform:~/CPE631-Navigation-ROS2-main$ 



═══════════════════════════════════════════════════════╗
║                   全部实验完成!                      ║
╠═══════════════════════════════════════════════════════╣
║  成功: 20    跳过: 5    总计: 25
║  耗时: 102分43秒
║  CSV 结果: /home/dx/ros2_ws/A/social_fixed_25.csv
║  日志目录: /tmp/launch_*.log, /tmp/collector_*.log, /tmp/sender_*.log
╚═══════════════════════════════════════════════════════╝
[03:05:28] 清理所有后台进程...
The daemon has been stopped
The daemon has been started
[03:05:41] 清理完成
dx@dx-VMware-Virtual-Platform:~/CPE631-Navigation-ROS2-main$ 00
00：未找到命令
dx@dx-VMware-Virtual-Platform:~/CPE631-Navigation-ROS2-main$ 
dx@dx-VMware-Virtual-Platform:~/CPE631-Navigation-ROS2-main$ grep -n "unknown_cost_value\|lethal_cost_threshold\|map_subscribe_transient_local\|cost_weight\|critical_cost" \
  /home/dx/CPE631-Navigation-ROS2-main/param/nav2_social_smac.yaml
158:        cost_weight: 4.0
162:        cost_weight: 5.0
167:        cost_weight: 3.0
172:        cost_weight: 5.0
177:        cost_weight: 3.5
179:        critical_cost: 350.0
187:        cost_weight: 5.0
196:        cost_weight: 5.0
202:        cost_weight: 2.0
225:      lethal_cost_threshold: 100
226:      unknown_cost_value: 255
267:        map_subscribe_transient_local: false
271:        unknown_cost_value: 0
293:        map_subscribe_transient_local: true
299:        map_subscribe_transient_local: true
303:        lethal_cost_threshold: 253
304:        unknown_cost_value: 0
321:    map_subscribe_transient_local: true
dx@dx-VMware-Virtual-Platform:~/CPE631-Navigation-ROS2-main$ 
@dx-VMware-Virtual-Platform:~$ cd /home/dx/CPE631-Navigation-ROS2-main && \
MODES=social_smac RUNS_PER_MODE=25 \
EXPERIMENT_TIMEOUT=300 \
ENABLE_RVIZ=0 GZ_GUI=0 \
CSV_FILE=/home/dx/ros2_ws/A/social_smac_fixed_25.csv \
./codefiles/run_all_experiments.sh
[12:41:28] ✔ 包目录: /home/dx/CPE631-Navigation-ROS2-main
[12:41:28] 同步 launch/param/worlds/models 到 workspace 包 ...
[12:41:28] ✔ 资源文件已同步
[12:41:28] 复制 codefiles/ → cpe631_ros2/ ...
'/home/dx/CPE631-Navigation-ROS2-main/codefiles/ped_pose_extractor.py' -> '/home/dx/CPE631-Navigation-ROS2-main/cpe631_ros2/ped_pose_extractor.py'
'/home/dx/CPE631-Navigation-ROS2-main/codefiles/social_nav_node.py' -> '/home/dx/CPE631-Navigation-ROS2-main/cpe631_ros2/social_nav_node.py'
'/home/dx/CPE631-Navigation-ROS2-main/codefiles/data_collector.py' -> '/home/dx/CPE631-Navigation-ROS2-main/cpe631_ros2/data_collector.py'
'/home/dx/CPE631-Navigation-ROS2-main/codefiles/goal_sender.py' -> '/home/dx/CPE631-Navigation-ROS2-main/cpe631_ros2/goal_sender.py'
'/home/dx/CPE631-Navigation-ROS2-main/codefiles/peds.py' -> '/home/dx/CPE631-Navigation-ROS2-main/cpe631_ros2/peds.py'
[12:41:28] ✔ 改进版代码已复制
[12:41:28] 在 /home/dx/CPE631-Navigation-ROS2-main 执行 colcon build ...
Starting >>> cpe631_ros2
--- stderr: cpe631_ros2                            
gmake[2]: *** 没有规则可制作目标“/opt/ros/jazzy/lib/libfastcdr.so.2.2.5”，由“libcpe631_ros2_dstar_planner.so” 需求。 停止。
gmake[1]: *** [CMakeFiles/Makefile2:220：CMakeFiles/cpe631_ros2_dstar_planner.dir/all] 错误 2
gmake[1]: *** 正在等待未完成的任务....
gmake: *** [Makefile:146：all] 错误 2
---
Failed   <<< cpe631_ros2 [0.48s, exited with code 2]

Summary: 0 packages finished [0.70s]
  1 package failed: cpe631_ros2
  1 package had stderr output: cpe631_ros2
[12:41:29] 清理所有后台进程...
The daemon is not running
The daemon has been started
[12:41:42] 清理完成
dx@dx-VMware-Virtual-Platform:~/CPE631-Navigation-ROS2-main$ 




━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  开始模式: social_smac
  navigation=true  social_navigation=true  enable_peds=true  planner=smac
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
[12:45:18] 启动 cafe_dynamic.launch.py (mode=social_smac, planner=smac) ...
[12:45:18]   launch PID = 5065
[12:45:18] 等待 45s 让仿真完全启动...
[12:46:03] ✔ 仿真已启动
[12:46:08] 验证 Nav2 节点就绪状态...
[12:46:29]   仍在等待 bt_navigator 激活 (15s, 当前状态: inactive)...
[12:46:51]   仍在等待 bt_navigator 激活 (30s, 当前状态: inactive)...
[12:47:12]   仍在等待 bt_navigator 激活 (45s, 当前状态: inactive)...
[12:47:33]   仍在等待 bt_navigator 激活 (60s, 当前状态: inactive)...
[12:47:54]   仍在等待 bt_navigator 激活 (75s, 当前状态: inactive)...
[12:48:16]   仍在等待 bt_navigator 激活 (90s, 当前状态: inactive)...
[12:48:37]   仍在等待 bt_navigator 激活 (105s, 当前状态: inactive)...

x@dx-VMware-Virtual-Platform:~/CPE631-Navigation-ROS2-main$ # 看 launch 日志里 的 ERROR/WARN（现在实时跑的）
tail -f /tmp/launch_social_smac.log | grep -E "ERROR|WARN|error|failed|exception|could not|plugin"
[controller_server-8] [ERROR] [1777654197.975155002] [local_costmap.local_costmap]: StaticLayer: "map" passed to lookupTransform argument target_frame does not exist. 
[gazebo-1] NodeShared::RecvSrvRequest() error sending response: Host unreachable
[controller_server-8] [ERROR] [1777654198.454789855] [local_costmap.local_costmap]: StaticLayer: "map" passed to lookupTransform argument target_frame does not exist. 
[controller_server-8] [ERROR] [1777654199.220679022] [local_costmap.local_costmap]: StaticLayer: "map" passed to lookupTransform argument target_frame does not exist. 
[controller_server-8] [ERROR] [1777654200.085387383] [local_costmap.local_costmap]: StaticLayer: "map" passed to lookupTransform argument target_frame does not exist. 
[controller_server-8] [ERROR] [1777654200.759264898] [local_costmap.local_costmap]: StaticLayer: "map" passed to lookupTransform argument target_frame does not exist. 
[controller_server-8] [ERROR] [1777654201.711198857] [local_costmap.local_costmap]: StaticLayer: "map" passed to lookupTransform argument target_frame does not exist. 
[gazebo-1] NodeShared::RecvSrvRequest() error sending response: Host unreachable
[controller_server-8] [ERROR] [1777654202.138711707] [local_costmap.local_costmap]: StaticLayer: "map" passed to lookupTransform argument target_frame does not exist. 
[controller_server-8] [ERROR] [1777654202.671582545] [local_costmap.local_costmap]: StaticLayer: "map" passed to lookupTransform argument target_frame does not exist. 
[controller_server-8] [ERROR] [1777654203.324191301] [local_costmap.local_costmap]: StaticLayer: "map" passed to lookupTransform argument target_frame does not exist. 
[controller_server-8] [ERROR] [1777654203.968698035] [local_costmap.local_costmap]: StaticLayer: "map" passed to lookupTransform argument target_frame does not exist. 
[controller_server-8] [ERROR] [1777654204.734898454] [local_costmap.local_costmap]: StaticLayer: "map" passed to lookupTransform argument target_frame does not exist. 
[controller_server-8] [ERROR] [1777654205.441062846] [local_costmap.local_costmap]: StaticLayer: "map" passed to lookupTransform argument target_frame does not exist. 
[controller_server-8] [ERROR] [1777654206.082459993] [local_costmap.local_costmap]: StaticLayer: "map" passed to lookupTransform argument target_frame does not exist. 
[gazebo-1] NodeShared::RecvSrvRequest() error sending response: Host unreachable
[controller_server-8] [ERROR] [1777654206.724190815] [local_costmap.local_costmap]: StaticLayer: "map" passed to lookupTransform argument target_frame does not exist. 
[controller_server-8] [ERROR] [1777654207.171842984] [local_costmap.local_costmap]: StaticLayer: "map" passed to lookupTransform argument target_frame does not exist. 
[gazebo-1] NodeShared::RecvSrvRequest() error sending response: Host unreachable
[controller_server-8] [ERROR] [1777654207.611358370] [local_costmap.local_costmap]: StaticLayer: "map" passed to lookupTransform argument target_frame does not exist. 
[controller_server-8] [ERROR] [1777654208.150383910] [local_costmap.local_costmap]: StaticLayer: "map" passed to lookupTransform argument target_frame does not exist. 
[controller_server-8] [ERROR] [1777654208.811981789] [local_costmap.local_costmap]: StaticLayer: "map" passed to lookupTransform argument target_frame does not exist. 
[gazebo-1] NodeShared::RecvSrvRequest() error sending response: Host unreachable
[controller_server-8] [ERROR] [1777654209.331688276] [local_costmap.local_costmap]: StaticLayer: "map" passed to lookupTransform argument target_frame does not exist. 
[controller_server-8] [ERROR] [1777654209.967074422] [local_costmap.local_costmap]: StaticLayer: "map" passed to lookupTransform argument target_frame does not exist. 
[controller_server-8] [ERROR] [1777654210.792478779] [local_costmap.local_costmap]: StaticLayer: "map" passed to lookupTransform argument target_frame does not exist. 
[controller_server-8] [ERROR] [1777654211.489903124] [local_costmap.local_costmap]: StaticLayer: "map" passed to lookupTransform argument target_frame does not exist. 
^C
dx@dx-VMware-Virtual-Platform:~/CPE631-Navigation-ROS2-main$ 

dx@dx-VMware-Virtual-Platform:~/CPE631-Navigation-ROS2-main$ grep -n "amcl\|map_server\|lifecycle_manager" /tmp/launch_social_smac.log | grep -E "ERROR|WARN|active|inactive|failed" | head -30
113:[map_server-5] [ERROR] [1777653920.235340491] [map_io]: Failed processing YAML file /home/dx/CPE631-Navigation-ROS2-main/install/cpe631_ros2/share/cpe631_ros2/maps/cafe.yaml at position (-1:-1) for reason: bad file: /home/dx/CPE631-Navigation-ROS2-main/install/cpe631_ros2/share/cpe631_ros2/maps/cafe.yaml
114:[map_server-5] [ERROR] [1777653920.235397713] [map_server]: Caught exception in callback for transition 10
115:[map_server-5] [ERROR] [1777653920.235410591] [map_server]: Original error: Failed to load map yaml file: /home/dx/CPE631-Navigation-ROS2-main/install/cpe631_ros2/share/cpe631_ros2/maps/cafe.yaml
116:[map_server-5] [WARN] [1777653920.235426829] [map_server]: Callback returned ERROR during the transition: configure
118:[lifecycle_manager-7] [ERROR] [1777653920.237769780] [lifecycle_manager_localization]: Failed to change state for node: map_server
119:[lifecycle_manager-7] [ERROR] [1777653920.237804384] [lifecycle_manager_localization]: Failed to bring up all requested nodes. Aborting bringup.
622:[lifecycle_manager-18] [ERROR] [1777654007.347735581] [lifecycle_manager_navigation]: Failed to change state for node: planner_server
623:[lifecycle_manager-18] [ERROR] [1777654007.347764554] [lifecycle_manager_navigation]: Failed to bring up all requested nodes. Aborting bringup.
dx@dx-VMware-Virtual-Platform:~/CPE631-Navigation-ROS2-main$ 
dx@dx-VMware-Virtual-Platform:~/CPE631-Navigation-ROS2-main$ grep -n "lifecycle_manager\|Activating\|activated\|failed to activate" /tmp/launch_social_smac.log | head -30
10:[INFO] [lifecycle_manager-7]: process started with pid [5078]
21:[INFO] [lifecycle_manager-18]: process started with pid [5101]
65:[lifecycle_manager-7] [INFO] [1777653920.012629434] [lifecycle_manager_localization]: Creating
72:[lifecycle_manager-18] [INFO] [1777653920.020076000] [lifecycle_manager_navigation]: Creating
73:[lifecycle_manager-7] [INFO] [1777653920.021666372] [lifecycle_manager_localization]: Creating and initializing lifecycle service clients
74:[lifecycle_manager-7] [INFO] [1777653920.045008539] [lifecycle_manager_localization]: Starting managed nodes bringup...
75:[lifecycle_manager-7] [INFO] [1777653920.045063442] [lifecycle_manager_localization]: Configuring map_server
76:[lifecycle_manager-18] [INFO] [1777653920.051572345] [lifecycle_manager_navigation]: Creating and initializing lifecycle service clients
102:[lifecycle_manager-18] [INFO] [1777653920.219068981] [lifecycle_manager_navigation]: Starting managed nodes bringup...
103:[lifecycle_manager-18] [INFO] [1777653920.219137892] [lifecycle_manager_navigation]: Configuring controller_server
118:[lifecycle_manager-7] [ERROR] [1777653920.237769780] [lifecycle_manager_localization]: Failed to change state for node: map_server
119:[lifecycle_manager-7] [ERROR] [1777653920.237804384] [lifecycle_manager_localization]: Failed to bring up all requested nodes. Aborting bringup.
154:[lifecycle_manager-18] [INFO] [1777653920.364117834] [lifecycle_manager_navigation]: Configuring smoother_server
158:[lifecycle_manager-18] [INFO] [1777653920.403157936] [lifecycle_manager_navigation]: Configuring planner_server
215:[lifecycle_manager-18] [INFO] [1777653921.489933787] [lifecycle_manager_navigation]: Configuring route_server
228:[lifecycle_manager-18] [INFO] [1777653921.521836138] [lifecycle_manager_navigation]: Configuring behavior_server
240:[lifecycle_manager-18] [INFO] [1777653921.557461411] [lifecycle_manager_navigation]: Configuring velocity_smoother
242:[lifecycle_manager-18] [INFO] [1777653921.562465645] [lifecycle_manager_navigation]: Configuring collision_monitor
251:[lifecycle_manager-18] [INFO] [1777653921.584669743] [lifecycle_manager_navigation]: Configuring bt_navigator
257:[lifecycle_manager-18] [INFO] [1777653921.668733725] [lifecycle_manager_navigation]: Configuring waypoint_follower
260:[lifecycle_manager-18] [INFO] [1777653921.686314504] [lifecycle_manager_navigation]: Configuring docking_server
266:[lifecycle_manager-18] [INFO] [1777653921.719575929] [lifecycle_manager_navigation]: Activating controller_server
267:[controller_server-8] [INFO] [1777653921.719759874] [controller_server]: Activating
268:[controller_server-8] [INFO] [1777653921.719783576] [local_costmap.local_costmap]: Activating
324:[lifecycle_manager-18] [INFO] [1777653924.741251837] [lifecycle_manager_navigation]: Server controller_server connected with bond.
325:[lifecycle_manager-18] [INFO] [1777653924.741279542] [lifecycle_manager_navigation]: Activating smoother_server
326:[smoother_server-9] [INFO] [1777653924.741506017] [smoother_server]: Activating
328:[lifecycle_manager-18] [INFO] [1777653924.846030768] [lifecycle_manager_navigation]: Server smoother_server connected with bond.
329:[lifecycle_manager-18] [INFO] [1777653924.846055555] [lifecycle_manager_navigation]: Activating planner_server
330:[planner_server-10] [INFO] [1777653924.846214728] [planner_server]: Activating
dx@dx-VMware-Virtual-Platform:~/CPE631-Navigation-ROS2-main$ 


experiment,mode,result,attempt,path_length_m,time_s,encounters,personal_violations,intimate_violations,min_ped_dist_m,mean_angular_acceleration
1,social_smac,succeeded,0,37.826,103.77,4,15,2,0.089,0.3862
2,social_smac,succeeded,0,34.355,104.82,4,6,2,0.26,0.3572
3,social_smac,aborted,0,29.937,94.91,7,7,3,0.243,0.3593
3,social_smac,aborted,1,37.777,128.84,9,16,5,0.222,0.455
4,social_smac,aborted,0,0.044,3.28,0,0,0,1.405,1.0581
4,social_smac,succeeded,1,37.703,102.58,9,11,3,0.116,0.3708
5,social_smac,succeeded,0,38.318,112.25,10,13,3,0.299,0.353
6,social_smac,aborted,0,3.855,18.57,1,3,1,0.396,0.3588
6,social_smac,succeeded,1,37.666,95.06,10,15,5,0.086,0.3791
7,social_smac,succeeded,0,33.951,93.13,4,4,1,0.327,0.3068
8,social_smac,succeeded,0,34.765,102.75,4,8,4,0.056,0.3686
9,social_smac,aborted,0,20.356,78.9,4,4,1,0.217,0.4021
9,social_smac,succeeded,1,40.959,141.47,11,14,2,0.162,0.3585
10,social_smac,aborted,0,4.409,13.23,1,1,0,0.995,0.4617
10,social_smac,succeeded,1,37.902,104.86,8,9,3,0.105,0.3135
11,social_smac,aborted,0,11.806,29.92,2,4,2,0.266,0.4031
11,social_smac,succeeded,1,39.264,111.34,12,11,3,0.156,0.329
12,social_smac,aborted,0,35.538,103.4,7,8,4,0.074,0.3537
12,social_smac,succeeded,1,41.021,173.59,10,13,3,0.378,0.3528
13,social_smac,timeout,0,12.765,210.06,35,29,3,0.17,0.1568
14,social_smac,succeeded,0,37.415,105.31,6,12,3,0.172,0.3248
15,social_smac,succeeded,0,35.884,104.14,8,10,3,0.024,0.3155
16,social_smac,timeout,0,3.242,210.1,0,2,0,1.174,0.1328
17,social_smac,timeout,0,48.938,206.64,21,23,9,0.154,0.2683
18,social_smac,succeeded,0,44.854,124.19,6,10,8,0.107,0.3147
19,social_smac,aborted,0,14.313,42.66,1,6,1,0.192,0.4389
19,social_smac,aborted,1,42.085,123.29,11,12,7,0.1,0.339
20,social_smac,timeout,0,39.788,221.05,34,35,7,0.025,0.1859
21,social_smac,succeeded,0,39.965,137.23,6,8,0,0.633,0.3486
22,social_smac,aborted,0,7.009,36.44,2,2,1,0.199,0.2981
22,social_smac,succeeded,1,43.477,134.07,8,11,5,0.102,0.2709
23,social_smac,aborted,0,22.572,275.15,11,9,7,0.057,0.1427
23,social_smac,timeout,1,45.035,300.9,33,38,16,0.032,0.1642
24,social_smac,aborted,0,13.388,38.37,1,1,1,0.267,0.3072
24,social_smac,succeeded,1,41.239,120.94,7,9,4,0.087,0.2381
25,social_smac,succeeded,0,33.392,206.68,15,19,8,0.097,0.1467


dx@dx-VMware-Virtual-Platform:~/CPE631-Navigation-ROS2-main$ grep -n "analytic_expansion_max_cost\|cost_penalty\|w_smooth\|PreferForwardCritic\|cost_weight\|max_path_occupancy_ratio\|PathAngleCritic" \
  /home/dx/CPE631-Navigation-ROS2-main/param/nav2_social_smac.yaml | grep -v "enabled\|cost_power\|offset\|threshold\|trajectory\|mode\|angle_to"
154:        "PathAngleCritic", "PreferForwardCritic"]
158:        cost_weight: 4.0
162:        cost_weight: 5.0
167:        cost_weight: 3.0
169:      PreferForwardCritic:
172:        cost_weight: 7.0
177:        cost_weight: 3.5
187:        cost_weight: 7.0
188:        max_path_occupancy_ratio: 0.03
196:        cost_weight: 5.0
199:      PathAngleCritic:
202:        cost_weight: 3.0
348:      analytic_expansion_max_cost: 230.0
349:      analytic_expansion_max_cost_override: false
354:      cost_penalty: 2.0
359:      use_quadratic_cost_penalty: false
365:        w_smooth: 0.40
dx@dx-VMware-Virtual-Platform:~/CPE631-Navigation-ROS2-main$ 
cd /home/dx/CPE631-Navigation-ROS2-main
tar -czf ../CPE631-backup-$(date +%F).tgz .
git init
git add .
git commit -m "sync: snapshot from linux"
git remote add origin git@github.com:ZSY007/CPE631-Navigation-ROS2.git
git branch -M main
git push -u origin main



x@dx-VMware-Virtual-Platform:~/CPE631-Navigation-ROS2-main$ cd /home/dx/CPE631-Navigation-ROS2-main
tar -czf ../CPE631-backup-$(date +%F).tgz .
git init
git add .
git commit -m "sync: snapshot from linux"
git remote add origin git@github.com:ZSY007/CPE631-Navigation-ROS2.git
git branch -M main
git push -u origin main
提示：使用 'master' 作为初始分支的名称。这个默认分支名称可能会更改。要在新仓库中
提示：配置使用初始分支名，并消除这条警告，请执行：
提示：
提示：	git config --global init.defaultBranch <名称>
提示：
提示：除了 'master' 之外，通常选定的名字有 'main'、'trunk' 和 'development'。
提示：可以通过以下命令重命名刚创建的分支：
提示：
提示：	git branch -m <name>
已初始化空的 Git 仓库于 /home/dx/CPE631-Navigation-ROS2-main/.git/
作者身份未知

*** 请告诉我您是谁。

运行

  git config --global user.email "you@example.com"
  git config --global user.name "Your Name"

来设置您账号的缺省身份标识。
如果仅在本仓库设置身份标识，则省略 --global 参数。

fatal: 无法自动探测邮件地址（得到 'dx@dx-VMware-Virtual-Platform.(none)'）
error: 源引用规格 main 没有匹配
error: 无法推送一些引用到 'github.com:ZSY007/CPE631-Navigation-ROS2.git'
dx@dx-VMware-Virtual-Platform:~/CPE631-Navigation-ROS2-main$ ^C
dx@dx-VMware-Virtual-Platform:~/CPE631-Navigation-ROS2-main$ 


