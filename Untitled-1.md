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



