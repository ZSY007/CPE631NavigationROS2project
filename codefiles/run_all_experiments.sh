#!/bin/bash
###############################################################################
#  run_all_experiments.sh  v3
#  ─────────────────────────
#  One-command run all three comparison experiments (Baseline / Dynamic / Social)
#
#  Architecture:
#    Each mode starts Gazebo + Nav2 once, 5 experiments share the same simulator.
#    Single abort → reset robot pose and retry (up to MAX_RETRY times).
#    Only restart system when Gazebo/Nav2 crashes.
#
#  Usage:
#    chmod +x run_all_experiments.sh
#    ./run_all_experiments.sh
#
#  Optional environment variables:
#    ROUTE               Custom route (semicolon-separated x,y,yaw)
#    DWELL_TIME          Dwell time at each goal point (seconds)
#    STARTUP_WAIT        Launch startup wait time (seconds), also pedestrian phase warm-up
#    COOLDOWN            Cooldown time between modes (seconds)
#    RUN_COOLDOWN        Cooldown time between experiments (seconds)
#    SETTLE_WAIT         Wait time for data_collector to write CSV (seconds)
#    RUNS_PER_MODE       Runs per mode (number of waypoint groups)
#    MODES               Run only specified modes, comma-separated, e.g. social,social_smac
#    RESET_PEDS_EACH_RUN Reset pedestrian positions before each run (default 1)
#    MAX_RETRY           Max retry attempts after abort (default 2)
#    ENABLE_RVIZ         Enable RViz for batch experiments (default 1)
#    GZ_GUI              Enable Gazebo GUI for batch experiments (default 1)
#    SKIP_BUILD          Set to 1 to skip build step
#    EXPERIMENT_TIMEOUT  Max simulation time per experiment (seconds, default 720; 0 = disabled)
#    EXPERIMENT_WALL_TIMEOUT  Real-time watchdog per experiment (seconds, default 0; 0 = disabled)
#    NAV2_READY_TIMEOUT  Wait timeout for bt_navigator activation (seconds, default 180)
#    APPEND_CSV          Set to 1 to append to existing CSV; default backs up old CSV and creates new
###############################################################################

set -euo pipefail

# ── Path configuration ──────────────────────────────────────────────────
# By default use the package directory where this script is located as colcon workspace root.
# This way after modifying code in current directory, re-running script compiles and uses current directory's code.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC_PKG_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
WS_ROOT="${WS_ROOT:-${SRC_PKG_DIR}}"
PKG_DIR="${PKG_DIR:-${SRC_PKG_DIR}}"
SETUP_BASH="${SETUP_BASH:-${WS_ROOT}/install/setup.bash}"
LOG_DIR="${LOG_DIR:-${WS_ROOT}/logs}"
mkdir -p "${LOG_DIR}"

# ── Experiment configuration ──────────────────────────────────────────────────
# Route: zigzag coverage, wp4 in mid-left lower open area, avoid cutting to left wall/narrow mouth from above
ROUTE="${ROUTE:-0.5,-6.5,2.08;-3.5,0.0,0.89;2.5,6.0,-2.75;-2.0,2.1,-1.10;2.0,-4.0,-2.16}"
DWELL_TIME="${DWELL_TIME:-0.5}"
STARTUP_WAIT="${STARTUP_WAIT:-45}"
COOLDOWN="${COOLDOWN:-15}"
RUN_COOLDOWN="${RUN_COOLDOWN:-5}"
SETTLE_WAIT="${SETTLE_WAIT:-30}"
RUNS_PER_MODE="${RUNS_PER_MODE:-5}"
MODES="${MODES:-}"
RESET_PEDS_EACH_RUN="${RESET_PEDS_EACH_RUN:-1}"
MAX_RETRY="${MAX_RETRY:-2}"
ENABLE_RVIZ="${ENABLE_RVIZ:-1}"
GZ_GUI="${GZ_GUI:-1}"
SKIP_BUILD="${SKIP_BUILD:-0}"
EXPERIMENT_TIMEOUT="${EXPERIMENT_TIMEOUT:-450}"
EXPERIMENT_WALL_TIMEOUT="${EXPERIMENT_WALL_TIMEOUT:-0}"
EXPERIMENT_WALL_TIMEOUT_PARAM="${EXPERIMENT_WALL_TIMEOUT}"
if [[ "${EXPERIMENT_WALL_TIMEOUT_PARAM}" =~ ^[0-9]+$ ]]; then
    EXPERIMENT_WALL_TIMEOUT_PARAM="${EXPERIMENT_WALL_TIMEOUT_PARAM}.0"
fi
NAV2_READY_TIMEOUT="${NAV2_READY_TIMEOUT:-180}"
if [[ -z "${CSV_FILE:-}" ]]; then
    if [[ "${MODES//[[:space:]]/}" == "social_smac" ]]; then
        CSV_FILE="${WS_ROOT}/social_smac_${RUNS_PER_MODE}.csv"
    elif [[ "${MODES//[[:space:]]/}" == "social_dstar" ]]; then
        CSV_FILE="${WS_ROOT}/social_dstar_${RUNS_PER_MODE}.csv"
    else
        CSV_FILE="${WS_ROOT}/experiment_results_v3.csv"
    fi
fi
EXPERIMENT_GOAL_TOPIC="${EXPERIMENT_GOAL_TOPIC:-/experiment_goal_pose}"
APPEND_CSV="${APPEND_CSV:-0}"

# data_collector parameters
ENCOUNTER_THRESHOLD="1.0"
PERSONAL_ZONE="1.2"
INTIMATE_ZONE="0.45"

# Robot initial pose (consistent with spawn pose in cafe.world)
ROBOT_MODEL_NAME="turtlebot3_burger"
ROBOT_INIT_X="1.0"
ROBOT_INIT_Y="0.0"
ROBOT_INIT_Z="0.05"
ROBOT_INIT_YAW="0.0"

# ── Color output ──────────────────────────────────────────────────
RED='\033[0;31m'
GREEN='\033[0;32m'
CYAN='\033[0;36m'
YELLOW='\033[1;33m'
BOLD='\033[1m'
NC='\033[0m'

log()  { echo -e "${CYAN}[$(date +%H:%M:%S)]${NC} $*"; }
ok()   { echo -e "${GREEN}[$(date +%H:%M:%S)] ✔ $*${NC}"; }
warn() { echo -e "${YELLOW}[$(date +%H:%M:%S)] ⚠ $*${NC}"; }
err()  { echo -e "${RED}[$(date +%H:%M:%S)] ✘ $*${NC}"; }

# ── 清理函数 ──────────────────────────────────────────────────
LAUNCH_PID=""

kill_node() {
    # 杀掉单个节点进程 (不杀 launch / Gazebo / Nav2)
    local name="$1"
    pkill -f "$name" 2>/dev/null || true
    sleep 0.5
    pkill -9 -f "$name" 2>/dev/null || true
}

cleanup_full() {
    log "清理所有后台进程..."

    # 1) SIGINT launch
    if [[ -n "$LAUNCH_PID" ]] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
        kill -INT "$LAUNCH_PID" 2>/dev/null || true
    fi
    sleep 3

    # 2) 强杀所有相关进程
    for pat in \
        "cafe_dynamic.launch" \
        "data_collector" "goal_sender" \
        "social_nav_node" "ped_pose_extractor" "cpe631_peds" "cpe631_map_republisher" \
        "gz sim" "gz_server" "ruby.*gz" \
        "rviz2" \
        "nav2" "bt_navigator" "controller_server" "planner_server" \
        "smoother_server" "behavior_server" "amcl" "lifecycle_manager" \
        "map_server" "velocity_smoother" "collision_monitor" \
        "parameter_bridge" "robot_state_publisher" \
        "ros2.launch" "ros2.run"; do
        pkill -f "$pat" 2>/dev/null || true
    done
    sleep 3

    for pat in \
        "gz sim" "gz_server" "ruby.*gz" \
        "nav2" "bt_navigator" "controller_server" "planner_server" \
        "smoother_server" "behavior_server" "amcl" "lifecycle_manager" \
        "map_server" "velocity_smoother" "collision_monitor" "rviz2" \
        "parameter_bridge" "robot_state_publisher" \
        "data_collector" "goal_sender" \
        "social_nav_node" "ped_pose_extractor" "cpe631_map_republisher"; do
        pkill -9 -f "$pat" 2>/dev/null || true
    done
    sleep 3

    LAUNCH_PID=""

    # 3) 清理 DDS 共享内存
    rm -f /dev/shm/fastrtps_* /dev/shm/cyclonedds_* 2>/dev/null || true
    rm -f /tmp/cyclonedds_* 2>/dev/null || true

    # 4) 重启 ros2 daemon
    timeout 5 ros2 daemon stop 2>/dev/null || true
    sleep 1
    timeout 5 ros2 daemon start 2>/dev/null || true
    sleep 1

    log "清理完成"
}

trap cleanup_full EXIT INT TERM

wait_lifecycle_active() {
    local node_name="$1"
    local timeout_s="${2:-20}"
    local elapsed=0
    local state=""

    while [[ $elapsed -lt $timeout_s ]]; do
        state=$(timeout 5 ros2 lifecycle get "$node_name" 2>/dev/null \
            | grep -oE '(active|inactive|unconfigured|finalized)' \
            | head -1) || true
        if [[ "$state" == "active" ]]; then
            return 0
        fi
        sleep 2
        elapsed=$((elapsed + 2))
    done

    warn "${node_name} 未在 ${timeout_s}s 内进入 active 状态 (当前: ${state:-未知})"
    return 1
}

# ── 取消 Nav2 残留 action ────────────────────────────────────
cancel_action_goals() {
    local action_name="$1"
    local cancel_req
    cancel_req="{goal_info: {stamp: {sec: 0, nanosec: 0}, goal_id: {uuid: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}}}"
    timeout 3 ros2 service call "${action_name}/_action/cancel_goal" \
        action_msgs/srv/CancelGoal "${cancel_req}" >/dev/null 2>&1 || true
}

cancel_nav2_goals() {
    log "取消残留 Nav2 action..."
    cancel_action_goals "/navigate_to_pose"
    cancel_action_goals "/compute_path_to_pose"
    cancel_action_goals "/follow_path"
}

stop_robot_motion() {
    local zero_twist
    zero_twist="{header: {frame_id: 'base_link'}, twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"
    timeout 2 ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped "${zero_twist}" >/dev/null 2>&1 || true
    timeout 2 ros2 topic pub --once /cmd_vel_smoothed geometry_msgs/msg/TwistStamped "${zero_twist}" >/dev/null 2>&1 || true
}

# ── 复位机器人位姿 ────────────────────────────────────────────
reset_robot_pose() {
    log "重置机器人位姿 → (${ROBOT_INIT_X}, ${ROBOT_INIT_Y}, yaw=${ROBOT_INIT_YAW})"

    # 先取消可能残留的 Nav2 action。上一轮 abort/timeout 后，BT 或
    # controller action 有时还会继续推 cmd_vel，导致 Gazebo set_pose 后
    # 机器人又被旧目标拖走。
    cancel_nav2_goals
    stop_robot_motion
    sleep 1

    # AMCL 不 active 时发布 /initialpose 会被忽略，长跑后会导致 Nav2 报
    # "Initial robot pose is not available"。
    wait_lifecycle_active "/amcl" 20 || true

    # 1) Gazebo: 物理位置重置。优先使用 set_pose_vector，和 peds.py 的
    #    行人复位路径保持一致；固定 GZ_IP 可避免 GUI/VM 环境下偶发
    #    "Host unreachable" 导致请求没打到 Gazebo。
    local pose_vector_req
    pose_vector_req="pose { name: \"${ROBOT_MODEL_NAME}\" position { x: ${ROBOT_INIT_X} y: ${ROBOT_INIT_Y} z: ${ROBOT_INIT_Z} } orientation { x: 0 y: 0 z: 0 w: 1 } }"
    if ! GZ_IP="${GZ_IP:-127.0.0.1}" timeout 8 gz service -s /world/cafe_world/set_pose_vector \
        --reqtype gz.msgs.Pose_V \
        --reptype gz.msgs.Boolean \
        --timeout 3000 \
        --req "${pose_vector_req}" \
        >"${LOG_DIR}/reset_robot_pose_gz.log" 2>&1; then
        warn "Gazebo set_pose_vector 失败，回退到 set_pose (查看 ${LOG_DIR}/reset_robot_pose_gz.log)"
        GZ_IP="${GZ_IP:-127.0.0.1}" timeout 5 gz service -s /world/cafe_world/set_pose \
            --reqtype gz.msgs.Pose \
            --reptype gz.msgs.Boolean \
            --timeout 3000 \
            --req "name: '${ROBOT_MODEL_NAME}', position: {x: ${ROBOT_INIT_X}, y: ${ROBOT_INIT_Y}, z: ${ROBOT_INIT_Z}}, orientation: {x: 0, y: 0, z: 0, w: 1}" \
            >"${LOG_DIR}/reset_robot_pose_gz_fallback.log" 2>&1 || \
            warn "Gazebo set_pose 失败 (非致命，查看 ${LOG_DIR}/reset_robot_pose_gz_fallback.log)"
    fi

    # 2) AMCL: 发布 /initialpose 让定位重置
    ros2 topic pub --once /initialpose \
        geometry_msgs/msg/PoseWithCovarianceStamped \
        "{header: {frame_id: 'map'}, pose: {pose: {position: {x: ${ROBOT_INIT_X}, y: ${ROBOT_INIT_Y}, z: 0.0}, orientation: {z: 0.0, w: 1.0}}}}" \
        --use-sim-time \
        2>/dev/null || warn "initialpose 发布失败 (非致命)"

    # 3) 清除 Nav2 costmap (避免残留障碍物)
    timeout 3 ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap "{}" 2>/dev/null || true
    timeout 3 ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap "{}" 2>/dev/null || true

    # 4) 等待 Nav2 稳定（costmap 重建 + TF 更新需要时间）
    sleep 5
    stop_robot_motion
    wait_lifecycle_active "/bt_navigator" 20 || true
    ok "机器人位姿已重置"
}

cancel_action_goals() {
    local action_name="$1"
    local cancel_req
    cancel_req="{goal_info: {stamp: {sec: 0, nanosec: 0}, goal_id: {uuid: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}}}"
    timeout 3 ros2 service call "${action_name}/_action/cancel_goal" \
        action_msgs/srv/CancelGoal "${cancel_req}" >/dev/null 2>&1 || true
}

cancel_nav2_goals() {
    log "取消残留 Nav2 action..."
    cancel_action_goals "/navigate_to_pose"
    cancel_action_goals "/compute_path_to_pose"
    cancel_action_goals "/follow_path"
}

stop_robot_motion() {
    local zero_twist
    zero_twist="{header: {frame_id: 'base_link'}, twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"
    timeout 2 ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped "${zero_twist}" >/dev/null 2>&1 || true
    timeout 2 ros2 topic pub --once /cmd_vel_smoothed geometry_msgs/msg/TwistStamped "${zero_twist}" >/dev/null 2>&1 || true
}

reset_pedestrians() {
    local enable_peds="$1"
    if [[ "${RESET_PEDS_EACH_RUN}" != "1" ]] || [[ "${enable_peds}" != "true" ]]; then
        return 0
    fi

    log "重置行人到初始位置..."
    if timeout 8 ros2 service call /reset_pedestrians std_srvs/srv/Trigger "{}" >"${LOG_DIR}/reset_pedestrians.log" 2>&1; then
        ok "行人已重置"
        sleep 2
        return 0
    fi

    warn "行人重置服务调用失败 (非致命)，查看 ${LOG_DIR}/reset_pedestrians.log"
    return 1
}

# ── 系统健康检查 ──────────────────────────────────────────────
check_system_health() {
    # 返回 0 = 健康, 1 = 需要重启
    if [[ -z "$LAUNCH_PID" ]] || ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
        err "launch 进程已退出"
        return 1
    fi

    local bt_state
    bt_state=$(timeout 5 ros2 lifecycle get /bt_navigator 2>/dev/null \
        | grep -oE '(active|inactive|unconfigured|finalized)' \
        | head -1) || true

    if [[ "$bt_state" != "active" ]]; then
        err "bt_navigator 状态异常: ${bt_state:-未知}"
        return 1
    fi

    return 0
}

# ── 启动仿真 ──────────────────────────────────────────────────
start_simulation() {
    local mode="$1"
    local navigation="$2"
    local social_nav="$3"
    local enable_peds="$4"
    local planner_variant="${5:-navfn}"

    log "启动 cafe_dynamic.launch.py (mode=${mode}, planner=${planner_variant}) ..."
    ros2 launch cpe631_ros2 cafe_dynamic.launch.py \
        navigation:="${navigation}" \
        social_navigation:="${social_nav}" \
        enable_peds:="${enable_peds}" \
        planner_variant:="${planner_variant}" \
        enable_rviz:="${ENABLE_RVIZ}" \
        gz_gui:="${GZ_GUI}" \
        > "${LOG_DIR}/launch_${mode}.log" 2>&1 &
    LAUNCH_PID=$!
    log "  launch PID = ${LAUNCH_PID}"

    log "等待 ${STARTUP_WAIT}s 让仿真完全启动..."
    sleep "${STARTUP_WAIT}"

    if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
        err "launch 已退出"
        warn "查看日志: cat ${LOG_DIR}/launch_${mode}.log"
        return 1
    fi
    ok "仿真已启动"

    # 短暂等待 bridge 建立 TF
    sleep 5

    # 验证 Nav2 就绪
    log "验证 Nav2 节点就绪状态..."
    local nav_ready=false
    local nav_check_elapsed=0
    local nav_check_timeout="${NAV2_READY_TIMEOUT}"
    while [[ $nav_check_elapsed -lt $nav_check_timeout ]]; do
        if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
            err "launch 在等待 Nav2 就绪时退出"
            return 1
        fi
        local bt_state
        bt_state=$(timeout 5 ros2 lifecycle get /bt_navigator 2>/dev/null \
            | grep -oE '(active|inactive|unconfigured|finalized)' \
            | head -1) || true
        if [[ "$bt_state" == "active" ]]; then
            nav_ready=true
            break
        fi
        sleep 3
        nav_check_elapsed=$((nav_check_elapsed + 3))
        [[ $((nav_check_elapsed % 15)) -eq 0 ]] && \
            log "  仍在等待 bt_navigator 激活 (${nav_check_elapsed}s, 当前状态: ${bt_state:-未知})..."
    done

    if ! $nav_ready; then
        err "bt_navigator 未在 ${nav_check_timeout}s 内就绪"
        return 1
    fi
    ok "Nav2 bt_navigator 已激活"
    return 0
}

# ── 运行单次实验 ──────────────────────────────────────────────
run_single_experiment() {
    local mode="$1"
    local experiment_id="$2"
    local attempt="$3"

    local collector_log="${LOG_DIR}/collector_${mode}_${experiment_id}_${attempt}.log"
    local sender_log="${LOG_DIR}/sender_${mode}_${experiment_id}_${attempt}.log"
    local result_file="${LOG_DIR}/result_${mode}_${experiment_id}_${attempt}.txt"

    # 清理上次残留的 result 文件
    rm -f "${result_file}"

    # 1) 启动 data_collector (每个 run 独立进程)
    #    auto_stop=false: 不再监听 action status 自动停止
    #    result_file: SIGUSR1 handler 从此文件读取最终 result
    log "启动 data_collector (mode=${mode}, exp=${experiment_id}, attempt=${attempt}) ..."
    ros2 run cpe631_ros2 data_collector --ros-args \
        -p use_sim_time:=true \
        -p mode:="${mode}" \
        -p csv_file:="${CSV_FILE}" \
        -p encounter_threshold:="${ENCOUNTER_THRESHOLD}" \
        -p personal_zone:="${PERSONAL_ZONE}" \
        -p intimate_zone:="${INTIMATE_ZONE}" \
        -p goal_topic:="${EXPERIMENT_GOAL_TOPIC}" \
        -p timestamp_filename:=false \
        -p auto_start:=true \
        -p auto_stop:=false \
        -p experiment_id:="${experiment_id}" \
        -p attempt:="${attempt}" \
        -p exit_after_result:=true \
        -p result_file:="${result_file}" \
        > "${collector_log}" 2>&1 &
    local collector_pid=$!
    log "  data_collector PID = ${collector_pid}"

    sleep 2

    # 2) 启动 goal_sender (阻塞等待它退出)
    if [[ "${EXPERIMENT_TIMEOUT}" == "0" ]]; then
        log "启动 goal_sender (sim-timeout=disabled, wall-watchdog=${EXPERIMENT_WALL_TIMEOUT_PARAM}s) ..."
    else
        log "启动 goal_sender (sim-timeout=${EXPERIMENT_TIMEOUT}s, wall-watchdog=${EXPERIMENT_WALL_TIMEOUT_PARAM}s) ..."
    fi
    local sender_exit=0
    ros2 run cpe631_ros2 goal_sender --ros-args \
        -p use_sim_time:=true \
        -p goal_topic:="${EXPERIMENT_GOAL_TOPIC}" \
        -p dwell_time:="${DWELL_TIME}" \
        -p route:="${ROUTE}" \
        -p experiment_timeout:="${EXPERIMENT_TIMEOUT}" \
        -p experiment_wall_timeout:="${EXPERIMENT_WALL_TIMEOUT_PARAM}" \
        > "${sender_log}" 2>&1 || sender_exit=$?
    log "  goal_sender 退出码 = ${sender_exit}"

    # 3) 翻译退出码为 result 字符串，写入 result_file
    local result_str="unknown"
    case $sender_exit in
        0) result_str="succeeded" ;;
        2) result_str="aborted" ;;
        3) result_str="canceled" ;;
        4) result_str="timeout" ;;
        *) result_str="error" ;;
    esac
    echo "${result_str}" > "${result_file}"
    log "  result = ${result_str} (写入 ${result_file})"

    # 4) 发送 SIGUSR1 通知 data_collector 保存数据并退出
    #    注意: ros2 run 是 wrapper 脚本，kill -USR1 $pid 只发给 wrapper
    #    需要用 pkill 直接找到 Python data_collector 进程
    log "发送 SIGUSR1 给 data_collector ..."
    pkill -USR1 -f "result_file:=${result_file}" 2>/dev/null || \
        kill -USR1 "$collector_pid" 2>/dev/null || true

    # 5) 等待 data_collector 自动退出 (exit_after_result=true)
    log "等待 data_collector 写入 CSV (最多 ${SETTLE_WAIT}s) ..."
    local settle_elapsed=0
    while [[ $settle_elapsed -lt $SETTLE_WAIT ]]; do
        if ! kill -0 "$collector_pid" 2>/dev/null; then
            ok "data_collector 已退出"
            break
        fi
        if grep -q "Saved to" "${collector_log}" 2>/dev/null; then
            ok "数据已落盘"
            sleep 2
            break
        fi
        sleep 1
        settle_elapsed=$((settle_elapsed + 1))
    done

    # 6) 如果 data_collector 还活着，强杀
    if kill -0 "$collector_pid" 2>/dev/null; then
        warn "data_collector 仍在运行，发送 SIGINT..."
        kill -INT "$collector_pid" 2>/dev/null || true
        sleep 3
        if kill -0 "$collector_pid" 2>/dev/null; then
            kill -9 "$collector_pid" 2>/dev/null || true
        fi
    fi

    # 7) 清理
    kill_node "goal_sender"
    pkill -f "result_file:=${result_file}" 2>/dev/null || true
    rm -f "${result_file}"

    return $sender_exit
}


# ── Step 0: 检查包目录 ───────────────────────────────────────
if [[ ! -f "${PKG_DIR}/package.xml" ]]; then
    err "找不到 ${PKG_DIR}/package.xml"
    exit 1
fi
ok "包目录: ${PKG_DIR}"

# ── Step 1: 同步源仓库改动并复制改进版代码 ────────────────────
sync_dir() {
    local rel_dir="$1"
    if [[ "${SRC_PKG_DIR}" == "${PKG_DIR}" ]]; then
        return 0
    fi
    mkdir -p "${PKG_DIR}/${rel_dir}"
    cp -a "${SRC_PKG_DIR}/${rel_dir}/." "${PKG_DIR}/${rel_dir}/"
}

log "同步 launch/param/worlds/models 到 workspace 包 ..."
sync_dir "launch"
sync_dir "param"
sync_dir "worlds"
sync_dir "models"
ok "资源文件已同步"

log "复制 codefiles/ → cpe631_ros2/ ..."
cp -v "${SRC_PKG_DIR}/codefiles/ped_pose_extractor.py" "${PKG_DIR}/cpe631_ros2/ped_pose_extractor.py"
cp -v "${SRC_PKG_DIR}/codefiles/social_nav_node.py"    "${PKG_DIR}/cpe631_ros2/social_nav_node.py"
cp -v "${SRC_PKG_DIR}/codefiles/data_collector.py"     "${PKG_DIR}/cpe631_ros2/data_collector.py"
cp -v "${SRC_PKG_DIR}/codefiles/goal_sender.py"        "${PKG_DIR}/cpe631_ros2/goal_sender.py"
cp -v "${SRC_PKG_DIR}/codefiles/peds.py"               "${PKG_DIR}/cpe631_ros2/peds.py"
ok "改进版代码已复制"

# ── Step 2: colcon build ─────────────────────────────────────
if [[ "$SKIP_BUILD" == "1" ]]; then
    warn "跳过 colcon build (SKIP_BUILD=1)"
else
    log "在 ${WS_ROOT} 执行 colcon build ..."
    cd "${WS_ROOT}"
    colcon build --packages-select cpe631_ros2 --symlink-install
    ok "编译完成"
fi

# ── Step 3: source ───────────────────────────────────────────
if [[ ! -f "$SETUP_BASH" ]]; then
    err "找不到 ${SETUP_BASH}，编译可能失败"
    exit 1
fi
set +u
for distro in jazzy humble iron rolling; do
    if [[ -f "/opt/ros/${distro}/setup.bash" ]]; then
        source "/opt/ros/${distro}/setup.bash"
        break
    fi
done
source "$SETUP_BASH"
set -u
ok "已 source workspace"

log "启动实验前清理残留 ROS/Gazebo/DDS 进程..."
cleanup_full

# 默认不追加旧 CSV，避免重复 run 污染统计；需要续跑时显式 APPEND_CSV=1。
if [[ -f "$CSV_FILE" && "$APPEND_CSV" != "1" ]]; then
    CSV_BACKUP="${CSV_FILE}.bak_$(date +%Y%m%d_%H%M%S)"
    mv "$CSV_FILE" "$CSV_BACKUP"
    warn "已有 CSV 已备份到 ${CSV_BACKUP}，本次将新建 ${CSV_FILE}"
fi

# ── 实验定义 ──────────────────────────────────────────────────
#  格式: "MODE  NAVIGATION  SOCIAL_NAV  ENABLE_PEDS  PLANNER_VARIANT"
EXPERIMENTS=(
    "baseline      true  false  false  navfn"
    "dynamic       true  false  true   navfn"
    "dynamic_astar true  false  true   astar"
    "dynamic_dstar true  false  true   dstar"
    "social        true  true   true   navfn"
    "social_astar  true  true   true   astar"
    "social_smac   true  true   true   smac"
    "social_dstar      true  true   true   dstar"
    "social_dstar_plus     true  true   true   dstarplus"
    "social_dstar_plus_v2  true  true   true   dstarplusv2"
)

if [[ -n "$MODES" ]]; then
    FILTERED_EXPERIMENTS=()
    IFS=',' read -ra REQUESTED_MODES <<< "$MODES"
    for exp_line in "${EXPERIMENTS[@]}"; do
        read -r exp_mode _ <<< "$exp_line"
        for requested_mode in "${REQUESTED_MODES[@]}"; do
            requested_mode="${requested_mode//[[:space:]]/}"
            if [[ "$exp_mode" == "$requested_mode" ]]; then
                FILTERED_EXPERIMENTS+=("$exp_line")
                break
            fi
        done
    done
    EXPERIMENTS=("${FILTERED_EXPERIMENTS[@]}")
    if [[ ${#EXPERIMENTS[@]} -eq 0 ]]; then
        err "MODES='${MODES}' 没有匹配任何实验模式"
        exit 1
    fi
fi

# ── 主循环 ────────────────────────────────────────────────────
TOTAL_EXPERIMENTS=$(( ${#EXPERIMENTS[@]} * RUNS_PER_MODE ))
GLOBAL_SUCCEEDED=0
GLOBAL_FAILED=0
GLOBAL_SKIPPED=0

echo ""
echo -e "${BOLD}╔═══════════════════════════════════════════════════════╗${NC}"
echo -e "${BOLD}║     ROS2 Social Navigation 三组对比实验 自动脚本 v3  ║${NC}"
echo -e "${BOLD}╠═══════════════════════════════════════════════════════╣${NC}"
echo -e "${BOLD}║  工作区: ${NC}${WS_ROOT}"
echo -e "${BOLD}║  路线:   ${NC}${ROUTE}"
echo -e "${BOLD}║  每组重复: ${NC}${RUNS_PER_MODE} 次${BOLD}  MAX_RETRY: ${NC}${MAX_RETRY}"
echo -e "${BOLD}║  总计: ${NC}${TOTAL_EXPERIMENTS} 次实验"
echo -e "${BOLD}╚═══════════════════════════════════════════════════════╝${NC}"
echo ""

START_TIME=$(date +%s)

for exp_line in "${EXPERIMENTS[@]}"; do
    read -r mode navigation social_nav enable_peds planner_variant <<< "$exp_line"
    planner_variant="${planner_variant:-navfn}"

    echo ""
    echo -e "${BOLD}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${BOLD}  开始模式: ${CYAN}${mode}${NC}"
    echo -e "${BOLD}  navigation=${navigation}  social_navigation=${social_nav}  enable_peds=${enable_peds}  planner=${planner_variant}${NC}"
    echo -e "${BOLD}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"

    # 启动本 mode 的仿真
    if ! start_simulation "$mode" "$navigation" "$social_nav" "$enable_peds" "$planner_variant"; then
        err "模式 ${mode} 仿真启动失败，跳过整个模式"
        cleanup_full
        GLOBAL_FAILED=$((GLOBAL_FAILED + RUNS_PER_MODE))
        sleep "${COOLDOWN}"
        continue
    fi

    # 跑 RUNS_PER_MODE 个 experiments
    for experiment_id in $(seq 1 "$RUNS_PER_MODE"); do
        echo ""
        log "═══ ${mode} 实验 ${experiment_id}/${RUNS_PER_MODE} ═══"

        local_succeeded=false
        attempt=0

        while [[ $attempt -le $MAX_RETRY ]]; do
            log "尝试 ${attempt}/${MAX_RETRY} ..."

            # 先检查系统健康
            if ! check_system_health; then
                warn "系统不健康，尝试重启仿真..."
                cleanup_full
                sleep "${COOLDOWN}"
                if ! start_simulation "$mode" "$navigation" "$social_nav" "$enable_peds" "$planner_variant"; then
                    err "仿真重启失败，跳过 ${mode} 剩余实验"
                    # 标记剩余实验为失败
                    remaining=$((RUNS_PER_MODE - experiment_id + 1))
                    GLOBAL_FAILED=$((GLOBAL_FAILED + remaining))
                    break 2  # 跳出两层循环 (while + for)
                fi
            fi

            # 每个 attempt 都从同一机器人/行人初始状态开始。之前只在
            # abort 后重置机器人，成功后进入下一轮或刚启动第一轮时不会
            # 主动校准，容易把上一轮的位置和 Nav2 残留 action 带进新 run。
            reset_robot_pose
            reset_pedestrians "$enable_peds" || true

            # 跑实验
            set +e
            run_single_experiment "$mode" "$experiment_id" "$attempt"
            sender_exit=$?
            set -e

            case $sender_exit in
                0)
                    ok "实验 ${mode} #${experiment_id} 成功 (attempt=${attempt})"
                    local_succeeded=true
                    GLOBAL_SUCCEEDED=$((GLOBAL_SUCCEEDED + 1))
                    break
                    ;;
                2)
                    warn "实验 ${mode} #${experiment_id} 被 Nav2 abort (attempt=${attempt})"
                    reset_robot_pose
                    if [[ $attempt -lt $MAX_RETRY ]]; then
                        log "准备重试..."
                        sleep "${RUN_COOLDOWN}"
                    fi
                    ;;
                3)
                    warn "实验 ${mode} #${experiment_id} 被取消 (attempt=${attempt})"
                    break  # 不重试
                    ;;
                4)
                    warn "实验 ${mode} #${experiment_id} 超时 (attempt=${attempt})"
                    reset_robot_pose
                    break  # 超时不重试
                    ;;
                *)
                    err "实验 ${mode} #${experiment_id} 异常退出 (exit=${sender_exit}, attempt=${attempt})"
                    break  # 未知错误不重试
                    ;;
            esac

            attempt=$((attempt + 1))
        done

        if ! $local_succeeded; then
            GLOBAL_SKIPPED=$((GLOBAL_SKIPPED + 1))
            warn "实验 ${mode} #${experiment_id} 最终失败，已跳过"
        fi

        # experiment 间短冷却
        sleep "${RUN_COOLDOWN}"
    done

    # 本 mode 结束，完全清理
    ok "模式 ${mode} 全部实验完成，清理..."
    cleanup_full
    log "Mode 间冷却 ${COOLDOWN}s ..."
    sleep "${COOLDOWN}"
done

END_TIME=$(date +%s)
ELAPSED=$(( END_TIME - START_TIME ))
MINUTES=$(( ELAPSED / 60 ))
SECONDS_REM=$(( ELAPSED % 60 ))

echo ""
echo -e "${BOLD}╔═══════════════════════════════════════════════════════╗${NC}"
echo -e "${BOLD}║                   全部实验完成!                      ║${NC}"
echo -e "${BOLD}╠═══════════════════════════════════════════════════════╣${NC}"
echo -e "${BOLD}║  ${GREEN}成功: ${GLOBAL_SUCCEEDED}${NC}${BOLD}    ${YELLOW}跳过: ${GLOBAL_SKIPPED}${NC}${BOLD}    总计: ${TOTAL_EXPERIMENTS}${NC}"
echo -e "${BOLD}║  耗时: ${NC}${MINUTES}分${SECONDS_REM}秒"
echo -e "${BOLD}║  CSV 结果: ${NC}${CSV_FILE}"
echo -e "${BOLD}║  日志目录: ${NC}${LOG_DIR}/"
echo -e "${BOLD}╚═══════════════════════════════════════════════════════╝${NC}"
