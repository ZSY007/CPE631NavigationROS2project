#!/bin/bash
# 顺序跑 dynamic / social / social_astar / social_smac 四组各 25 轮
# 每组结束后自动清理，等待 BETWEEN_COOLDOWN 秒再启动下一组
# 任意一组失败不影响后续组继续执行

set -uo pipefail

WS_ROOT="/home/dx/ros2_ws/A"
PKG_DIR="${WS_ROOT}/CPE631-Navigation-ROS2-main"
SCRIPT="${PKG_DIR}/codefiles/run_all_experiments.sh"

RUNS="${RUNS_PER_MODE:-25}"
BETWEEN_COOLDOWN="${BETWEEN_COOLDOWN:-30}"
LOG_DIR="${LOG_DIR:-${WS_ROOT}/run_logs}"

mkdir -p "${LOG_DIR}"

COMMON_OPTS=(
    RUNS_PER_MODE="${RUNS}"
    RESET_PEDS_EACH_RUN=1
    GZ_GUI=0
    ENABLE_RVIZ=0
    SKIP_BUILD=0
)

declare -A CSV_MAP=(
    [dynamic]="${WS_ROOT}/dyn_clean.csv"
    [social]="${WS_ROOT}/soc_clean.csv"
    [social_astar]="${WS_ROOT}/astar_clean.csv"
    [social_smac]="${WS_ROOT}/smac_clean.csv"
)

MODES_ORDER=(dynamic social social_astar social_smac)

log() { echo -e "\033[0;36m[$(date +%H:%M:%S)] $*\033[0m"; }
ok()  { echo -e "\033[0;32m[$(date +%H:%M:%S)] ✔ $*\033[0m"; }
err() { echo -e "\033[0;31m[$(date +%H:%M:%S)] ✘ $*\033[0m"; }

OVERALL_START=$(date +%s)
FAILED_MODES=()

for mode in "${MODES_ORDER[@]}"; do
    csv="${CSV_MAP[$mode]}"
    log_file="${LOG_DIR}/${mode}_$(date +%Y%m%d_%H%M%S).log"

    log "════════════════════════════════════"
    log "开始模式: ${mode}  (${RUNS} 轮)"
    log "CSV  → ${csv}"
    log "日志 → ${log_file}"
    log "════════════════════════════════════"

    set +e
    env "${COMMON_OPTS[@]}" \
        MODES="${mode}" \
        CSV_FILE="${csv}" \
        bash "${SCRIPT}" 2>&1 | tee "${log_file}"
    exit_code=${PIPESTATUS[0]}
    set -e

    if [[ $exit_code -eq 0 ]]; then
        ok "模式 ${mode} 完成"
    else
        err "模式 ${mode} 异常退出 (code=${exit_code})，继续下一组"
        FAILED_MODES+=("${mode}")
    fi

    log "冷却 ${BETWEEN_COOLDOWN}s 后启动下一组..."
    sleep "${BETWEEN_COOLDOWN}"
done

ELAPSED=$(( $(date +%s) - OVERALL_START ))
echo ""
echo "════════════ 全部结束 ════════════"
echo "总耗时: $(( ELAPSED / 60 ))分$(( ELAPSED % 60 ))秒"
if [[ ${#FAILED_MODES[@]} -eq 0 ]]; then
    ok "四组全部成功"
else
    err "以下模式异常退出: ${FAILED_MODES[*]}"
    echo "对应日志在 ${LOG_DIR}/ 里检查"
fi
echo "=================================="
