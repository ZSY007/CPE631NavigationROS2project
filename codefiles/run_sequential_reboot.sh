#!/usr/bin/env bash
# Run dynamic -> reboot -> social -> reboot -> social_astar -> reboot -> social_smac.
# This script is intended to be started by cron @reboot after setup.

set -uo pipefail

USER_NAME="${EXP_USER:-dx}"
HOME_DIR="${EXP_HOME:-/home/${USER_NAME}}"
WS_ROOT="${WS_ROOT:-${HOME_DIR}/ros2_ws/A}"
PKG_DIR="${PKG_DIR:-${HOME_DIR}/CPE631-Navigation-ROS2-main}"
EXP_SCRIPT="${EXP_SCRIPT:-${PKG_DIR}/codefiles/run_all_experiments.sh}"

STATE_FILE="${STATE_FILE:-${WS_ROOT}/.exp_next_mode}"
DONE_FILE="${DONE_FILE:-${WS_ROOT}/.exp_reboot_done}"
LOCK_FILE="${LOCK_FILE:-${WS_ROOT}/.exp_reboot.lock}"
LOG_DIR="${LOG_DIR:-${WS_ROOT}/run_logs}"
MASTER_LOG="${MASTER_LOG:-${LOG_DIR}/master.log}"

RUNS_PER_MODE="${RUNS_PER_MODE:-25}"
REBOOT_WAIT="${REBOOT_WAIT:-90}"
BETWEEN_REBOOT_DELAY="${BETWEEN_REBOOT_DELAY:-20}"
CRON_MARKER="${CRON_MARKER:-CPE631_REBOOT_EXPERIMENT}"

MODES_ORDER=(dynamic social social_astar social_smac)
CSV_FILES=(
  "${WS_ROOT}/dyn_clean.csv"
  "${WS_ROOT}/soc_clean.csv"
  "${WS_ROOT}/astar_clean.csv"
  "${WS_ROOT}/smac_clean.csv"
)

mkdir -p "${LOG_DIR}"
touch "${MASTER_LOG}"

if [[ "${1:-}" == "--status" ]]; then
  echo "state_file=${STATE_FILE}"
  echo "state=$(cat "${STATE_FILE}" 2>/dev/null || echo 0)"
  echo "done_file=${DONE_FILE}"
  echo "master_log=${MASTER_LOG}"
  echo "runner=${PKG_DIR}/codefiles/run_sequential_reboot.sh"
  echo "experiment_script=${EXP_SCRIPT}"
  exit 0
fi

exec >>"${MASTER_LOG}" 2>&1

log() { printf '[%s] %s\n' "$(date '+%F %T')" "$*"; }

remove_cron_entry() {
  local tmp
  tmp="$(mktemp)"
  crontab -l 2>/dev/null | grep -v "${CRON_MARKER}" >"${tmp}" || true
  crontab "${tmp}" || true
  rm -f "${tmp}"
}

finish_all() {
  log "All modes completed. Removing @reboot cron entry."
  echo "done $(date '+%F %T')" >"${DONE_FILE}"
  remove_cron_entry
  rm -f "${LOCK_FILE}"
}

if [[ "${1:-}" == "--reset" ]]; then
  echo 0 >"${STATE_FILE}"
  rm -f "${DONE_FILE}" "${LOCK_FILE}"
  log "State reset to 0."
  exit 0
fi

if ! mkdir "${LOCK_FILE}" 2>/dev/null; then
  log "Another run_sequential_reboot.sh instance is active; exiting."
  exit 0
fi
trap 'rm -rf "${LOCK_FILE}"' EXIT

if [[ "${1:-}" == "--from-reboot" ]]; then
  log "Started from @reboot; waiting ${REBOOT_WAIT}s for system services."
  sleep "${REBOOT_WAIT}"
fi

if [[ ! -x "${EXP_SCRIPT}" ]]; then
  chmod +x "${EXP_SCRIPT}" 2>/dev/null || true
fi
if [[ ! -f "${EXP_SCRIPT}" ]]; then
  log "ERROR: experiment script not found: ${EXP_SCRIPT}"
  exit 1
fi

if [[ ! -f "${STATE_FILE}" ]]; then
  echo 0 >"${STATE_FILE}"
fi

idx="$(cat "${STATE_FILE}" 2>/dev/null | tr -cd '0-9')"
idx="${idx:-0}"

if (( idx >= ${#MODES_ORDER[@]} )); then
  finish_all
  exit 0
fi

mode="${MODES_ORDER[$idx]}"
csv_file="${CSV_FILES[$idx]}"
mode_log="${LOG_DIR}/${mode}_$(date '+%Y%m%d_%H%M%S').log"

log "============================================================"
log "Starting mode index=${idx}, mode=${mode}, runs=${RUNS_PER_MODE}"
log "CSV: ${csv_file}"
log "Mode log: ${mode_log}"
log "Experiment script: ${EXP_SCRIPT}"
log "============================================================"

set +e
env \
  RUNS_PER_MODE="${RUNS_PER_MODE}" \
  RESET_PEDS_EACH_RUN=1 \
  GZ_GUI="${GZ_GUI:-1}" \
  ENABLE_RVIZ="${ENABLE_RVIZ:-1}" \
  SKIP_BUILD="${SKIP_BUILD:-0}" \
  MODES="${mode}" \
  CSV_FILE="${csv_file}" \
  bash "${EXP_SCRIPT}" 2>&1 | tee "${mode_log}"
exit_code=${PIPESTATUS[0]}
set -e

log "Mode ${mode} finished with exit_code=${exit_code}."

next_idx=$((idx + 1))
echo "${next_idx}" >"${STATE_FILE}"

if (( next_idx >= ${#MODES_ORDER[@]} )); then
  finish_all
  exit "${exit_code}"
fi

log "Next mode index=${next_idx}. Rebooting in ${BETWEEN_REBOOT_DELAY}s."
sleep "${BETWEEN_REBOOT_DELAY}"
sudo /sbin/reboot
