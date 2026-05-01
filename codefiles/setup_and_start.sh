#!/usr/bin/env bash
# One-shot setup for the reboot-resume experiment runner.
# Run with: sudo bash codefiles/setup_and_start.sh

set -euo pipefail

USER_NAME="${EXP_USER:-dx}"
HOME_DIR="$(getent passwd "${USER_NAME}" | cut -d: -f6)"
WS_ROOT="${WS_ROOT:-${HOME_DIR}/ros2_ws/A}"
PKG_DIR="${PKG_DIR:-${HOME_DIR}/CPE631-Navigation-ROS2-main}"
RUNNER="${RUNNER:-${PKG_DIR}/codefiles/run_sequential_reboot.sh}"
LOG_DIR="${LOG_DIR:-${WS_ROOT}/run_logs}"
MASTER_LOG="${MASTER_LOG:-${LOG_DIR}/master.log}"
STATE_FILE="${STATE_FILE:-${WS_ROOT}/.exp_next_mode}"
CRON_MARKER="${CRON_MARKER:-CPE631_REBOOT_EXPERIMENT}"

if [[ "${EUID}" -ne 0 ]]; then
  echo "Please run as root: sudo bash codefiles/setup_and_start.sh"
  exit 1
fi

if [[ ! -f "${RUNNER}" ]]; then
  echo "Runner not found: ${RUNNER}"
  exit 1
fi

install -d -o "${USER_NAME}" -g "${USER_NAME}" "${LOG_DIR}"
touch "${MASTER_LOG}"
chown "${USER_NAME}:${USER_NAME}" "${MASTER_LOG}"

chmod +x "${RUNNER}"
chmod +x "${PKG_DIR}/codefiles/run_all_experiments.sh" 2>/dev/null || true

cat >/etc/sudoers.d/exp_reboot <<EOF
${USER_NAME} ALL=(ALL) NOPASSWD: /sbin/reboot
EOF
chmod 0440 /etc/sudoers.d/exp_reboot
visudo -cf /etc/sudoers.d/exp_reboot >/dev/null

if [[ -f /etc/gdm3/custom.conf ]]; then
  cp -n /etc/gdm3/custom.conf /etc/gdm3/custom.conf.bak
  if grep -q '^AutomaticLoginEnable=' /etc/gdm3/custom.conf; then
    sed -i 's/^AutomaticLoginEnable=.*/AutomaticLoginEnable=True/' /etc/gdm3/custom.conf
  else
    sed -i '/^\[daemon\]/a AutomaticLoginEnable=True' /etc/gdm3/custom.conf
  fi
  if grep -q '^AutomaticLogin=' /etc/gdm3/custom.conf; then
    sed -i "s/^AutomaticLogin=.*/AutomaticLogin=${USER_NAME}/" /etc/gdm3/custom.conf
  else
    sed -i "/^\[daemon\]/a AutomaticLogin=${USER_NAME}" /etc/gdm3/custom.conf
  fi
fi

runuser -u "${USER_NAME}" -- bash -lc "echo 0 > '${STATE_FILE}'"
runuser -u "${USER_NAME}" -- bash -lc "rm -f '${WS_ROOT}/.exp_reboot_done' '${WS_ROOT}/.exp_reboot.lock'"

CRON_CMD="@reboot sleep 20; ${RUNNER} --from-reboot # ${CRON_MARKER}"
tmp_cron="$(mktemp)"
runuser -u "${USER_NAME}" -- bash -lc "crontab -l 2>/dev/null | grep -v '${CRON_MARKER}'" >"${tmp_cron}" || true
printf '%s\n' "${CRON_CMD}" >>"${tmp_cron}"
crontab -u "${USER_NAME}" "${tmp_cron}"
rm -f "${tmp_cron}"

cat <<EOF
Setup complete.

State file: ${STATE_FILE}
Master log: ${MASTER_LOG}
Runner:     ${RUNNER}

Starting the first mode now as ${USER_NAME}.
Monitor with:
  tail -f ${MASTER_LOG}
EOF

runuser -u "${USER_NAME}" -- bash -lc "'${RUNNER}'"
