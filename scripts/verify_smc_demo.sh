#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"

source_ros_setup() {
  if [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    set +u
    # shellcheck disable=SC1090
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
    set -u
    return
  fi

  local distros=(jazzy humble iron rolling)
  local distro
  for distro in "${distros[@]}"; do
    if [[ -f "/opt/ros/${distro}/setup.bash" ]]; then
      set +u
      # shellcheck disable=SC1090
      source "/opt/ros/${distro}/setup.bash"
      set -u
      return
    fi
  done

  echo "No ROS 2 setup.bash found under /opt/ros" >&2
  exit 1
}

cleanup() {
  if [[ -n "${LAUNCH_PID:-}" ]] && kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    kill -INT "${LAUNCH_PID}" 2>/dev/null || true
    for _ in {1..20}; do
      if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
        break
      fi
      sleep 0.2
    done
    kill -TERM "${LAUNCH_PID}" 2>/dev/null || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi
}

trap cleanup EXIT

source_ros_setup

cd "${WORKSPACE_DIR}"
colcon build --packages-select smc_demo
set +u
source "${WORKSPACE_DIR}/install/setup.bash"
set -u

LOG_DIR="$(mktemp -d)"
LAUNCH_LOG="${LOG_DIR}/launch.log"
STATE_LOG="${LOG_DIR}/state.log"
TF_LOG="${LOG_DIR}/tf.log"

ros2 launch smc_demo smc_demo.launch.py start_rviz:=false >"${LAUNCH_LOG}" 2>&1 &
LAUNCH_PID=$!

sleep 3

timeout 10s ros2 topic echo --once /smc/state >"${STATE_LOG}"
timeout 10s ros2 topic echo --once /tf >"${TF_LOG}"

cleanup
trap - EXIT

if grep -Eq "Traceback|rcl_shutdown already called|\\[ERROR\\]" "${LAUNCH_LOG}"; then
  echo "Launch log contains errors:" >&2
  cat "${LAUNCH_LOG}" >&2
  exit 1
fi

if ! grep -q "data:" "${STATE_LOG}"; then
  echo "Did not receive /smc/state payload" >&2
  exit 1
fi

if ! grep -q "child_frame_id: base_link" "${TF_LOG}"; then
  echo "Did not receive expected /tf transform" >&2
  exit 1
fi

echo "smc_demo smoke test passed"
