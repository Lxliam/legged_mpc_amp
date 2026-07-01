#!/usr/bin/env bash
# One-shot AMP data collection for trot vx/vy/yaw motion sequences.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

ROBOT_TYPE="${ROBOT_TYPE:-go2}"
RUN_ID="${RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
AMP_LOG_DIR="${AMP_LOG_DIR:-${PROJECT_DIR}/amp_data/auto_trot_${RUN_ID}}"
AMP_LOG_PREFIX="${AMP_LOG_PREFIX:-trot_auto}"
GAZEBO_GUI="${GAZEBO_GUI:-false}"
GAZEBO_HEADLESS="${GAZEBO_HEADLESS:-true}"
GAZEBO_PAUSED="${GAZEBO_PAUSED:-false}"
SIM_STARTUP_WAIT="${SIM_STARTUP_WAIT:-8}"
CTRL_STARTUP_WAIT="${CTRL_STARTUP_WAIT:-4}"

SIM_PID=""
CTRL_PID=""

cleanup() {
    set +e
    rostopic pub -1 /amp/enable_logging std_msgs/Bool "data: false" >/dev/null 2>&1
    rostopic pub -1 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" >/dev/null 2>&1

    for pid in "${CTRL_PID}" "${SIM_PID}"; do
        if [ -n "${pid}" ] && kill -0 "${pid}" >/dev/null 2>&1; then
            kill -INT "${pid}" >/dev/null 2>&1
        fi
    done
    sleep 2
    for pid in "${CTRL_PID}" "${SIM_PID}"; do
        if [ -n "${pid}" ] && kill -0 "${pid}" >/dev/null 2>&1; then
            kill -TERM "${pid}" >/dev/null 2>&1
        fi
    done
}
trap cleanup EXIT INT TERM

if [ -f "${PROJECT_DIR}/env.sh" ]; then
    # shellcheck source=/dev/null
    source "${PROJECT_DIR}/env.sh" "${ROBOT_TYPE}"
fi

mkdir -p "${AMP_LOG_DIR}"

echo "============================================"
echo "  Automatic trot AMP data collection"
echo "  Robot:      ${ROBOT_TYPE}"
echo "  Log dir:    ${AMP_LOG_DIR}"
echo "  Prefix:     ${AMP_LOG_PREFIX}"
echo "  Gazebo GUI: ${GAZEBO_GUI}"
echo "============================================"

roslaunch legged_robot_description empty_world.launch \
    robot_type:="${ROBOT_TYPE}" \
    gui:="${GAZEBO_GUI}" \
    headless:="${GAZEBO_HEADLESS}" \
    paused:="${GAZEBO_PAUSED}" &
SIM_PID=$!

echo "[1/3] Gazebo launched, waiting ${SIM_STARTUP_WAIT}s ..."
sleep "${SIM_STARTUP_WAIT}"

roslaunch legged_controllers load_controller.launch \
    robot_type:="${ROBOT_TYPE}" \
    cheater:=false \
    enable_amp_logging:=true \
    amp_log_dir:="${AMP_LOG_DIR}" \
    amp_log_prefix:="${AMP_LOG_PREFIX}" \
    amp_log_frequency:=50.0 \
    amp_start_recording:=false \
    start_gait_keyboard:=false &
CTRL_PID=$!

echo "[2/3] Controller launch started, waiting ${CTRL_STARTUP_WAIT}s ..."
sleep "${CTRL_STARTUP_WAIT}"

echo "[3/3] Running automatic vx/vy/yaw trot schedule ..."
rosrun legged_controllers auto_amp_data_collector.py \
    --robot-name legged_robot \
    --gait trot \
    --amp-log-dir "${AMP_LOG_DIR}" \
    --amp-log-prefix "${AMP_LOG_PREFIX}" \
    --amp-log-frequency 50.0 \
    --manifest "${AMP_LOG_DIR}/collection_manifest.json"

echo ""
echo "============================================"
echo "  AMP collection finished"
echo "  CSV logs:  ${AMP_LOG_DIR}"
echo "  Manifest:  ${AMP_LOG_DIR}/collection_manifest.json"
echo ""
echo "  Convert for IsaacLab:"
echo "  python3 scripts/convert_amp_data_isaaclab.py --input_dir ${AMP_LOG_DIR} --output_dir ${PROJECT_DIR}/amp_dataset_isaaclab"
echo "============================================"
