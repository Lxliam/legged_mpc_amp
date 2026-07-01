#!/bin/bash
# Automated AMP data collection script
# Collects motion data for multiple gaits and velocities

ROBOT_TYPE=${ROBOT_TYPE:-go2}
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
AMP_LOG_DIR=${AMP_LOG_DIR:-${PROJECT_DIR}/amp_data}
DURATION=${DURATION:-10}

GAITS=("trot" "pace" "dynamic_walk" "static_walk" "flying_trot")
VELOCITIES=("0.2" "0.4" "0.6" "0.8")

echo "============================================"
echo "  AMP Data Collection for ${ROBOT_TYPE}"
echo "  Log directory: ${AMP_LOG_DIR}"
echo "============================================"

mkdir -p ${AMP_LOG_DIR}

echo "[1/3] Starting simulation..."
export ROBOT_TYPE=${ROBOT_TYPE}
roslaunch legged_robot_description empty_world.launch &
SIM_PID=$!
sleep 5

echo "[2/3] Loading controller with AMP logging..."
roslaunch legged_controllers load_controller.launch \
    enable_amp_logging:=true \
    amp_log_dir:=${AMP_LOG_DIR} \
    amp_log_prefix:=motion &
CTRL_PID=$!
sleep 3

echo "[3/3] Starting controller..."
rosservice call /controller_manager/switch_controller "start_controllers: ['controllers/legged_controller']
stop_controllers: ['']
strictness: 0
start_asap: false
timeout: 0.0"
sleep 3

echo "Controller started. Beginning data collection..."

for gait in "${GAITS[@]}"; do
    for vel in "${VELOCITIES[@]}"; do
        echo ""
        echo "=== Collecting: gait=${gait}, vel=${vel} ==="

        LOG_PREFIX="${gait}_v${vel}"

        rosservice call /controller_manager/switch_controller "start_controllers: ['controllers/legged_controller']
stop_controllers: ['']
strictness: 0
start_asap: false
timeout: 0.0"
        sleep 1

        rostopic pub -r 10 /cmd_vel geometry_msgs/Twist "{linear: {x: ${vel}, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" &
        VEL_PID=$!

        echo "  Recording for ${DURATION} seconds..."
        sleep ${DURATION}

        kill $VEL_PID 2>/dev/null
        wait $VEL_PID 2>/dev/null

        rostopic pub -1 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

        echo "  Switching to stance..."
        sleep 2

        echo "  Done: ${LOG_PREFIX}"
    done
done

echo ""
echo "============================================"
echo "  Data collection complete!"
echo "  Log files: ${AMP_LOG_DIR}/"
echo ""
echo "  Next step: Convert to AMP format:"
echo "  python3 scripts/convert_amp_data.py --input_dir ${AMP_LOG_DIR} --output_dir ${PROJECT_DIR}/amp_dataset"
echo "============================================"

kill $CTRL_PID $SIM_PID 2>/dev/null
