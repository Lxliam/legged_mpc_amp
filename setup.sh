#!/usr/bin/env bash
# Setup script for legged_mpc_amp project
# This project uses NMPC-WBC to generate gait motion data for AMP (Adversarial Motion Priors) training

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="${SCRIPT_DIR}"
DEFAULT_ROS_DISTRO="${ROS_DISTRO:-noetic}"
ROS_SETUP="${ROS_SETUP:-}"
PINOCCHIO_PKGCONFIG="${PINOCCHIO_PKGCONFIG:-}"
CATKIN_BUILD_ARGS="${CATKIN_BUILD_ARGS:--DCMAKE_BUILD_TYPE=RelWithDebInfo}"

prepend_path_once() {
    local var_name="$1"
    local path_value="$2"

    if [ -z "${path_value}" ] || [ ! -d "${path_value}" ]; then
        return
    fi

    eval "local current_value=\"\${${var_name}:-}\""
    case ":${current_value}:" in
        *":${path_value}:"*) ;;
        *) export "${var_name}=${path_value}${current_value:+:${current_value}}" ;;
    esac
}

detect_ros_setup() {
    if [ -n "${ROS_SETUP}" ]; then
        return
    fi

    if [ -f "/opt/ros/${DEFAULT_ROS_DISTRO}/setup.bash" ]; then
        ROS_SETUP="/opt/ros/${DEFAULT_ROS_DISTRO}/setup.bash"
        return
    fi

    local setup_candidate
    for setup_candidate in /opt/ros/*/setup.bash; do
        if [ -f "${setup_candidate}" ]; then
            ROS_SETUP="${setup_candidate}"
            return
        fi
    done
}

detect_pinocchio_pkgconfig() {
    if [ -n "${PINOCCHIO_PKGCONFIG}" ]; then
        return
    fi

    # The ROS setup has already populated PKG_CONFIG_PATH. Do not silently
    # select a conda installation, which can mix Pinocchio with ROS HPP-FCL.
    # Custom and conda prefixes must be selected explicitly by the caller.
}

print_dependency_help() {
    echo "System dependencies still required outside the workspace include:"
    echo "  python3-catkin-tools, python3-rosdep, libeigen3-dev, libboost-all-dev,"
    echo "  liburdfdom-dev, Pinocchio, HPP-FCL, and ROS controller/Gazebo packages."
    echo ""
    if command -v apt-get >/dev/null 2>&1; then
        echo "Recommended Ubuntu/Debian install command:"
        echo "  sudo apt update"
        echo "  sudo apt install python3-catkin-tools python3-rosdep libeigen3-dev libboost-all-dev liburdfdom-dev ros-${DEFAULT_ROS_DISTRO}-pinocchio ros-${DEFAULT_ROS_DISTRO}-hpp-fcl"
        echo "  rosdep install --from-paths src --ignore-src -r -y"
    else
        echo "This script did not detect apt-get. Install equivalent packages with your system package manager,"
        echo "then run rosdep if available:"
        echo "  rosdep install --from-paths src --ignore-src -r -y"
    fi
    echo ""
}

echo "============================================================"
echo "  legged_mpc_amp - NMPC-WBC Gait Generator for AMP Training"
echo "============================================================"
echo ""
echo "Project directory: ${PROJECT_DIR}"
echo ""

detect_ros_setup

if [ -z "${ROS_SETUP}" ] || [ ! -f "${ROS_SETUP}" ]; then
    echo "[ERROR] ROS setup file was not found."
    echo "Set ROS_SETUP to your ROS setup file, for example:"
    echo "  ROS_SETUP=/opt/ros/noetic/setup.bash bash setup.sh"
    exit 1
fi

# Start from a clean ROS prefix so older OCS2 workspaces sourced in the parent
# shell do not leak into this workspace build.
unset CMAKE_PREFIX_PATH
unset ROS_PACKAGE_PATH
source "${ROS_SETUP}"
ROS_PREFIX="$(cd "$(dirname "${ROS_SETUP}")" && pwd)"

detect_pinocchio_pkgconfig
if [ -n "${PINOCCHIO_PKGCONFIG}" ] && [ -f "${PINOCCHIO_PKGCONFIG}/pinocchio.pc" ]; then
    prepend_path_once PKG_CONFIG_PATH "${PINOCCHIO_PKGCONFIG}"
    echo "[OK] Pinocchio pkg-config found: ${PINOCCHIO_PKGCONFIG}"
else
    echo "[INFO] No extra Pinocchio pkg-config path was detected."
    echo "       If Pinocchio is installed in conda or a custom prefix, set:"
    echo "       PINOCCHIO_PKGCONFIG=/path/to/env/lib/pkgconfig bash setup.sh"
fi

if ! command -v catkin >/dev/null 2>&1; then
    echo "[ERROR] catkin command was not found."
    echo "Install catkin tools first:"
    echo "  sudo apt install python3-catkin-tools"
    exit 1
fi

if ! command -v rosdep >/dev/null 2>&1; then
    echo "[WARN] rosdep command was not found. Dependency resolution must be handled manually."
fi

if [ ! -d "${PROJECT_DIR}/src/third_party/ocs2" ] || [ ! -d "${PROJECT_DIR}/src/third_party/ocs2_robotic_assets" ]; then
    echo "[ERROR] Vendored OCS2 sources are missing."
    echo "Expected:"
    echo "  ${PROJECT_DIR}/src/third_party/ocs2"
    echo "  ${PROJECT_DIR}/src/third_party/ocs2_robotic_assets"
    echo ""
    echo "Clone them into this workspace with:"
    echo "  git clone https://github.com/leggedrobotics/ocs2.git src/third_party/ocs2"
    echo "  git clone https://github.com/leggedrobotics/ocs2_robotic_assets.git src/third_party/ocs2_robotic_assets"
    exit 1
fi

echo "[OK] ROS found: ${ROS_SETUP}"
echo "[OK] Vendored OCS2 sources found in src/third_party."
echo ""
print_dependency_help

# Build the project
echo "Building project..."
cd "${PROJECT_DIR}"
catkin config --extend "${ROS_PREFIX}"
catkin build ${CATKIN_BUILD_ARGS}

echo ""
echo "============================================================"
echo "  Build complete!"
echo "============================================================"
echo ""
echo "=== Project Structure ==="
echo ""
echo "  src/legged_common       - Hardware interface definitions"
echo "  src/legged_interface    - NMPC optimal control (OCS2)"
echo "  src/legged_wbc          - Whole body controller"
echo "  src/legged_estimation   - State estimation (Kalman filter)"
echo "  src/legged_hw           - Hardware abstraction base class"
echo "  src/legged_gazebo       - Gazebo simulation interface"
echo "  src/legged_controllers  - Core controller + AMP data logging"
echo "  src/legged_robot_description - Robot URDF/meshes"
echo "  src/qpoases_catkin      - QP solver for WBC"
echo "  src/third_party/ocs2    - Vendored OCS2 packages"
echo "  src/third_party/ocs2_robotic_assets - OCS2 robot assets"
echo "  scripts/                - Data collection & conversion scripts"
echo ""
echo "=== How to Collect AMP Data ==="
echo ""
echo "  # Step 1: Load the workspace environment in each new terminal"
echo "  source ${PROJECT_DIR}/env.sh go2"
echo ""
echo "  # Robot type can be changed, for example:"
echo "  source ${PROJECT_DIR}/env.sh a1"
echo ""
echo "  # Step 2: Launch simulation"
echo "  roslaunch legged_robot_description empty_world.launch"
echo ""
echo "  # Step 3: Load controller and keyboard control (in another terminal)"
echo "  roslaunch legged_controllers keyboard_control.launch \\"
echo "      enable_amp_logging:=true"
echo ""
echo "  # Step 4: In the keyboard terminal:"
echo "  #   press i to start the controller and stand"
echo "  #   press 1 to switch to trot"
echo "  #   press l to start AMP recording"
echo ""
echo "  # Step 5: Drive while recording, then press l again to stop and save"
echo "  # Use w/s/a/d/q/e for velocity commands"
echo ""
echo "  # Step 6: Convert data to AMP format"
echo "  python3 ${PROJECT_DIR}/scripts/convert_amp_data.py \\"
echo "      --input_dir ${PROJECT_DIR}/amp_data --output_dir ${PROJECT_DIR}/amp_dataset --normalize"
echo ""
echo "=== Or use automated collection ==="
echo ""
echo "  bash ${PROJECT_DIR}/scripts/collect_amp_data.sh"
echo ""
echo "=== How to Adapt to Your Robot ==="
echo ""
echo "  1. Create URDF: src/legged_robot_description/urdf/<your_robot>/const.xacro"
echo "  2. Add meshes:   src/legged_robot_description/meshes/<your_robot>/"
echo "  3. Create config: src/legged_controllers/config/<your_robot>/task.info, reference.info, gait.info"
echo "  4. Set ROBOT_TYPE=<your_robot>"
echo ""
