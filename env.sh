#!/usr/bin/env bash
# Source this file in every new terminal before running this workspace.

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    echo "[ERROR] env.sh must be sourced so it can update the current shell."
    echo "Usage:"
    echo "  source ${0} [robot_type]"
    echo ""
    echo "Example:"
    echo "  source ${0} go2"
    echo "  source ${0} Lite3"
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="${SCRIPT_DIR}"
ROBOT_TYPE_INPUT="${1:-${ROBOT_TYPE:-go2}}"
DEFAULT_ROS_DISTRO="${ROS_DISTRO:-noetic}"
ROS_SETUP="${ROS_SETUP:-}"
PINOCCHIO_PKGCONFIG="${PINOCCHIO_PKGCONFIG:-}"
PINOCCHIO_LIB_DIR="${PINOCCHIO_LIB_DIR:-}"

prepend_ld_preload() {
    local library_path="$1"
    if [ ! -f "${library_path}" ]; then
        return
    fi

    case ":${LD_PRELOAD:-}:" in
        *":${library_path}:"*) ;;
        *) export LD_PRELOAD="${library_path}${LD_PRELOAD:+:${LD_PRELOAD}}" ;;
    esac
}

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

    # Prefer the Pinocchio provided by the sourced ROS environment. Custom
    # and conda prefixes must be selected explicitly by the caller.
}

detect_ros_setup

if [ ! -f "${ROS_SETUP}" ]; then
    echo "[ERROR] ROS setup file was not found: ${ROS_SETUP}"
    return 1
fi

source "${ROS_SETUP}"

detect_pinocchio_pkgconfig

if [ -f "${PINOCCHIO_PKGCONFIG}/pinocchio.pc" ]; then
    prepend_path_once PKG_CONFIG_PATH "${PINOCCHIO_PKGCONFIG}"
    PINOCCHIO_LIB_DIR="${PINOCCHIO_LIB_DIR:-${PINOCCHIO_PKGCONFIG%/pkgconfig}}"

    if [ -n "${LD_LIBRARY_PATH:-}" ]; then
        LD_LIBRARY_PATH=":${LD_LIBRARY_PATH}:"
        LD_LIBRARY_PATH="${LD_LIBRARY_PATH//:${PINOCCHIO_LIB_DIR}:/:}"
        LD_LIBRARY_PATH="${LD_LIBRARY_PATH#:}"
        LD_LIBRARY_PATH="${LD_LIBRARY_PATH%:}"
        export LD_LIBRARY_PATH
    fi

    for PRELOAD_LIBRARY in \
        "${PINOCCHIO_LIB_DIR}/libstdc++.so.6" \
        "${PINOCCHIO_LIB_DIR}/liboctomath.so.1.9" \
        "${PINOCCHIO_LIB_DIR}/liboctomap.so.1.9"; do
        prepend_ld_preload "${PRELOAD_LIBRARY}"
    done
fi

if [ ! -f "${PROJECT_DIR}/devel/setup.bash" ]; then
    echo "[WARN] Workspace has not been built yet: ${PROJECT_DIR}/devel/setup.bash"
    echo "       Run bash setup.sh first, then source this file again."
else
    source "${PROJECT_DIR}/devel/setup.bash"
fi

export ROBOT_TYPE="${ROBOT_TYPE_INPUT}"
export LEGGED_MPC_AMP_ROOT="${PROJECT_DIR}"

echo "[OK] legged_mpc_amp environment loaded."
echo "     Workspace: ${LEGGED_MPC_AMP_ROOT}"
echo "     ROBOT_TYPE=${ROBOT_TYPE}"
if [ -n "${LD_PRELOAD:-}" ]; then
    echo "     Preload: ${LD_PRELOAD}"
fi
