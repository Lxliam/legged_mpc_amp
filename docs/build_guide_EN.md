# Build and Environment Guide

This project vendors OCS2 and `ocs2_robotic_assets` under `src/third_party/`, so they are built as part of the current catkin workspace. You do not need to maintain a separate `~/ocs2_catkin_ws`.

Vendored third-party packages:

```text
src/third_party/ocs2
src/third_party/ocs2_robotic_assets
src/qpoases_catkin
```

OCS2 RaiSim, MPCNet, documentation, and unrelated example packages are skipped through `CATKIN_IGNORE`. Pinocchio, HPP-FCL, Gazebo, ROS control, and related packages are still installed as system dependencies.

The vendored OCS2 sources are patched for the ROS Noetic API. Use one Pinocchio/HPP-FCL
installation consistently; do not mix the ROS packages with conda libraries.

## Install Dependencies

The example below targets Ubuntu 20.04 + ROS Noetic. For other ROS 1 distributions, install equivalent packages:

```bash
sudo apt update
sudo apt install \
  python3-catkin-tools python3-rosdep \
  libeigen3-dev libboost-all-dev liburdfdom-dev \
  ros-noetic-pinocchio ros-noetic-hpp-fcl

cd /path/to/legged_mpc_amp
source /opt/ros/noetic/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

If Pinocchio is installed from conda or a custom prefix, make sure `pkg-config` can find it:

```bash
export PINOCCHIO_PKGCONFIG="${CONDA_PREFIX}/lib/pkgconfig"
```

For the ROS Noetic packages above, this variable is not needed. Do not point it
at a conda or custom installation unless that installation is intentionally
used for both Pinocchio and HPP-FCL.

If a previous build selected a conda Pinocchio, remove that cached CMake option
before rebuilding with the ROS packages:

```bash
catkin config --remove-args -Dpinocchio_DIR=/path/to/conda/lib/cmake/pinocchio
catkin clean --build -y
```

## Build

```bash
cd /path/to/legged_mpc_amp
bash setup.sh
```

`setup.sh` automatically looks for the ROS environment matching `${ROS_DISTRO}` and defaults to `noetic`. To override paths or build arguments:

```bash
ROS_SETUP=/opt/ros/noetic/setup.bash \
PINOCCHIO_PKGCONFIG="${CONDA_PREFIX}/lib/pkgconfig" \
CATKIN_BUILD_ARGS="-DCMAKE_BUILD_TYPE=RelWithDebInfo" \
bash setup.sh
```

With the ROS Noetic packages, omit `PINOCCHIO_PKGCONFIG`. If your machine does not use Noetic, point `ROS_SETUP` to the matching ROS 1 `setup.bash`.

## Source Every Terminal

The default robot is Go2. Run this first in every new terminal:

```bash
cd /path/to/legged_mpc_amp
source env.sh go2
```

You can switch to another configured robot:

```bash
source env.sh a1
source env.sh go1
source env.sh aliengo
source env.sh Lite3
```

`env.sh` and `setup.sh` support the same path overrides:

```bash
export ROS_SETUP=/opt/ros/noetic/setup.bash
export PINOCCHIO_PKGCONFIG="${CONDA_PREFIX}/lib/pkgconfig"
source env.sh go2
```

When using conda Pinocchio, `env.sh` only applies the runtime settings needed by Pinocchio/HPP-FCL. Do not globally add the whole conda `lib` directory to `LD_LIBRARY_PATH`, otherwise Gazebo may accidentally load conda versions of libraries such as `libffi` or `libcurl` and exit immediately.

## Launch Simulation

Terminal 1:

```bash
source env.sh go2
roslaunch legged_robot_description empty_world.launch
```

This launch file generates `tmp/legged_control/<robot_type>.urdf`, starts Gazebo, and loads the robot model.

Terminal 2:

```bash
source env.sh go2
roslaunch legged_controllers keyboard_control.launch \
  enable_amp_logging:=true \
  amp_log_dir:=$(pwd)/amp_data
```

After startup, press `i` first. When the controller terminal prints `Initial policy has been received.`, let the robot stabilize in `stance` for a few seconds, then press `1` to switch to `trot`.
