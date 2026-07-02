# legged_mpc_amp

[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](LICENSE)

A general-purpose AMP data generator for quadruped robots based on NMPC + WBC. It wraps Gazebo keyboard control, fully automatic AMP recording, and multi-robot integration interfaces.

## Demo

![Lite3 standard gait walk](video/lite3satndwalk.gif)

## Quick Start

```bash
# 1. Install system dependencies (Ubuntu 20.04 + ROS Noetic)
sudo apt update
sudo apt install python3-catkin-tools python3-rosdep libeigen3-dev \
  libboost-all-dev liburdfdom-dev libpinocchio-dev libhpp-fcl-dev

# 2. Build the workspace
cd /path/to/legged_mpc_amp
source /opt/ros/noetic/setup.bash
bash setup.sh
```

## Launch Simulation

```bash
# Terminal 1: start Gazebo
source env.sh go2
roslaunch legged_robot_description empty_world.launch

# Terminal 2: start keyboard control + AMP recording
source env.sh go2
roslaunch legged_controllers keyboard_control.launch \
  enable_amp_logging:=true \
  amp_log_dir:=$(pwd)/amp_data
```

After startup, press `i` to initialize, press `1` to switch to `trot`, and use `w/s` to move forward/backward.

## Features

- **Automatic AMP recording**: collect 1-2 minutes of diverse motion data with one command
- **IsaacLab export**: `convert_amp_data_isaaclab.py` exports `.npz` motion files directly
- **Foot trajectory visualization**: inspect MPC-optimized foot trajectories in real time with `foot_trajectory_plotter.py`
- **Multi-robot support**: Go1/Go2/A1/Aliengo/Lite3; see the [new robot setup guide](docs/new_robot_setup_EN.md)

## Keyboard Shortcuts

| Key | Action |
|-----|--------|
| `i` | Initialize the controller |
| `1` | Switch to trot gait |
| `w/s` | Increase/decrease forward velocity |
| `a/d` | Increase/decrease yaw velocity |
| `l` | Start/stop AMP recording |
| `0` | Switch to stance |

## Documentation

- [Build and environment guide](docs/build_guide_EN.md)
- [New quadruped robot setup](docs/new_robot_setup_EN.md)
- [AMP data collection and conversion](docs/amp_data_guide_EN.md)

## Acknowledgements

This project is built on [QiayuanLiao/legged_control](https://github.com/qiayuanl/legged_control.git). Thanks to the original author for the excellent work. The modifications and wrappers follow the BSD 3-Clause license.

## Support

If this project helps you, a Star would be appreciated. Issues are welcome.
