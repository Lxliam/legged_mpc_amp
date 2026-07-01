# legged_mpc_amp Workflow

`legged_mpc_amp` controls quadruped robots with an NMPC-WBC stack and can record motion data for AMP training.


## Build

This repository vendors OCS2 and `ocs2_robotic_assets` under `src/third_party/`, so they are built as part of this catkin workspace. You do not need to maintain a separate `~/ocs2_catkin_ws`.

Vendored third-party packages:

```text
src/third_party/ocs2
src/third_party/ocs2_robotic_assets
src/qpoases_catkin
```

OCS2 RaiSim, MPCNet, documentation, and unrelated example packages are skipped through `CATKIN_IGNORE` files to avoid unnecessary dependencies. Pinocchio, HPP-FCL, Gazebo, ROS control, and related packages are still installed as system dependencies.

Install the basic dependencies first. The following example targets Ubuntu/Debian with ROS Noetic. For other ROS 1 distributions or systems, install the equivalent packages.

```bash
sudo apt update
sudo apt install \
  python3-catkin-tools python3-rosdep \
  libeigen3-dev libboost-all-dev liburdfdom-dev \
  libpinocchio-dev libhpp-fcl-dev

cd /path/to/legged_mpc_amp
source /opt/ros/noetic/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

If Pinocchio is installed from conda or a custom prefix, make sure `pkg-config` can find it. For the active conda environment:

```bash
export PINOCCHIO_PKGCONFIG="${CONDA_PREFIX}/lib/pkgconfig"
```

If you use the default system package `libpinocchio-dev`, this variable is usually not needed.

Then build the workspace:

```bash
cd /path/to/legged_mpc_amp
bash setup.sh
```

`setup.sh` automatically looks for the ROS environment matching `${ROS_DISTRO}` and defaults to `noetic`. You can explicitly override ROS, Pinocchio, and catkin build arguments:

```bash
ROS_SETUP=/opt/ros/noetic/setup.bash \
PINOCCHIO_PKGCONFIG="${CONDA_PREFIX}/lib/pkgconfig" \
CATKIN_BUILD_ARGS="-DCMAKE_BUILD_TYPE=RelWithDebInfo" \
bash setup.sh
```

If Pinocchio comes from system packages, omit `PINOCCHIO_PKGCONFIG`. If your machine does not use Noetic, point `ROS_SETUP` to the matching ROS 1 `setup.bash`.

The default robot is Go2. In every new terminal, use the project environment script to initialize ROS, Pinocchio, the workspace, and the robot type:

```bash
cd /path/to/legged_mpc_amp
source env.sh go2
```

Supported robot types depend on `src/legged_controllers/config/` and `src/legged_robot_description/urdf/`. This repository currently includes:

```text
a1, aliengo, go1, go2, Lite3
```

## Source the Environment in Every Terminal

Run this first in every new terminal used by Gazebo, the controller, or command publishing:

```bash
cd /path/to/legged_mpc_amp
source env.sh go2
```

This step is important. When this project uses conda Pinocchio, `env.sh` sets `PKG_CONFIG_PATH` and uses `LD_PRELOAD` only for the small set of conda libraries needed by Pinocchio/HPP-FCL at runtime. Do not globally add the whole conda `lib` directory to `LD_LIBRARY_PATH`, because Gazebo may accidentally load conda versions of libraries such as `libffi` or `libcurl` and exit immediately.

`env.sh` and `setup.sh` support the same path overrides:

```bash
export ROS_SETUP=/opt/ros/noetic/setup.bash
export PINOCCHIO_PKGCONFIG="${CONDA_PREFIX}/lib/pkgconfig"
source env.sh go2
```

You can also switch to another robot:

```bash
source env.sh a1
source env.sh go1
source env.sh aliengo
source env.sh Lite3
```

## Launch Simulation

Terminal 1:

```bash
roslaunch legged_robot_description empty_world.launch
```

This launch file generates `tmp/legged_control/<robot_type>.urdf`, starts Gazebo, and loads the robot model.

## Keyboard Control

Terminal 2:

```bash
roslaunch legged_controllers keyboard_control.launch \
  enable_amp_logging:=true \
  amp_log_dir:=$(pwd)/amp_data
```

This is the recommended entry point for loading the controller and keyboard console.

Keyboard shortcuts:

```text
i       Start the legged controller, switch to stance, and send zero velocity
k       Stop the legged controller
space   Send zero velocity
w/s     Increase/decrease forward velocity
a/d     Increase/decrease yaw angular velocity
q/e     Increase/decrease lateral velocity
z/x     Decrease/increase velocity step size
r       Reset velocity step size
l       Start/stop AMP recording
0       stance
1       trot
2       standing_trot
?       Show help
```

After launch, press `i` first to initialize. When the controller terminal prints `Initial policy has been received.`, let the robot stabilize in `stance` for a few seconds, then press `1` to switch to `trot`.

After a gait switch, the keyboard console automatically sends zero velocity for about 2 seconds. Wait until this protection window ends, then increase speed gradually with `w`. The default increment is `0.10 m/s`. Forward velocity is limited to `+-1.00 m/s`, lateral velocity to `+-0.50 m/s`, and yaw angular velocity to `+-1.00 rad/s`. `/cmd_vel` is also ramp-limited to avoid sudden velocity target changes that can make the robot fall and trigger safety protection.

After switching gait, the controller terminal should print `[GaitReceiver]: Setting new gait after time ...`. If this line does not appear, the gait was not received by MPC; do not continue increasing velocity.

## Plot Foot Trajectories in Real Time

After Gazebo and the controller are running, open another terminal to inspect the optimized MPC foot trajectories. The script subscribes to `/legged_robot/optimizedStateTrajectory`, extracts the LF, RF, LH, and RH end-effector trajectories from the `EE Trajectories` marker, and displays them with matplotlib.

Terminal 3:

```bash
cd /path/to/legged_mpc_amp
source env.sh go2
rosrun legged_controllers foot_trajectory_plotter.py --view xz
```

Common views:

```bash
rosrun legged_controllers foot_trajectory_plotter.py --view xz   # side view, useful for swing height and leg shape
rosrun legged_controllers foot_trajectory_plotter.py --view xy   # top view, useful for foot placement distribution
rosrun legged_controllers foot_trajectory_plotter.py --view z    # foot height curves only
rosrun legged_controllers foot_trajectory_plotter.py --view 3d   # 3D trajectories
```

By default, the script displays a rolling trail of recent foot positions over the last 10 seconds. To show only the latest MPC horizon:

```bash
rosrun legged_controllers foot_trajectory_plotter.py \
  --view xz \
  --trail-seconds 0
```

If `rosrun` cannot find the script, run it directly from the source tree:

```bash
python3 src/legged_controllers/scripts/foot_trajectory_plotter.py \
  --topic /legged_robot/optimizedStateTrajectory \
  --view xz
```

## Automatically Collect Trot AMP Data

To automatically collect around 1-2 minutes of trot data, run:

```bash
cd /path/to/legged_mpc_amp
bash scripts/collect_trot_amp_data.sh
```

The script starts Gazebo, loads the controller, waits for initialization and the initial policy, keeps the robot in `stance`, then switches gait and enables the AMP logger according to a predefined velocity schedule. Each CSV stores one continuous motion sequence instead of splitting every small velocity segment into a separate file.

By default, it records about 110 seconds of valid motion data covering:

```text
vx: forward and backward linear velocity
vy: left and right lateral velocity
wz: positive and negative yaw angular velocity
vx + vy: diagonal translation
vx + vy + wz: translation combined with turning
```

Fully static segments use `stance` and are not recorded as in-place `trot`. The script uses `trot` only when at least one of `vx`, `vy`, or `wz` is nonzero.

Logs are written to a timestamped subdirectory by default:

```text
amp_data/auto_trot_YYYYmmdd_HHMMSS/
```

`collection_manifest.json` records the sequence associated with each CSV and the `vx/vy/wz/duration` for every segment inside the sequence.

Common parameters can be overridden with environment variables:

```bash
ROBOT_TYPE=go2 \
AMP_LOG_DIR=$(pwd)/amp_data \
GAZEBO_GUI=false \
bash scripts/collect_trot_amp_data.sh
```

## Convert to the IsaacLab Motion Structure

IsaacLab tasks usually require the motion file layout to exactly match the AMP observation layout used by the task. This repository provides a structured export script. By default, it remaps the OCS2 joint and foot order `[LF, LH, RF, RH]` to the common Unitree/IsaacLab order `[FL, FR, RL, RR]`:

```bash
python3 scripts/convert_amp_data_isaaclab.py \
  --input_dir amp_data \
  --output_dir amp_dataset_isaaclab
```

The output contains:

```text
amp_dataset_isaaclab/isaaclab_motions.npz
amp_dataset_isaaclab/isaaclab_motion_metadata.json
amp_dataset_isaaclab/sequences/*.npy
```

The `.npz` file includes structured arrays such as `root_pos`, `root_rot_rpy`, `root_lin_vel_b`, `root_ang_vel_b`, `joint_pos`, `joint_vel`, and `foot_contact`, as well as the concatenated flat array `motions`.

`root_lin_vel_b` and `root_ang_vel_b` are expressed in the base/body frame and correspond to IsaacLab fields such as `asset.data.root_lin_vel_b` and `asset.data.root_ang_vel_b`.

The default flat layout is:

```text
root_pos, root_rot_rpy, root_lin_vel_b, root_ang_vel_b, joint_pos, joint_vel, foot_contact
```

`isaaclab_motions.npz` can still be used for global visualization and validation. During AMP training, do not sample clips across `sequence_lengths` boundaries. If your loader does not handle `sequence_lengths`, prefer importing `sequences/*.npy` directly, because each file is one continuous motion sequence.

If your IsaacLab asset or task uses the original `[LF, LH, RF, RH]` order, add:

```bash
python3 scripts/convert_amp_data_isaaclab.py \
  --input_dir amp_data \
  --output_dir amp_dataset_isaaclab \
  --no_remap_to_isaaclab
```

## Adding a New Quadruped Robot

If the robot has 4 legs, 12 actuated joints, 4 foot contact points, and 1 IMU, it can usually be integrated through URDF/xacro and configuration files.

There are two supported integration approaches.

### Option A: Recommended, Reuse the Generic xacro

Add the following files:

1. `src/legged_robot_description/urdf/<robot_type>/const.xacro`
2. `src/legged_controllers/config/<robot_type>/task.info`
3. `src/legged_controllers/config/<robot_type>/reference.info`
4. `src/legged_controllers/config/<robot_type>/gait.info`
5. If meshes are not in a shared directory, place them under `src/legged_robot_description/meshes/<robot_type>/`

Integration steps:

1. Reuse `src/legged_robot_description/urdf/robot.xacro` and `common/leg.xacro`.
2. Configure dimensions, mass, inertia, mesh filenames, and joint axes in `<robot_type>/const.xacro`.
3. If the original URDF uses different joint axis directions, set `haa_axis_x`, `hfe_axis_y`, and `kfe_axis_y`. For example, Lite3 uses `-1, -1, -1`.
4. Configure mesh filenames through `trunk_mesh_file`, `hip_mesh_file`, `thigh_mesh_file`, `thigh_mirror_mesh_file`, and `calf_mesh_file`.
5. If right-side leg meshes need mirroring, configure `thigh_mirror_mesh_scale`.
6. The generated URDF is written to `tmp/legged_control/<robot_type>.urdf`.

### Option B: Use the Original URDF Directly

The raw URDF must satisfy at least the following:

1. It has a `base` link or an equivalent floating base link that Pinocchio can model correctly.
2. It has 12 actuated revolute joints, all listed in control order in `task.info` under `jointNames`.
3. It has 4 foot links, all listed in OCS2 gait order in `task.info` under `contactNames3DoF`.
4. It has ros_control `transmission` entries, otherwise Gazebo or the hardware interface cannot find controllable joints.
5. It has Gazebo foot contact sensor configuration so the controller can read foot contact states.
6. It has an IMU link/sensor. The default IMU name is `imu`; if you use another name, update the controller parameters accordingly.
7. Mesh paths must be resolvable by ROS/Gazebo. Placing meshes under `src/legged_robot_description/meshes/<robot_type>/` is recommended.
8. `defaultJointState` in `reference.info` and `initialState` in `task.info` must match the `jointNames` order exactly.

If the original URDF uses its own names, such as `FL_HipX_joint`, `FR_Knee_joint`, or `HL_FOOT`, you can keep those names. They must be written exactly in `task.info`, and all standing posture configuration must use the same order.

The most important part of `task.info` is ordering:

```text
model_settings
{
  recompileLibrariesCppAd       true
  modelFolderCppAd              tmp/legged_control/<robot_type>

  jointNames
  {
    [0] LF_HAA
    [1] LF_HFE
    [2] LF_KFE
    [3] LH_HAA
    [4] LH_HFE
    [5] LH_KFE
    [6] RF_HAA
    [7] RF_HFE
    [8] RF_KFE
    [9] RH_HAA
    [10] RH_HFE
    [11] RH_KFE
  }

  contactNames3DoF
  {
    [0] LF_FOOT
    [1] RF_FOOT
    [2] LH_FOOT
    [3] RH_FOOT
  }
}
```

`jointNames` and `contactNames3DoF` are not the same ordering concept. In this project, the joint vector order is `LF, LH, RF, RH`, while the OCS2 gait contact phase order is `LF, RF, LH, RH`. Mixing these two orders commonly causes the robot to stand roughly in place but fall immediately when switching to `trot`, or to produce clearly wrong control force directions.

Configure the following in `reference.info`:

1. `comHeight`: start with a conservative estimate based on leg length, then tune after the robot can stand.
2. `defaultJointState`: the order must exactly match `jointNames`.
3. `initialModeSchedule`: usually start with `STANCE`.

`gait.info` must contain at least `stance` and the gait you want to use. To avoid startup warnings, keep the existing gait names from the current configurations when possible. Unused gaits can initially be copied from a conservative template.

Also verify Gazebo and controller configuration:

1. The 12 actuated joint names in the URDF exactly match `jointNames`.
2. The 4 foot link names exactly match `contactNames3DoF`.
3. The IMU name is `imu` by default. If it differs, update the launch file or controller parameters.
4. The `selfCollision` block contains either `enabled false` or `enabled true`.
5. For a new robot, set `recompileLibrariesCppAd` to `true` the first time. After the model is stable, change it to `false` to speed up startup.

Recommended validation order:

1. Run `roslaunch legged_robot_description empty_world.launch` and confirm the model is generated and loaded.
2. Start the controller and enter only `stance`; check that all four legs support the robot and the joint bending directions are correct.
3. After `stance` is stable, switch to `trot` with zero velocity first, then slowly increase forward velocity.
4. If the robot falls in `stance`, check joint order, joint axis directions, the default standing pose, and foot contact names first.
5. If `stance` is stable but `trot` fails, check `contactNames3DoF` order, gait timing, swing height, friction, and WBC/PD parameters first.

Finally, add the completed `urdf/<robot_type>/` and `config/<robot_type>/` files, and record the new robot in the README robot list. `env.sh` can already accept the new robot type as an argument.
