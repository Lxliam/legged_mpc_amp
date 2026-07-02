# AMP Data Collection and Conversion

This project supports keyboard recording, automatic trot data collection, real-time foot trajectory visualization, and export to the motion structure commonly used by IsaacLab.

## Keyboard Control and Manual Recording

Start Gazebo and the controller:

```bash
# Terminal 1
source env.sh go2
roslaunch legged_robot_description empty_world.launch

# Terminal 2
source env.sh go2
roslaunch legged_controllers keyboard_control.launch \
  enable_amp_logging:=true \
  amp_log_dir:=$(pwd)/amp_data
```

Common shortcuts:

| Key | Action |
|-----|--------|
| `i` | Start the legged controller, switch to stance, and send zero velocity |
| `k` | Stop the legged controller |
| `space` | Send zero velocity |
| `w/s` | Increase/decrease forward velocity |
| `a/d` | Increase/decrease yaw angular velocity |
| `q/e` | Increase/decrease lateral velocity |
| `z/x` | Decrease/increase velocity step size |
| `r` | Reset velocity step size |
| `l` | Start/stop AMP recording |
| `0` | stance |
| `1` | trot |
| `2` | standing_trot |
| `?` | Show help |

Recommended flow:

1. Press `i` to initialize.
2. Wait until the controller terminal prints `Initial policy has been received.`.
3. Let the robot stabilize in `stance` for a few seconds.
4. Press `1` to switch to `trot`.
5. Wait for the roughly 2-second protection window to finish, then gradually increase velocity with `w/s/a/d/q/e`.

The default velocity step is `0.10 m/s`. Forward velocity is limited to `+-1.00 m/s`, lateral velocity to `+-0.50 m/s`, and yaw angular velocity to `+-1.00 rad/s`. After switching gait, the controller terminal should print `[GaitReceiver]: Setting new gait after time ...`. If this line does not appear, the gait was not received by MPC, so do not continue increasing velocity.

## Automatic Trot Data Collection

Collect around 1-2 minutes of trot data:

```bash
cd /path/to/legged_mpc_amp
bash scripts/collect_trot_amp_data.sh
```

The script starts Gazebo, loads the controller, waits for initialization and the initial policy, then enables the AMP logger according to a predefined velocity schedule. By default, each CSV stores one continuous motion sequence, and logs are written to:

```text
amp_data/auto_trot_YYYYmmdd_HHMMSS/
```

`collection_manifest.json` records the sequence associated with each CSV and the `vx/vy/wz/duration` for every segment inside the sequence. Common environment variables:

```bash
ROBOT_TYPE=go2 \
AMP_LOG_DIR=$(pwd)/amp_data \
GAZEBO_GUI=false \
bash scripts/collect_trot_amp_data.sh
```

The default data schedule covers forward/backward motion, left/right lateral motion, positive/negative yaw, diagonal translation, and translation combined with turning. Fully static segments use `stance` and are not recorded as in-place `trot`.

## Convert to IsaacLab Motion

Export structured IsaacLab motion data:

```bash
python3 scripts/convert_amp_data_isaaclab.py \
  --input_dir amp_data \
  --output_dir amp_dataset_isaaclab
```

Output files:

```text
amp_dataset_isaaclab/isaaclab_motions.npz
amp_dataset_isaaclab/isaaclab_motion_metadata.json
amp_dataset_isaaclab/sequences/*.npy
```

The `.npz` contains `root_pos`, `root_rot_rpy`, `root_lin_vel_b`, `root_ang_vel_b`, `joint_pos`, `joint_vel`, `foot_contact`, and the concatenated flat array `motions`. The default flat layout is:

```text
root_pos, root_rot_rpy, root_lin_vel_b, root_ang_vel_b, joint_pos, joint_vel, foot_contact
```

By default, the script remaps this project's OCS2 joint/foot order `[LF, LH, RF, RH]` to the common Unitree/IsaacLab order `[FL, FR, RL, RR]`. If your IsaacLab asset or task uses the original order:

```bash
python3 scripts/convert_amp_data_isaaclab.py \
  --input_dir amp_data \
  --output_dir amp_dataset_isaaclab \
  --no_remap_to_isaaclab
```

During training, do not sample clips across `sequence_lengths` boundaries. If your loader does not handle `sequence_lengths`, prefer importing `sequences/*.npy` directly; each file is one continuous motion sequence.

## Foot Trajectory Visualization

After Gazebo and the controller are running, inspect MPC-optimized foot trajectories in real time:

```bash
source env.sh go2
rosrun legged_controllers foot_trajectory_plotter.py --view xz
```

Common views:

```bash
rosrun legged_controllers foot_trajectory_plotter.py --view xz   # side view
rosrun legged_controllers foot_trajectory_plotter.py --view xy   # top view
rosrun legged_controllers foot_trajectory_plotter.py --view z    # foot height curves
rosrun legged_controllers foot_trajectory_plotter.py --view 3d   # 3D trajectories
```

Show only the latest MPC horizon:

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
