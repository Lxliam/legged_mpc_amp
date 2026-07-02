# New Quadruped Robot Setup

Robots with 4 legs, 12 actuated joints, 4 foot contact points, and 1 IMU can usually be integrated through URDF/xacro files and controller configuration.

## Recommended Approach: Reuse the Generic xacro

Add these files:

1. `src/legged_robot_description/urdf/<robot_type>/const.xacro`
2. `src/legged_controllers/config/<robot_type>/task.info`
3. `src/legged_controllers/config/<robot_type>/reference.info`
4. `src/legged_controllers/config/<robot_type>/gait.info`
5. If the robot has dedicated meshes, put them under `src/legged_robot_description/meshes/<robot_type>/`

Integration notes:

1. Reuse `src/legged_robot_description/urdf/robot.xacro` and `common/leg.xacro`.
2. Configure dimensions, mass, inertia, mesh filenames, and joint axes in `<robot_type>/const.xacro`.
3. If the original URDF uses different joint axis directions, set `haa_axis_x`, `hfe_axis_y`, and `kfe_axis_y`. For example, Lite3 uses `-1, -1, -1`.
4. Configure meshes through `trunk_mesh_file`, `hip_mesh_file`, `thigh_mesh_file`, `thigh_mirror_mesh_file`, and `calf_mesh_file`.
5. If right-side leg meshes need mirroring, configure `thigh_mirror_mesh_scale`.

The generated URDF is written to `tmp/legged_control/<robot_type>.urdf`.

## Use the Original URDF Directly

The original URDF must satisfy at least the following:

1. It has a `base` link or an equivalent floating base link that Pinocchio can model correctly.
2. It has 12 actuated revolute joints, fully listed in control order under `jointNames` in `task.info`.
3. It has 4 foot links, fully listed in OCS2 gait order under `contactNames3DoF` in `task.info`.
4. It has ros_control `transmission` entries, otherwise Gazebo or the hardware interface cannot find controllable joints.
5. It has Gazebo foot contact sensor configuration so the controller can read foot contact states.
6. It has an IMU link/sensor. The default IMU name is `imu`; if yours differs, update the controller parameters accordingly.
7. Mesh paths must be resolvable by ROS/Gazebo. Placing them under `src/legged_robot_description/meshes/<robot_type>/` is recommended.
8. `defaultJointState` in `reference.info` and `initialState` in `task.info` must match the `jointNames` order.

Original names such as `FL_HipX_joint`, `FR_Knee_joint`, or `HL_FOOT` can be kept, but the exact names must be used in `task.info` and all standing posture configuration.

## Ordering Requirements

The most error-prone part of `task.info` is ordering:

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

`jointNames` and `contactNames3DoF` are different ordering concepts. This project uses `LF, LH, RF, RH` for the joint vector, while OCS2 gait contact phases use `LF, RF, LH, RH`. Mixing these orders commonly makes the robot stand roughly in `stance` but fall as soon as it switches to `trot`.

## Configuration Checklist

`reference.info`:

1. Set `comHeight` to a conservative estimate based on leg length first, then tune it after the robot can stand.
2. Make `defaultJointState` match `jointNames` exactly.
3. Start `initialModeSchedule` with `STANCE` in most cases.

`gait.info` must contain at least `stance` and the gait you want to use. To avoid startup warnings, keep existing gait names when possible; unused gaits can initially be copied from a conservative template.

Gazebo and controller checks:

1. The 12 actuated joint names in the URDF exactly match `jointNames`.
2. The 4 foot link names exactly match `contactNames3DoF`.
3. The default IMU name is `imu`; if yours differs, update the launch file or controller parameters.
4. The `selfCollision` block contains either `enabled false` or `enabled true`.
5. For a new robot, set `recompileLibrariesCppAd` to `true` on the first startup. After the model is stable, switch it to `false` to speed up startup.

## Validation Order

1. Run `roslaunch legged_robot_description empty_world.launch` and confirm that the model is generated and loaded.
2. Start the controller and enter only `stance`; check whether all four legs support the robot and whether joint bending directions are correct.
3. After `stance` is stable, switch to `trot` at zero velocity first, then slowly increase forward velocity.
4. If the robot falls in `stance`, check joint order, joint axis directions, default standing posture, and foot contact names first.
5. If `stance` is stable but `trot` fails, check `contactNames3DoF` order, gait timing, swing height, friction, and WBC/PD parameters first.

Finally, add the completed `urdf/<robot_type>/` and `config/<robot_type>/` files. `env.sh` can already accept the new robot type as an argument.
