# 新四足机器人接入

满足四足、12 个驱动关节、4 个足端接触点、1 个 IMU 的机器人，通常可以通过 URDF/xacro 和配置文件接入。

## 推荐方式：复用通用 xacro

新增文件：

1. `src/legged_robot_description/urdf/<robot_type>/const.xacro`
2. `src/legged_controllers/config/<robot_type>/task.info`
3. `src/legged_controllers/config/<robot_type>/reference.info`
4. `src/legged_controllers/config/<robot_type>/gait.info`
5. 如有专用 mesh，放到 `src/legged_robot_description/meshes/<robot_type>/`

接入要点：

1. 复用 `src/legged_robot_description/urdf/robot.xacro` 和 `common/leg.xacro`。
2. 在 `<robot_type>/const.xacro` 配置尺寸、质量、惯量、mesh 文件名和关节轴。
3. 原始 URDF 关节轴方向不同时，设置 `haa_axis_x`、`hfe_axis_y`、`kfe_axis_y`。例如 Lite3 使用 `-1, -1, -1`。
4. 通过 `trunk_mesh_file`、`hip_mesh_file`、`thigh_mesh_file`、`thigh_mirror_mesh_file`、`calf_mesh_file` 配置 mesh。
5. 右侧腿 mesh 如需镜像，用 `thigh_mirror_mesh_scale` 配置。

生成后的 URDF 会位于 `tmp/legged_control/<robot_type>.urdf`。

## 直接使用原始 URDF

原始 URDF 至少要满足：

1. 有 `base` 或等价 floating base link，并能被 Pinocchio 正确建模。
2. 有 12 个驱动 revolute joints，并在 `task.info` 的 `jointNames` 里按控制顺序完整列出。
3. 有 4 个足端 link，并在 `task.info` 的 `contactNames3DoF` 里按 OCS2 gait 顺序完整列出。
4. 有 ros_control `transmission`，否则 Gazebo/硬件接口找不到可控关节。
5. 有 Gazebo foot contact sensor 配置，控制器才能读到脚端接触状态。
6. 有 IMU link/sensor。默认 IMU 名是 `imu`，如果不同，需要同步修改控制器参数。
7. mesh 路径能被 ROS/Gazebo 解析，建议放在 `src/legged_robot_description/meshes/<robot_type>/`。
8. `reference.info` 的 `defaultJointState` 和 `task.info` 的 `initialState` 必须与 `jointNames` 顺序一致。

原始命名可以保留，例如 `FL_HipX_joint`、`FR_Knee_joint`、`HL_FOOT`，但必须在 `task.info` 和所有站姿配置里使用真实名字。

## 顺序要求

`task.info` 里最容易出错的是顺序：

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

注意：`jointNames` 和 `contactNames3DoF` 不是同一个顺序概念。本项目关节向量顺序是 `LF, LH, RF, RH`，OCS2 gait 接触相位顺序是 `LF, RF, LH, RH`。混用这两个顺序时，常见现象是 stance 勉强正常，一切到 trot 就摔。

## 配置检查

`reference.info`：

1. `comHeight` 先按腿长估一个保守高度，站稳后再调。
2. `defaultJointState` 顺序必须和 `jointNames` 完全一致。
3. `initialModeSchedule` 通常先保持 `STANCE`。

`gait.info` 至少包含 `stance` 和要使用的 gait。为了避免启动 warning，建议保留现有配置里的 gait 名称；不用的 gait 可以先复制一个保守模板。

Gazebo 和控制器：

1. URDF 里的 12 个 actuated joints 名字和 `jointNames` 完全一致。
2. 4 个足端 link 名字和 `contactNames3DoF` 完全一致。
3. IMU 名字默认是 `imu`，如果不同，需要在 launch 或控制器参数里同步修改。
4. `selfCollision` block 里要有 `enabled false` 或 `enabled true`。
5. 新机器人第一次启动建议把 `recompileLibrariesCppAd` 设为 `true`，模型稳定后再改成 `false` 加快启动。

## 验证顺序

1. `roslaunch legged_robot_description empty_world.launch`，确认模型能生成和加载。
2. 启动 controller 后只进 `stance`，观察四足是否同时支撑、关节弯曲方向是否正确。
3. stance 稳定后再切 `trot`，先零速度踏步，再缓慢加前进速度。
4. 如果 stance 就摔，优先查 joint 顺序、关节轴方向、默认站姿和足端接触名。
5. 如果 stance 稳但 trot 摔，优先查 `contactNames3DoF` 顺序、gait 周期、swing height、摩擦和 WBC/PD 参数。

最后补齐 `<robot_type>` 对应的 `urdf/<robot_type>/` 和 `config/<robot_type>/`。`env.sh` 可以直接接收新的 robot type 参数。
