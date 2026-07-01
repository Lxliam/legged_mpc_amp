# legged_mpc_amp 使用流程

`legged_mpc_amp` 是一个用 NMPC-WBC 控制四足机器人，并可记录 AMP 训练用的运动数据的工具。

# 演示视频
![Lite3 robot standard gait walk](video/lite3satndwalk.gif)

## 编译

本仓库把 OCS2 和 `ocs2_robotic_assets` 放在 `src/third_party/` 下，作为当前 catkin workspace 的第三方源码一起编译。不再需要单独维护 `~/ocs2_catkin_ws`。

当前 vendored 第三方包：

```text
src/third_party/ocs2
src/third_party/ocs2_robotic_assets
src/qpoases_catkin
```

OCS2 的 RaiSim、MPCNet、文档和无关示例包已通过 `CATKIN_IGNORE` 跳过，避免额外拉入不需要的依赖。Pinocchio、HPP-FCL、Gazebo、ROS controller 等仍作为系统依赖安装。

首次使用先安装基础依赖。下面是 Ubuntu/Debian + ROS Noetic 的例子，其他 ROS 1 发行版或系统需要安装等价包：

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

如果 Pinocchio 来自 conda 或自定义安装路径，需要让 `pkg-config` 能找到它。使用当前 conda 环境时：

```bash
export PINOCCHIO_PKGCONFIG="${CONDA_PREFIX}/lib/pkgconfig"
```

如果使用默认系统包安装的 `libpinocchio-dev`，通常不需要设置这个变量。

然后编译：

```bash
cd /path/to/legged_mpc_amp
bash setup.sh
```

`setup.sh` 会自动查找 `${ROS_DISTRO}` 对应的 ROS 环境，默认是 `noetic`。在不同机器上也可以显式指定 ROS、Pinocchio 和 catkin build 参数：

```bash
ROS_SETUP=/opt/ros/noetic/setup.bash \
PINOCCHIO_PKGCONFIG="${CONDA_PREFIX}/lib/pkgconfig" \
CATKIN_BUILD_ARGS="-DCMAKE_BUILD_TYPE=RelWithDebInfo" \
bash setup.sh
```

如果 Pinocchio 使用系统包，上面可以不传 `PINOCCHIO_PKGCONFIG`；如果机器上不是 Noetic，把 `ROS_SETUP` 改成对应 ROS 1 发行版的 `setup.bash`。

默认使用 Go2。每个新终端可以用项目环境脚本一次性完成 ROS、Pinocchio、workspace 和 robot type 初始化：

```bash
cd /path/to/legged_mpc_amp
source env.sh go2
```

支持的 robot type 取决于 `src/legged_controllers/config/` 和 `src/legged_robot_description/urdf/`，当前包含：

```text
a1, aliengo, go1, go2, Lite3
```

## 每个终端都先 source

后面的每个新终端都先执行：

```bash
cd /path/to/legged_mpc_amp
source env.sh go2
```

也可以切换其他机器人：

```bash
source env.sh a1
source env.sh go1
source env.sh aliengo
source env.sh Lite3
```

## 启动仿真

终端 1：

```bash
roslaunch legged_robot_description empty_world.launch
```

该 launch 会生成 `tmp/legged_control/<robot_type>.urdf`，启动 Gazebo，并加载机器人模型。

## 键盘控制台

终端 2：

```bash
roslaunch legged_controllers keyboard_control.launch \
  enable_amp_logging:=true \
  amp_log_dir:=$(pwd)/amp_data
```

推荐使用这个入口加载 controller 和键盘控制台。控制台快捷键：

```text
i       启动 legged controller，切到 stance，并发送零速度
k       停止 legged controller
space   零速度
w/s     增加/减小前进速度
a/d     增加/减小转向角速度
q/e     增加/减小侧向速度
z/x     减小/增大速度步长
r       重置速度步长
l       开始/结束 AMP 录制
0       stance
1       trot
2       standing_trot
?       显示帮助
```

启动后先按 `i` 初始化。看到控制器终端打印 `Initial policy has been received.` 后，先让机器人在 `stance` 稳住几秒，再按 `1` 切到 `trot`。切 gait 后键盘控制台会自动清零速度并保持约 2 秒，等保护时间结束后再一点一点按 `w` 加速。默认每次增加 `0.10 m/s`，前进速度限幅为 `+-1.00 m/s`，横向速度限幅为 `+-0.50 m/s`，转向角速度限幅为 `+-1.00 rad/s`，并且 `/cmd_vel` 会做斜坡限幅，避免速度目标突变导致机器人趴下触发 safety。切 gait 后控制器终端应该打印 `[GaitReceiver]: Setting new gait after time ...`，如果没有这行，说明 gait 没有被 MPC 接收，不要继续加速度。

## 实时查看足端轨迹曲线

启动 Gazebo 和 controller 后，可以另开一个终端实时查看 MPC 优化出的四足足端轨迹。这个脚本订阅 `/legged_robot/optimizedStateTrajectory`，从其中的 `EE Trajectories` marker 提取 LF、RF、LH、RH 四条足端轨迹并用 matplotlib 显示。

终端 3：

```bash
cd /path/to/legged_mpc_amp
source env.sh go2
rosrun legged_controllers foot_trajectory_plotter.py --view xz
```

常用视图：

```bash
rosrun legged_controllers foot_trajectory_plotter.py --view xz   # 侧视图，最适合看抬脚高度和摆腿形状
rosrun legged_controllers foot_trajectory_plotter.py --view xy   # 俯视图，适合看落脚点前后/左右分布
rosrun legged_controllers foot_trajectory_plotter.py --view z    # 只看足端高度曲线
rosrun legged_controllers foot_trajectory_plotter.py --view 3d   # 三维轨迹
```

默认显示最近 10 秒足端当前位置形成的滚动轨迹。如果只想看最新 MPC horizon，可以设置：

```bash
rosrun legged_controllers foot_trajectory_plotter.py \
  --view xz \
  --trail-seconds 0
```

如果 `rosrun` 找不到脚本，也可以直接运行源码里的 Python 脚本：

```bash
python3 src/legged_controllers/scripts/foot_trajectory_plotter.py \
  --topic /legged_robot/optimizedStateTrajectory \
  --view xz
```

### 自动采集 trot AMP 数据

如果要自动采集一段 1-2 分钟左右的 trot 数据，可以直接运行：

```bash
cd /path/to/legged_mpc_amp
bash scripts/collect_trot_amp_data.sh
```

脚本会启动 Gazebo、加载 controller，等待 init/initial policy 完成后先保持 `stance`，然后按预设速度序列自动切换 gait 并打开 AMP logger。默认每个 CSV 保存一组连续动作片段，不再按每个小速度段单独切文件。默认采集约 110 秒有效 motion 数据，覆盖：

```text
vx: 正向和反向线速度
vy: 左右两个方向的侧向线速度
wz: 正向和反向 yaw 角速度
vx + vy: 对角方向平移
vx + vy + wz: 平移叠加转向
```

注意：完全静止段使用 `stance`，不会录成 `trot` 原地踏步；只要 `vx/vy/wz` 任意一个非零，才使用 `trot`。

日志默认写到带时间戳的子目录，例如：

```text
amp_data/auto_trot_YYYYmmdd_HHMMSS/
```

其中 `collection_manifest.json` 记录每个 CSV 文件对应的 sequence，以及 sequence 内每个动作片段的 `vx/vy/wz/duration`。常用参数可以通过环境变量覆盖：

```bash
ROBOT_TYPE=go2 \
AMP_LOG_DIR=$(pwd)/amp_data \
GAZEBO_GUI=false \
bash scripts/collect_trot_amp_data.sh
```

### 转成 IsaacLab motion 结构

IsaacLab 侧通常要让 motion 文件的字段顺序和 task 里的 AMP observation 完全一致。仓库提供了一个结构化导出脚本，默认会把本项目 OCS2 关节/足端顺序 `[LF, LH, RF, RH]` 重排为 Unitree/IsaacLab 常见的 `[FL, FR, RL, RR]`：

```bash
python3 scripts/convert_amp_data_isaaclab.py \
  --input_dir amp_data \
  --output_dir amp_dataset_isaaclab
```

输出包含：

```text
amp_dataset_isaaclab/isaaclab_motions.npz
amp_dataset_isaaclab/isaaclab_motion_metadata.json
amp_dataset_isaaclab/sequences/*.npy
```

`.npz` 里既有结构化数组，例如 `root_pos`、`root_rot_rpy`、`root_lin_vel_b`、`root_ang_vel_b`、`joint_pos`、`joint_vel`、`foot_contact`，也有拼好的 `motions` flat array。`root_lin_vel_b` 和 `root_ang_vel_b` 都是 base/body 坐标系下的速度，对应 IsaacLab 里的 `asset.data.root_lin_vel_b` 和 `asset.data.root_ang_vel_b`。默认 flat layout 是：

```text
root_pos, root_rot_rpy, root_lin_vel_b, root_ang_vel_b, joint_pos, joint_vel, foot_contact
```

`isaaclab_motions.npz` 可以继续用于整体可视化验证；实际 AMP 训练采样时不要跨 `sequence_lengths` 边界取片段。如果你的 loader 不处理 `sequence_lengths`，优先直接导入 `sequences/*.npy`，每个文件就是一个连续 motion sequence。

如果 IsaacLab 资产或 task 使用原始 `[LF, LH, RF, RH]` 顺序，可以加：

```bash
python3 scripts/convert_amp_data_isaaclab.py \
  --input_dir amp_data \
  --output_dir amp_dataset_isaaclab \
  --no_remap_to_isaaclab
```

## 新四足机器人接入

只要机器人满足四足、12 个驱动关节、4 个足端接触点、1 个 IMU，通常可以通过 URDF/xacro 和配置文件接入。

有两种接入方式。

### 方式 A：推荐，复用通用 xacro

新增这些文件：

1. `src/legged_robot_description/urdf/<robot_type>/const.xacro`
2. `src/legged_controllers/config/<robot_type>/task.info`
3. `src/legged_controllers/config/<robot_type>/reference.info`
4. `src/legged_controllers/config/<robot_type>/gait.info`
5. 如果 mesh 不在通用目录下，放到 `src/legged_robot_description/meshes/<robot_type>/`

接入步骤：

1. 复用 `src/legged_robot_description/urdf/robot.xacro` 和 `common/leg.xacro`。
2. 在 `<robot_type>/const.xacro` 里配置尺寸、质量、惯量、mesh 文件名和关节轴。
3. 如果原始 URDF 的关节轴方向不同，设置 `haa_axis_x`、`hfe_axis_y`、`kfe_axis_y`。例如 Lite3 使用 `-1, -1, -1`。
4. mesh 文件名通过 `trunk_mesh_file`、`hip_mesh_file`、`thigh_mesh_file`、`thigh_mirror_mesh_file`、`calf_mesh_file` 配置。
5. 右侧腿 mesh 如果需要镜像，用 `thigh_mirror_mesh_scale` 配置。
6. 生成后的 URDF 会放在 `tmp/legged_control/<robot_type>.urdf`。

### 方式 B：直接使用原始 URDF

直接 URDF 至少要满足：

1. 有 `base` 或等价的 floating base link，并能被 Pinocchio 正确建模。
2. 有 12 个驱动 revolute joints，并在 `task.info` 的 `jointNames` 里按控制顺序完整列出。
3. 有 4 个足端 link，并在 `task.info` 的 `contactNames3DoF` 里按 OCS2 gait 顺序完整列出。
4. 有 ros_control `transmission`，否则 Gazebo/硬件接口找不到可控关节。
5. 有 Gazebo foot contact sensor 配置，控制器才能读到脚端接触状态。
6. 有 IMU link/sensor。默认 IMU 名是 `imu`，如果使用其他名字，需要同步改控制器参数。
7. mesh 路径必须能被 ROS/Gazebo 解析，建议放在 `src/legged_robot_description/meshes/<robot_type>/`。
8. `reference.info` 里的 `defaultJointState` 和 `task.info` 里的 `initialState` 必须和 `jointNames` 顺序一致。

如果原始 URDF 使用自己的命名，例如 `FL_HipX_joint`、`FR_Knee_joint`、`HL_FOOT`，可以保留这些名字，但必须在 `task.info` 里写真实名字，并同步所有站姿配置。

`task.info` 里最重要的是顺序：

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

注意：`jointNames` 和 `contactNames3DoF` 的顺序不是同一个概念。当前工程的关节向量顺序是 `LF, LH, RF, RH`，而 OCS2 gait 的接触相位顺序是 `LF, RF, LH, RH`。如果把这两个顺序混用，常见现象是 stance 勉强正常，一切到 trot 就摔，或者站起来控制力方向明显不对。

`reference.info` 里需要配置：

1. `comHeight`，先按腿长估一个保守高度，机器人能稳住后再调。
2. `defaultJointState`，顺序必须和 `jointNames` 完全一致。
3. `initialModeSchedule` 通常先保持 `STANCE`。

`gait.info` 里至少要有 `stance` 和要使用的 gait。为了避免启动时 warning，建议保留现有配置里的 gait 名称；不用的 gait 也可以先复制一个保守模板。

Gazebo 和控制器还要确认：

1. URDF 里 12 个 actuated joints 名字和 `jointNames` 完全一致。
2. 4 个足端 link 名字和 `contactNames3DoF` 完全一致。
3. IMU 名字默认是 `imu`，如果不同，需要在 launch 或控制器参数里同步修改。
4. `selfCollision` block 里要有 `enabled false` 或 `enabled true`。
5. 新机器人第一次启动建议把 `recompileLibrariesCppAd` 设为 `true`，模型稳定后再改成 `false` 加快启动。

推荐验证顺序：

1. `roslaunch legged_robot_description empty_world.launch`，先确认模型能生成和加载。
2. 启动 controller 后只进 `stance`，观察四足是否同时支撑、关节弯曲方向是否正确。
3. stance 稳定后再切 `trot`，先零速度踏步，再缓慢加前进速度。
4. 如果 stance 就摔，优先查 joint 顺序、关节轴方向、默认站姿和足端接触名。
5. 如果 stance 稳但 trot 摔，优先查 `contactNames3DoF` 顺序、gait 周期、swing height、摩擦和 WBC/PD 参数。

最后把 `<robot_type>` 对应的 `urdf/<robot_type>/` 和 `config/<robot_type>/` 补齐，并在 README 的 robot 列表里记录即可；`env.sh` 可以直接接收新的 robot type 参数。
