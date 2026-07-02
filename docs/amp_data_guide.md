# AMP 数据采集与转换

本项目支持键盘录制、自动采集 trot 数据、实时查看足端轨迹，并可导出 IsaacLab 常用 motion 结构。

## 键盘控制与手动录制

启动 Gazebo 和控制器：

```bash
# 终端 1
source env.sh go2
roslaunch legged_robot_description empty_world.launch

# 终端 2
source env.sh go2
roslaunch legged_controllers keyboard_control.launch \
  enable_amp_logging:=true \
  amp_log_dir:=$(pwd)/amp_data
```

常用快捷键：

| 按键 | 功能 |
|------|------|
| `i` | 启动 legged controller，切到 stance，并发送零速度 |
| `k` | 停止 legged controller |
| `space` | 零速度 |
| `w/s` | 增加/减小前进速度 |
| `a/d` | 增加/减小转向角速度 |
| `q/e` | 增加/减小侧向速度 |
| `z/x` | 减小/增大速度步长 |
| `r` | 重置速度步长 |
| `l` | 开始/结束 AMP 录制 |
| `0` | stance |
| `1` | trot |
| `2` | standing_trot |
| `?` | 显示帮助 |

建议流程：

1. 按 `i` 初始化。
2. 等控制器终端打印 `Initial policy has been received.`。
3. 在 `stance` 稳住几秒。
4. 按 `1` 切到 `trot`。
5. 等约 2 秒保护时间结束，再用 `w/s/a/d/q/e` 逐步加速。

默认速度步长是 `0.10 m/s`。前进速度限幅为 `+-1.00 m/s`，横向速度限幅为 `+-0.50 m/s`，转向角速度限幅为 `+-1.00 rad/s`。切换 gait 后，控制器终端应出现 `[GaitReceiver]: Setting new gait after time ...`；如果没有这行，说明 gait 没有被 MPC 接收，先不要继续加速。

## 自动采集 trot 数据

采集 1-2 分钟左右的 trot 数据：

```bash
cd /path/to/legged_mpc_amp
bash scripts/collect_trot_amp_data.sh
```

脚本会启动 Gazebo、加载 controller、等待 init/initial policy 完成，然后按预设速度序列打开 AMP logger。默认每个 CSV 保存一组连续动作片段，日志写到：

```text
amp_data/auto_trot_YYYYmmdd_HHMMSS/
```

`collection_manifest.json` 会记录每个 CSV 对应的 sequence，以及 sequence 内每段动作的 `vx/vy/wz/duration`。常用环境变量：

```bash
ROBOT_TYPE=go2 \
AMP_LOG_DIR=$(pwd)/amp_data \
GAZEBO_GUI=false \
bash scripts/collect_trot_amp_data.sh
```

默认数据覆盖前进/后退、左右侧移、正反向 yaw、对角平移，以及平移叠加转向。完全静止段使用 `stance`，不会录成 `trot` 原地踏步。

## 转成 IsaacLab motion

导出结构化 IsaacLab motion：

```bash
python3 scripts/convert_amp_data_isaaclab.py \
  --input_dir amp_data \
  --output_dir amp_dataset_isaaclab
```

输出文件：

```text
amp_dataset_isaaclab/isaaclab_motions.npz
amp_dataset_isaaclab/isaaclab_motion_metadata.json
amp_dataset_isaaclab/sequences/*.npy
```

`.npz` 包含 `root_pos`、`root_rot_rpy`、`root_lin_vel_b`、`root_ang_vel_b`、`joint_pos`、`joint_vel`、`foot_contact`，以及拼好的 `motions` flat array。默认 flat layout：

```text
root_pos, root_rot_rpy, root_lin_vel_b, root_ang_vel_b, joint_pos, joint_vel, foot_contact
```

默认会把本项目 OCS2 关节/足端顺序 `[LF, LH, RF, RH]` 重排为 Unitree/IsaacLab 常见的 `[FL, FR, RL, RR]`。如果 IsaacLab 资产或 task 使用原始顺序：

```bash
python3 scripts/convert_amp_data_isaaclab.py \
  --input_dir amp_data \
  --output_dir amp_dataset_isaaclab \
  --no_remap_to_isaaclab
```

训练采样时不要跨 `sequence_lengths` 边界取片段。如果 loader 不处理 `sequence_lengths`，优先直接导入 `sequences/*.npy`，每个文件都是一段连续 motion sequence。

## 足端轨迹可视化

Gazebo 和 controller 启动后，可实时查看 MPC 优化出的四足足端轨迹：

```bash
source env.sh go2
rosrun legged_controllers foot_trajectory_plotter.py --view xz
```

常用视图：

```bash
rosrun legged_controllers foot_trajectory_plotter.py --view xz   # 侧视图
rosrun legged_controllers foot_trajectory_plotter.py --view xy   # 俯视图
rosrun legged_controllers foot_trajectory_plotter.py --view z    # 足端高度曲线
rosrun legged_controllers foot_trajectory_plotter.py --view 3d   # 三维轨迹
```

只看最新 MPC horizon：

```bash
rosrun legged_controllers foot_trajectory_plotter.py \
  --view xz \
  --trail-seconds 0
```

如果 `rosrun` 找不到脚本，可以直接运行源码：

```bash
python3 src/legged_controllers/scripts/foot_trajectory_plotter.py \
  --topic /legged_robot/optimizedStateTrajectory \
  --view xz
```
