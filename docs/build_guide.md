# 编译与环境配置

本项目把 OCS2 和 `ocs2_robotic_assets` 放在 `src/third_party/` 下，作为当前 catkin workspace 的第三方源码一起编译，不需要额外维护 `~/ocs2_catkin_ws`。

当前随仓库编译的第三方包：

```text
src/third_party/ocs2
src/third_party/ocs2_robotic_assets
src/qpoases_catkin
```

OCS2 的 RaiSim、MPCNet、文档和无关示例包通过 `CATKIN_IGNORE` 跳过。Pinocchio、HPP-FCL、Gazebo、ROS controller 等仍作为系统依赖安装。

## 安装依赖

下面以 Ubuntu 20.04 + ROS Noetic 为例，其他 ROS 1 发行版请安装等价包：

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

如果 Pinocchio 来自 conda 或自定义路径，需要让 `pkg-config` 能找到它：

```bash
export PINOCCHIO_PKGCONFIG="${CONDA_PREFIX}/lib/pkgconfig"
```

使用系统包 `libpinocchio-dev` 时通常不需要设置这个变量。

## 编译

```bash
cd /path/to/legged_mpc_amp
bash setup.sh
```

`setup.sh` 会自动查找 `${ROS_DISTRO}` 对应的 ROS 环境，默认是 `noetic`。需要显式指定路径或编译参数时：

```bash
ROS_SETUP=/opt/ros/noetic/setup.bash \
PINOCCHIO_PKGCONFIG="${CONDA_PREFIX}/lib/pkgconfig" \
CATKIN_BUILD_ARGS="-DCMAKE_BUILD_TYPE=RelWithDebInfo" \
bash setup.sh
```

如果 Pinocchio 使用系统包，可以省略 `PINOCCHIO_PKGCONFIG`。如果机器不是 Noetic，把 `ROS_SETUP` 改成对应 ROS 1 发行版的 `setup.bash`。

## 每个终端都 source

默认机器人是 Go2。每个新终端先执行：

```bash
cd /path/to/legged_mpc_amp
source env.sh go2
```

也可以切换其他已配置机器人：

```bash
source env.sh a1
source env.sh go1
source env.sh aliengo
source env.sh Lite3
```

`env.sh` 和 `setup.sh` 支持同样的路径覆盖：

```bash
export ROS_SETUP=/opt/ros/noetic/setup.bash
export PINOCCHIO_PKGCONFIG="${CONDA_PREFIX}/lib/pkgconfig"
source env.sh go2
```

如果使用 conda 版 Pinocchio，`env.sh` 只会为 Pinocchio/HPP-FCL 相关库做必要的运行时设置。不要把整个 conda `lib` 目录全局加入 `LD_LIBRARY_PATH`，否则 Gazebo 可能误加载 conda 里的 `libffi`、`libcurl` 等库并直接退出。

## 启动仿真

终端 1：

```bash
source env.sh go2
roslaunch legged_robot_description empty_world.launch
```

该 launch 会生成 `tmp/legged_control/<robot_type>.urdf`，启动 Gazebo，并加载机器人模型。

终端 2：

```bash
source env.sh go2
roslaunch legged_controllers keyboard_control.launch \
  enable_amp_logging:=true \
  amp_log_dir:=$(pwd)/amp_data
```

启动后先按 `i` 初始化。控制器终端打印 `Initial policy has been received.` 后，让机器人在 `stance` 稳住几秒，再按 `1` 切到 `trot`。
