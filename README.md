# rokae_same_test

AR5L/AR5R 真机控制环境：Ubuntu 22.04 + ROS 2 Humble + Rokae ROS 2 官方栈，
外加一套自定义的 `arm_teleop` 遥操作桥接包（终端输入 -> topic -> arm_controller）。

本仓库采用**宿主机原生安装**（不用 Docker/容器）。下面的步骤等价于把
[Dockerfile.rokae](Dockerfile.rokae) 里做的事情（装 ROS 2 Humble、装依赖包、拉
rokae_ros2 源码、下 xCore SDK、colcon build）原样搬到宿主机上执行一遍。
`Dockerfile.rokae` 仍保留在仓库里作为容器方式的参考，不是必须用的。

- [arm_teleop/](arm_teleop/)：自定义 ROS 2 包，包含两个节点：
  - `joint_input_node`：读终端输入的关节角度，发布到 `/arm_teleop/joint_command`
  - `trajectory_bridge_node`：订阅该话题，转发为 `trajectory_msgs/JointTrajectory`
    发给 `arm_controller`（`/arm_controller/joint_trajectory`）
  - `launch/bringup.launch.py`：一键拉起真机硬件接口 + `trajectory_bridge_node`

## 前提

- 系统必须是 **Ubuntu 22.04（Jammy）**，ROS 2 Humble 只官方支持这个版本。
  ```bash
  lsb_release -a   # 确认 Codename: jammy
  ```
- 能访问 `packages.ros.org`（ROS 2 apt 源）、`github.com`/
  `raw.githubusercontent.com`（拉 rokae_ros2 源码、下 xCore SDK、rosdep 更新索引）。

## 一、安装 ROS 2 Humble Desktop

```bash
# 1. locale
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 2. 添加 ROS 2 apt 源
sudo apt install -y software-properties-common curl
sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 3. 安装
sudo apt update
sudo apt install -y ros-humble-desktop
```

## 二、安装其余依赖包（对应 Dockerfile.rokae 里的 apt 安装列表）

```bash
sudo apt install -y \
  build-essential ca-certificates curl git \
  python3-colcon-common-extensions python3-rosdep \
  libeigen3-dev liborocos-kdl-dev \
  ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-controller-manager \
  ros-humble-joint-state-broadcaster ros-humble-joint-trajectory-controller \
  ros-humble-forward-command-controller ros-humble-realtime-tools \
  ros-humble-control-msgs ros-humble-trajectory-msgs \
  ros-humble-moveit ros-humble-moveit-ros-planning-interface ros-humble-moveit-kinematics \
  ros-humble-moveit-planners-ompl ros-humble-moveit-simple-controller-manager \
  ros-humble-moveit-ros-visualization ros-humble-interactive-markers \
  ros-humble-joint-state-publisher-gui ros-humble-robot-state-publisher ros-humble-rviz2

# rosdep 只需要在这台机器上初始化一次
sudo rosdep init   # 如果提示 "already exists"，忽略这条报错继续下一步
rosdep update
```

## 三、拉取 rokae_ros2 官方源码 + xCore SDK，编译工作空间

```bash
export WS=$HOME/ar5_ws
mkdir -p $WS/src

# 1. 官方栈源码（含 rokae_hardware、rokae_msgs、AR5L/AR5R MoveIt 配置等）
git clone --depth 1 --branch main \
  https://github.com/RokaeRobot/rokae_ros2.git $WS/src/rokae_ros2

# 2. xCore SDK：下载对应版本/架构的预编译库，放进 rokae_hardware 约定的目录
XCORE_SDK_VERSION=0.7.1
XCORE_SDK_ARCH=x86_64   # 如果是 ARM 主机改成 aarch64
mkdir -p /tmp/xcore_sdk
curl -fL --retry 3 \
  "https://github.com/RokaeRobot/xCoreSDK-CPP/releases/download/v${XCORE_SDK_VERSION}/xCoreSDK-${XCORE_SDK_VERSION}-linux-${XCORE_SDK_ARCH}.tar.gz" \
  -o /tmp/xcore_sdk.tar.gz
tar -xzf /tmp/xcore_sdk.tar.gz -C /tmp/xcore_sdk
SDK_LIB_DIR="$(find /tmp/xcore_sdk -type f -name libxCoreSDK.a -printf '%h\n' | head -n 1)"
mkdir -p $WS/src/rokae_ros2/rokae_hardware/sdk/lib
cp -a "${SDK_LIB_DIR}/." $WS/src/rokae_ros2/rokae_hardware/sdk/lib/
rm -rf /tmp/xcore_sdk /tmp/xcore_sdk.tar.gz

# 校验 SDK 库文件确实放到位了
test -f $WS/src/rokae_ros2/rokae_hardware/sdk/lib/libxCoreSDK.a && \
test -f $WS/src/rokae_ros2/rokae_hardware/sdk/lib/libxMateModel.a && \
echo "xCore SDK OK"

# 3. rosdep 装依赖（跳过 orocos_kdl/rviz——已经用 apt 装过；跳过
#    warehouse_ros_mongo/gazebo_ros——真机场景用不到，且部分源里没有这两个包）
cd $WS
rosdep install --from-paths \
  src/rokae_ros2/rokae_hardware \
  src/rokae_ros2/rokae_msgs \
  src/rokae_ros2/rokae_description \
  src/rokae_ros2/rokae_example \
  src/rokae_ros2/rokae_xMateAR5L_moveit_config \
  --ignore-src -r -y \
  --skip-keys "orocos_kdl rviz warehouse_ros_mongo gazebo_ros"

# 4. 编译
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to \
  rokae_hardware rokae_example rokae_xMateAR5L_moveit_config

# 5. 以后每个新终端自动加载这个工作空间
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source $WS/install/setup.bash" >> ~/.bashrc
```

## 四、拉取本仓库，把 arm_teleop 接入工作空间

```bash
git clone git@github.com:Cheng-0114/rokae_same_test.git ~/rokae_same_test
# 如果仓库是公开的，也可以用: git clone https://github.com/Cheng-0114/rokae_same_test.git ~/rokae_same_test

ln -sfn ~/rokae_same_test/arm_teleop $WS/src/arm_teleop
cd $WS
colcon build --symlink-install --packages-select arm_teleop
source install/setup.bash
```
`--symlink-install` 是软链接安装，以后改 `arm_teleop` 下的 Python 代码不需要重新
`colcon build`，改完保存即生效；改 `package.xml`/新增依赖/新增 launch 文件才需要重新
`colcon build --packages-select arm_teleop`。

## 五、运行

打开新终端会自动 source 好整个工作空间（写进了 `~/.bashrc`）。

**终端 1 —— 拉起真机硬件接口 + 桥接节点：**
```bash
ros2 launch arm_teleop bringup.launch.py \
  robot_type:=AR5L robot_ip:=<机器人控制器IP> local_ip:=<本机IP>
```

**终端 2 —— 终端输入控制：**
```bash
ros2 run arm_teleop joint_input_node
```
按提示输入 7 个关节角度(弧度) + 可选运动时长(秒)，确认后发送。

## 安全提醒

真机测试前务必确认：运动幅度小、速度慢、现场有人可以随时按急停。
