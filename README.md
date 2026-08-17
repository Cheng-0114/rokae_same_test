# rokae_same_test

AR5L/AR5R 真机控制环境：Ubuntu 22.04 + ROS 2 Humble + Rokae ROS 2 官方栈，
外加一套自定义的 `arm_teleop` 遥操作桥接包（终端输入 -> topic -> arm_controller）。

本仓库采用**宿主机原生安装**（不用 Docker/容器）。下面的步骤等价于把
[Dockerfile.rokae](Dockerfile.rokae) 里做的事情（装 ROS 2 Humble、装依赖包、拉
rokae_ros2 源码、下 xCore SDK、colcon build）原样搬到宿主机上执行一遍。
`Dockerfile.rokae` 仍保留在仓库里作为容器方式的参考，不是必须用的。

- [arm_teleop/](arm_teleop/)：自定义 ROS 2 包，包含三个节点：
  - **节点 A** `joint_input_node`：可视化遥操作面板（Tkinter GUI），7 个关节滑块
    实时设定目标、7 个当前角度只读展示、频率/成功率/运行时间，连续流式发布到
    `/arm_teleop/joint_command`；另有"全部左移/全部右移"按钮，按住可以让全部
    关节联动移动（同一个限速）
  - **节点 B** `trajectory_bridge_node`：订阅该话题，按 `control_mode` 转发——
    `trajectory` 模式转发为 `trajectory_msgs/JointTrajectory` 给 `arm_controller`；
    `streaming` 模式直接转发位置给 `streaming_position_controller`。同时把真实
    关节状态转发到 `/arm_teleop/joint_state`
  - **节点 C** `stress_test_node`：交互式压力测试节点，命令行输入"目标角度增量
    频率 时长"，分步爬升到目标点位并统计发送成功率
  - `launch/bringup.launch.py`：一键拉起真机硬件接口 + 节点 B，`control_mode`
    参数二选一决定走轨迹控制还是流式控制（详见下面"运行"一节）
  - `scripts/cleanup.sh`：崩溃后手动清理残留进程用（详见"故障排查"一节）
- [arm_teleop_usb/](arm_teleop_usb/)：独立的 `ament_cmake` C++ 包，只有一个节点：
  - **节点 D** `usb_motor_bridge_node`：主臂 USB 电机反馈桥接节点。通过 libusb
    读取主臂控制板（GD32，USB CDC，VID=0x28e9 PID=0x018a）发来的 DM 电机反馈帧
    （`CMD_DM_FB`），在帧回调里整帧打包成一条 `sensor_msgs/JointState` 发布到
    `/arm_teleop/master_joint_state`（name=`joint{id}`，position/velocity/effort
    对应 位置/速度/力矩）。启动后自动握手(`CMD_CONNECT`)+ 维持心跳，但**不会
    自动使能电机**——电机使能是会让主臂真的通电出力的动作，需要显式调用
    `~/enable_motors` 服务（`std_srvs/SetBool`）才会生效。协议/USB收发代码
    复制自 [remote_control-8ee3e55/pc_usb](arm_teleop/remote_control-8ee3e55/pc_usb/)
    （主臂固件配套的 PC 端调试程序），逻辑未做修改。

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
  libusb-1.0-0-dev \
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
`libusb-1.0-0-dev` 是节点D(`arm_teleop_usb`)读主臂 USB 反馈需要的库，其余跟原来一样。

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

## 四、拉取本仓库，把 arm_teleop / arm_teleop_usb 接入工作空间

```bash
git clone git@github.com:Cheng-0114/rokae_same_test.git ~/rokae_same_test
# 如果仓库是公开的，也可以用: git clone https://github.com/Cheng-0114/rokae_same_test.git ~/rokae_same_test

ln -sfn ~/rokae_same_test/arm_teleop $WS/src/arm_teleop
ln -sfn ~/rokae_same_test/arm_teleop_usb $WS/src/arm_teleop_usb
cd $WS
colcon build --symlink-install --packages-select arm_teleop arm_teleop_usb
source install/setup.bash
```
`--symlink-install` 是软链接安装，以后改 `arm_teleop` 下的 Python 代码不需要重新
`colcon build`，改完保存即生效；改 `package.xml`/新增依赖/新增 launch 文件才需要重新
`colcon build --packages-select arm_teleop`。`arm_teleop_usb` 是 C++ 包，改了
`arm_teleop_usb` 下的 `.cpp`/`.h` 代码都需要重新
`colcon build --packages-select arm_teleop_usb`。

## 五、运行前网络准备（把有线网卡配到机械臂同一网段）

`robot_ip` 是机械臂**控制器**的 IP，`local_ip` 是**本机网卡**的 IP，两个别填反
（用 `ip addr` 看本机网卡实际 IP，不要凭印象填）。这一步只需要在**第一次**接
这台机械臂、或者换了台电脑时做一次；配成静态连接后插拔网线/重启都不用重
新配。

1. 确认有线网口名字和物理链路状态：
   ```bash
   nmcli device status          # 找有线网口对应的 DEVICE 名，例如 enx6c1ff706895d
   cat /sys/class/net/<网口名>/carrier   # 1 = 有物理连接，0 = 网线没插好/对端没通
   ```
2. 找机械臂控制器实际的 IP（找不到就问带这台机械臂的人，或者去示教器网络设置
   里看），确认跟本机网口不在同一网段（比如控制器是 `192.168.9.160`，本机网口
   默认拿到的是别的网段），就需要给本机网口追加一个跟控制器同网段的静态 IP：
   ```bash
   # <连接名> 用 nmcli connection show 里对应这个网口的 NAME（一般叫 "Wired connection 1"）
   # <本机静态IP> 跟控制器 IP 同网段、不能跟网段内其它设备重复，例如 192.168.9.50/24
   sudo nmcli connection modify "<连接名>" +ipv4.addresses <本机静态IP>/24
   sudo nmcli connection up "<连接名>"
   ```
   这是给现有连接**追加**一个静态地址，不会动这个网口原来的地址（比如原来
   DHCP/静态拿到的办公网 IP），两个地址可以共存。
3. 验证：
   ```bash
   ip addr show <网口名>        # 确认新加的静态IP在列表里
   ping -c 4 <控制器IP>
   ```
   如果一直 `目标主机不可达`（ARP 失败）且物理链路 carrier=1，说明本机这一侧
   配置没问题，问题在链路对端——去查控制器有没有上电、网线是不是真的接到了
   控制器网口（而不是接到了别的交换机/路由器）。

## 六、USB 设备权限准备（节点D，主臂USB反馈，仅用到节点D时需要）

`usb_motor_bridge_node` 用 libusb 直接操作 USB 设备，默认普通用户没有权限，
不加这一步就得 `sudo ros2 run ...` 才能打开设备——但 ROS 节点长期以 root 身份
跑并不合适（容易跟其它以普通用户跑的节点产生权限不一致的问题），建议加一条
udev 规则让普通用户也能直接访问：

```bash
sudo tee /etc/udev/rules.d/99-arm-master-usb.rules > /dev/null <<'EOF'
SUBSYSTEM=="usb", ATTR{idVendor}=="28e9", ATTR{idProduct}=="018a", MODE="0666", GROUP="plugdev"
EOF
sudo udevadm control --reload-rules
sudo udevadm trigger
```
加完规则后**把主臂 USB 线拔了重插一次**（udev 规则只在设备重新枚举时生效），
然后 `ros2 run arm_teleop_usb usb_motor_bridge_node` 不用 `sudo` 也能正常打开
设备。

## 七、运行

打开新终端会自动 source 好整个工作空间（写进了 `~/.bashrc`）。

### 模式一：trajectory（默认，走 MoveIt/JointTrajectoryController）

**终端 1 —— 拉起真机硬件接口 + 节点 B：**
```bash
ros2 launch arm_teleop bringup.launch.py \
  robot_type:=AR5L robot_ip:=<机器人控制器IP> local_ip:=<本机IP>
```
等看到 `机器人连接成功` 和实时打印的关节角度，再进行下一步。

**终端 2 —— 节点 C 压力测试（可选）：**
```bash
ros2 run arm_teleop stress_test_node --ros-args -p enable:=true
```
等提示"已获取到真实关节状态，可以开始测试"后，在 `stress_test>` 提示符输入
`目标角度增量(rad) 频率(Hz) 总时长(s)`，例如 `0.02 50 1`，输入 `y` 确认才会真正
发送；测完自动打印成功率，输入 `q` 退出。

### 模式二：streaming（高频遥操作，走节点 A 的滑块面板）

**终端 1 —— 同上，但加两个参数：**
```bash
ros2 launch arm_teleop bringup.launch.py \
  robot_type:=AR5L robot_ip:=<机器人控制器IP> local_ip:=<本机IP> \
  control_mode:=streaming enable_servoj:=true
```

**终端 2 —— 节点 A 可视化面板：**
```bash
ros2 run arm_teleop joint_input_node
```

窗口里等滑块自动解锁（代表已经拿到机械臂当前真实位置），按下面顺序操作：

1. **先点一次"同步目标到当前位置"**，把滑块和实际发送值对齐到机械臂当前
   真实姿态，避免程序里默认的目标值跟机械臂实际位置对不上、一开始发送就
   跳一下。这一步不能省，**每次点"开始"之前都建议先点一下**，尤其是如果
   在这之前用节点 A/C 单独动过机械臂。
2. 确认"发送频率"符合预期（这个只能在点"开始"前改，开始后锁定）。
3. 想真正驱动机械臂就勾上"使能"，不勾就是只看数据不发送(dry-run)，可以先
   这样跑一遍确认没问题。
4. 点"开始"，再去拖滑块。
5. 想停止就点"停止"（会保留当前发送值，不会跳回0）；关窗口前建议先点停止。

`streaming` 模式下没有 `JointTrajectoryController` 自带的限速/限加速度保护，
平滑和安全完全靠发送端自己控制——节点 A 内部有一个隐藏的滑块限速（每周期
最多移动 `0.3rad/频率`），拖得再快机械臂也不会瞬间跳过去，但这也意味着别
把这个限速去掉，除非你很清楚自己在做什么。

`trajectory` 和 `streaming` 不能同时用、也不能运行中途切换，要切换必须重启
`bringup.launch.py`（换 `control_mode` 参数）。

### 节点D：主臂 USB 电机反馈（独立于上面两种模式，跟着主臂硬件走）

跟 AR5L/AR5R 那条真机控制链路完全独立，不需要先跑 `bringup.launch.py`。插好
主臂 USB 线后：

```bash
ros2 run arm_teleop_usb usb_motor_bridge_node
```

日志打印"USB 设备已连接，发送 CMD_CONNECT 建立协议连接"说明已经握手成功，
`ros2 topic echo /arm_teleop/master_joint_state` 应该能看到反馈——但反馈帧要
真正从固件发出来，还需要电机被使能，节点D**不会自动使能**（使能属于会让
主臂真的通电出力的动作），手动触发：

```bash
ros2 service call /usb_motor_bridge_node/enable_motors std_srvs/srv/SetBool "{data: true}"
```
`data: false` 则失能。失能/程序退出前建议先失能一次，避免主臂一直带力。

## 八、故障排查

**`Robot instantiation failed: 网络异常` / `network connection`**：
1. 先 `ping <控制器IP>` 确认网络层通不通。
2. 检查 `robot_ip`/`local_ip` 有没有填反（`robot_ip` 是控制器、`local_ip` 是
   本机网卡，用 `ip addr` 核对）。
3. 如果网络和 IP 都没问题，但连了一下又断，或者反复连不上：去示教器上看
   有没有报警没清除、当前是不是"远程模式"、示教器本身有没有占着连接。
4. 都排除了还不行，给控制柜断电重启一次，清掉控制器内部可能残留的连接
   状态。

**崩溃后残留进程导致没法重新启动**：
`bringup.launch.py` 本身现在已经加了自动清理——`ros2_control_node` 崩溃时
会带着 `robot_state_publisher`/节点B/spawner 一起自动退出，`ros2 launch`
命令自己就会结束，不需要手动清理。

但节点 A（GUI）和节点 C 是在单独终端用 `ros2 run` 起的，不属于这棵 launch
树，如果这两个进程本身卡死或者终端被强制关掉，可能会有残留。这种情况用：
```bash
bash ~/rokae_same_test/arm_teleop/scripts/cleanup.sh
```
会先列出匹配到的相关残留进程，确认后再清理，不会误杀无关进程。

**`Could not enable FIFO RT scheduling policy: Operation not permitted`**：
这台机器没有给 ROS 进程开实时调度权限，`write()`/`read()` 周期可能偶尔抖动
到几十毫秒甚至更久。用 `streaming` 模式 + `enable_servoj:=true` 时如果频繁
断线，可以先试试 `enable_servoj:=false`（关掉 SDK 侧的伺服跟踪，纯粹靠发送
端自己控制节奏）排除是不是这个导致的。

**节点D启动失败 `USB 热插拔监听启动失败` / 一直打不开设备**：
1. 先确认设备确实插上了：`lsusb | grep 28e9:018a`。
2. 大概率是权限问题：按"六、USB 设备权限准备"加 udev 规则并重插一次 USB 线；
   临时验证可以先 `sudo ros2 run arm_teleop_usb usb_motor_bridge_node` 看是否
   变正常，能则确认是权限问题。
3. `/arm_teleop/master_joint_state` 一直没有消息：先看节点D日志有没有打印
   "USB 设备已连接"，没有说明协议层握手都没成功（检查设备/线缆）；打印了但
   还是没消息，大概率是忘了调 `enable_motors` 服务使能电机（固件只有电机使能
   后才会产生反馈帧，见节点D运行说明）。

**`apt-get update`/`apt-get install` 卡住不动，长时间没反应**：
大概率是 DNS 把包源解析成了 IPv6 地址，但这台机器/网络实际没有可用的 IPv6
路由，apt 卡在连接超时上。用 `getent hosts archive.ubuntu.com` 看解析结果是
不是全是 IPv6（形如 `2620:...`），是的话强制 apt 走 IPv4：
```bash
apt-get update -o Acquire::ForceIPv4=true
apt-get install -y -o Acquire::ForceIPv4=true <包名>
```
卡住的旧 apt 进程记得先 kill 掉再重试，否则会报 `dpkg lock` 被占用。

## 安全提醒

真机测试前务必确认：运动幅度小、速度慢、现场有人可以随时按急停。

节点D的 `enable_motors` 服务调用会让**主臂**（不是 AR5 follower）真的通电出力，
调用前确认主臂周围没有人手/异物卡在关节间隙里；不用的时候记得调用一次
`enable_motors {data: false}` 失能，不要让主臂一直带力空转。

## 附：用 Docker 容器跑（可选，`Dockerfile.rokae` 对应的环境）

前面一到八都是宿主机原生安装的流程；如果是用 `Dockerfile.rokae` build 出来的
镜像（或已有一个基于它的容器，比如 `rokae_down:latest`）跑，有两点跟宿主机
原生方式不一样：

**1. 节点A(GUI) 要用 X11 把窗口显示到宿主机屏幕，X11 参数只能在创建容器时带上**
（`docker exec` 之后没法后补 `-v`/新的 `-e`），完整的创建命令：
```bash
xhost +local:root   # 宿主机执行一次，放行容器内root连宿主机X server
                     # 宿主机图形会话重启后要重新执行这条

docker run -dit \
  --name <容器名> \
  --network host \
  --privileged \
  -v <本仓库绝对路径>:/root/<容器内挂载目录名> \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -e DISPLAY=$DISPLAY \
  rokae_down:latest bash
```
`--network host` 让容器和宿主机共用同一份网络栈，理论上 DDS 话题发现应该跨
host/container 边界互通，但实测发现**不总是可靠**（具体原因没查透彻，怀疑
跟这台机器这轮反复改动有线网卡IP、多网卡多路由有关系）。稳妥做法：**节点
A/B/C/D 放到同一个容器里跑**，不要指望节点A单独在宿主机跑、节点B/D在容器
里跑还能实时互相发现——本仓库就是这么踩过一次坑才改成全放同一容器的。

**2. `Dockerfile.rokae` 现在的 apt 列表里已经带上了这几个包**，都是给容器内
跑 GUI/节点D/排查用的，跟真机控制链路本身无关：
- `libusb-1.0-0-dev` —— 节点D(`arm_teleop_usb`)编译依赖
- `iputils-ping` / `usbutils` —— 排查用（`ping`/`lsusb`）
- `fonts-wqy-zenhei` —— 节点A界面是中文标签，没有中文字体窗口里的字会
  完全不显示（不是乱码，是真的空白），装了这个字体包就正常了

如果现在跑的容器是在这几个包加进 `Dockerfile.rokae` 之前就建好的，重新
build 镜像太重，直接在容器里手动补装就行（apt 卡住看上面故障排查那条
IPv6 说明）：
```bash
docker exec -it <容器名> bash
apt-get update -o Acquire::ForceIPv4=true
apt-get install -y -o Acquire::ForceIPv4=true \
  libusb-1.0-0-dev iputils-ping usbutils fonts-wqy-zenhei
```
