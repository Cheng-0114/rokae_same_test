# Rokae ROS2 软件包说明

**软件包版本：0.0.5**  
变更记录见 [CHANGELOG.rst](CHANGELOG.rst)。  
完整使用说明见 [doc/rokae ros2使用手册.md](doc/rokae%20ros2使用手册.md)。

## 介绍

### 目的和范围

**目的**  
本软件包提供珞石（Rokae）xMate 系列机械臂在 Ubuntu 22.04 + ROS 2 Humble 下的驱动、MoveIt 2 规划与 RViz 可视化集成，覆盖仿真与真实机器人操作。  

**范围**  
当前版本适配协作机型（CR / ER / Pro / SR / AR 系列）以及工业标准机型（XB / NB / EB 等）。后续会适配更多机型，用户也可按现有格式自行扩展。  

更多官方文档：https://docs.rokae.com/docs/ROS2

## 安装与编译

```bash
## 工作空间需包含 src 子目录（示例路径 ~/aos）
source /opt/ros/humble/setup.bash
cd ~/aos
colcon build
source install/setup.bash
```

建议在 **.bashrc** 末尾添加环境加载（`ctrl+h` 显示隐藏文件）：

```bash
source /opt/ros/humble/setup.bash
source ~/aos/install/setup.bash
```

## 工作空间概述

```text
├── doc---------------------------使用手册与 Demo 说明
├── rokae_description-------------URDF / mesh / ros2_control
├── rokae_hardware----------------硬件接口、控制器配置、launch、SDK
├── rokae_msgs--------------------自定义消息 / 服务
├── rokae_example-----------------运动示例
├── rokae_gazebo------------------Gazebo 仿真
└── rokae_xMate*_moveit_config----各机型 MoveIt 配置
```

## 快速启动

```bash
## 假硬件验证软件链
ros2 launch rokae_hardware rokae_moveit_launch.py robot_type:=SR5 use_fake_hardware:=true

## 连接真机（替换机型与 IP）
ros2 launch rokae_hardware rokae_moveit_launch.py robot_type:=SR5 use_fake_hardware:=false robot_ip:=192.168.2.160 local_ip:=192.168.2.129
```

**注意**  
将 `SR5` 换成相应机型；`robot_ip` 为控制器 IP，`local_ip` 为本机与机器人同网段网卡 IP。  

