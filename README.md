# rokae_same_test

AR5L/AR5R 真机控制环境：Ubuntu 22.04 + ROS 2 Humble + Rokae ROS 2 官方栈，
外加一套自定义的 `arm_teleop` 遥操作桥接包（终端输入 -> topic -> arm_controller）。

- [Dockerfile.rokae](Dockerfile.rokae)：构建真机控制环境镜像，会在构建时自动下载官方
  `rokae_ros2` 仓库和匹配版本的 xCore SDK。
- [arm_teleop/](arm_teleop/)：自定义 ROS 2 包，包含两个节点：
  - `joint_input_node`：读终端输入的关节角度，发布到 `/arm_teleop/joint_command`
  - `trajectory_bridge_node`：订阅该话题，转发为 `trajectory_msgs/JointTrajectory`
    发给 `arm_controller`（`/arm_controller/joint_trajectory`）
  - `launch/bringup.launch.py`：一键拉起真机硬件接口 + `trajectory_bridge_node`

## 一、新机器上的环境准备

### 1. 安装 Docker Engine

```bash
curl -fsSL https://get.docker.com | sudo sh
```

验证安装（要求自带 buildx 插件，`docker build` 默认走 buildx）：
```bash
docker --version
docker buildx version
```

### 2. 把当前用户加入 docker 组（免 sudo 跑 docker）

```bash
sudo usermod -aG docker $USER
newgrp docker          # 立即生效；或者重新登录/重启终端
docker ps               # 不报权限错误就说明配置好了
```

### 3. 配置 Git + SSH，确保能访问 GitHub

```bash
# 检查是否已有身份配置
git config --global user.name
git config --global user.email
# 没有的话设置一下
git config --global user.name "你的名字"
git config --global user.email "你的邮箱"

# 检查是否已有 SSH key
ls ~/.ssh/id_ed25519.pub 2>/dev/null || ls ~/.ssh/id_rsa.pub 2>/dev/null

# 没有则生成一个
ssh-keygen -t ed25519 -C "你的邮箱"
cat ~/.ssh/id_ed25519.pub
# 把输出内容添加到 GitHub -> Settings -> SSH and GPG keys

# 验证
ssh -T git@github.com
# 看到 "Hi <你的用户名>! You've successfully authenticated..." 即可
```

### 4. 网络要求

构建镜像的过程中需要能访问：
- `hub.docker.com` / Docker Hub（拉取基础镜像 `osrf/ros:humble-desktop`）
- `github.com` / `raw.githubusercontent.com`（`git clone` rokae_ros2、下载 xCore SDK
  release、rosdep 更新索引）
- Ubuntu/ROS 官方 apt 源

如果在内网/无法直连 GitHub 的环境，需要提前配置好代理或镜像源，否则构建会在
`git clone` 或 `curl` 那一步失败。

## 二、拉取代码

```bash
git clone git@github.com:Cheng-0114/rokae_same_test.git
cd rokae_same_test
```

## 三、构建镜像

```bash
docker build --pull -f Dockerfile.rokae -t rokae_down:latest .
```
构建时间较长（apt 安装、git clone、colcon build 编译，实测约 6 分钟），耐心等待。

## 四、创建并进入容器

```bash
docker run -it \
  --name self_develop_main_arm_testr \
  --hostname self_develop_main_arm_testr \
  -v "$(pwd):/root/self_develop_main_arm_test" \
  --network host \
  --privileged \
  rokae_down:latest \
  bash
```
- `-v "$(pwd):/root/self_develop_main_arm_test"`：把仓库目录挂载进容器，宿主机改代码
  容器内实时可见。
- `--network host`：真机场景需要和机械臂控制器在同一网络直连。
- `--privileged`：需要访问串口/USB 等硬件资源时使用；纯软件调试可以去掉。

之后再次使用：
```bash
docker start -ai self_develop_main_arm_testr     # 启动已停止的容器
docker exec -it self_develop_main_arm_testr bash  # 容器运行中，开新终端进入
```

## 五、把 arm_teleop 接入 ROS 工作空间

`rokae_ros2` 官方栈在镜像构建时已经编译好，放在容器内固定路径 `/root/ar5_ws`
（这是 [Dockerfile.rokae](Dockerfile.rokae) 里 `ENV WS=/root/ar5_ws` 定的名字，不是
ROS 2 规定路径）。自定义包 `arm_teleop` 挂载在 `/root/self_develop_main_arm_test`，
不在这个工作空间里，需要软链接进去后再编译一次：

```bash
# 在容器内执行
ln -sfn /root/self_develop_main_arm_test/arm_teleop /root/ar5_ws/src/arm_teleop

source /opt/ros/humble/setup.bash
cd /root/ar5_ws
colcon build --symlink-install --packages-select arm_teleop
```
`--symlink-install` 是软链接安装，以后改 `arm_teleop` 下的 Python 代码不需要重新
`colcon build`，改完保存即生效；改 `package.xml`/新增依赖/新增 launch 文件才需要重新
`colcon build --packages-select arm_teleop`。

新开的容器终端会自动 `source /root/ar5_ws/install/setup.bash`（写在镜像的
`~/.bashrc` 里），能直接找到 `arm_teleop`、`rokae_hardware`、`rokae_msgs` 等包。

## 六、运行

**终端 1 —— 拉起真机硬件接口 + 桥接节点：**
```bash
docker exec -it self_develop_main_arm_testr bash
ros2 launch arm_teleop bringup.launch.py \
  robot_type:=AR5L robot_ip:=<机器人控制器IP> local_ip:=<本机IP>
```

**终端 2 —— 终端输入控制：**
```bash
docker exec -it self_develop_main_arm_testr bash
ros2 run arm_teleop joint_input_node
```
按提示输入 7 个关节角度(弧度) + 可选运动时长(秒)，确认后发送。

## 安全提醒

真机测试前务必确认：运动幅度小、速度慢、现场有人可以随时按急停。
