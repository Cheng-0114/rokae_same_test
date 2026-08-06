"""拉起真机硬件接口(arm_controller 所在的 real_moveit.launch.py) + 节点 B。

节点 A(joint_input_node) 是交互式终端输入，不放进这个 launch，
单独用 `ros2 run arm_teleop joint_input_node` 在另一个终端里跑。

用法:
  ros2 launch arm_teleop bringup.launch.py \
      robot_type:=AR5L robot_ip:=<控制器IP> local_ip:=<本机IP>

高频遥操作(流式控制)模式:
  ros2 launch arm_teleop bringup.launch.py \
      robot_type:=AR5L robot_ip:=<控制器IP> local_ip:=<本机IP> \
      control_mode:=streaming enable_servoj:=true
  此时 arm_controller(JointTrajectoryController) 不会被启动，改为启动
  streaming_position_controller(JointGroupPositionController)，节点B也会
  切换成直接转发位置命令，不再经过样条轨迹重新规划；enable_servoj:=true 让
  硬件接口额外调用 SDK 的 setServoJoint 做带 lookahead 的平滑跟踪。
  注意：streaming 模式没有 JointTrajectoryController 自带的限速/限加速度
  保护，发送端(节点B/C)必须自己控制好步长和频率。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_type = LaunchConfiguration('robot_type')
    robot_ip = LaunchConfiguration('robot_ip')
    local_ip = LaunchConfiguration('local_ip')
    control_mode = LaunchConfiguration('control_mode')
    enable_servoj = LaunchConfiguration('enable_servoj')
    servo_joint_period_s = LaunchConfiguration('servo_joint_period_s')

    real_moveit_launch = os.path.join(
        get_package_share_directory('rokae_hardware'),
        'launch', 'real_moveit.launch.py',
    )

    arm_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(real_moveit_launch),
        launch_arguments={
            'robot_type': robot_type,
            'robot_ip': robot_ip,
            'local_ip': local_ip,
            'control_mode': control_mode,
            'enable_servoj': enable_servoj,
            'servo_joint_period_s': servo_joint_period_s,
        }.items(),
    )

    trajectory_bridge_node = Node(
        package='arm_teleop',
        executable='trajectory_bridge_node',
        name='trajectory_bridge_node',
        output='screen',
        parameters=[{'control_mode': control_mode}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('robot_type', default_value='AR5L', description='机型，如 AR5L/AR5R'),
        # 以下两个 IP 沿用 real_moveit.launch.py 自身的默认值，仅作占位，
        # 真机请务必显式传入实际的控制器 IP 和本机 IP。
        DeclareLaunchArgument('robot_ip', default_value='10.0.2.163', description='机器人控制器 IP'),
        DeclareLaunchArgument('local_ip', default_value='10.0.2.162', description='本机(与控制器通信网卡) IP'),
        DeclareLaunchArgument(
            'control_mode', default_value='trajectory',
            description="'trajectory'(默认，走MoveIt/JointTrajectoryController) 或 "
                         "'streaming'(高频遥操作，走 streaming_position_controller，节点B直发位置命令)",
        ),
        DeclareLaunchArgument(
            'enable_servoj', default_value='false',
            description='true 时硬件接口调用 SDK setServoJoint 做带 lookahead 的平滑跟踪，建议 streaming 模式下开启',
        ),
        DeclareLaunchArgument(
            'servo_joint_period_s', default_value='0.004',
            description='setServoJoint 控制周期(秒)，应与 controller_manager update_rate 一致(250Hz -> 0.004)',
        ),
        arm_bringup,
        trajectory_bridge_node,
    ])
