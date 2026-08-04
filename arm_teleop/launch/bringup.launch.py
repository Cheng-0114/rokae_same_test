"""拉起真机硬件接口(arm_controller 所在的 real_moveit.launch.py) + 节点 B。

节点 A(joint_input_node) 是交互式终端输入，不放进这个 launch，
单独用 `ros2 run arm_teleop joint_input_node` 在另一个终端里跑。

用法:
  ros2 launch arm_teleop bringup.launch.py \
      robot_type:=AR5L robot_ip:=<控制器IP> local_ip:=<本机IP>
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
        }.items(),
    )

    trajectory_bridge_node = Node(
        package='arm_teleop',
        executable='trajectory_bridge_node',
        name='trajectory_bridge_node',
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('robot_type', default_value='AR5L', description='机型，如 AR5L/AR5R'),
        # 以下两个 IP 沿用 real_moveit.launch.py 自身的默认值，仅作占位，
        # 真机请务必显式传入实际的控制器 IP 和本机 IP。
        DeclareLaunchArgument('robot_ip', default_value='10.0.2.163', description='机器人控制器 IP'),
        DeclareLaunchArgument('local_ip', default_value='10.0.2.162', description='本机(与控制器通信网卡) IP'),
        arm_bringup,
        trajectory_bridge_node,
    ])
