"""节点 B：订阅 /arm_teleop/joint_command，转发为 JointTrajectory 发给 arm_controller。

不直接调 rokae SDK / rokae_driver 的 movej 服务，原因：
- rokae_driver 的 movej 回调写死了 6 个关节(std::array<double,6>)，
  AR5L/AR5R 实际是 7 轴(joint1~joint7)，会丢第 7 轴。
- real_moveit.launch.py 里 enable_driver 默认关闭，官方说明是"避免和
  RokaeHardwareInterface 重复建立 SDK 连接"，说明真机场景走的是
  ros2_control 的硬件接口 + arm_controller(JointTrajectoryController)，
  而不是这个独立的 driver 节点。
所以这里改为把指令封装成标准的 trajectory_msgs/JointTrajectory，
发到 arm_controller 已经订阅的 /arm_controller/joint_trajectory 话题，
这是 rokae_ros2 包里官方支持、关节数正确、且不会抢 SDK 连接的路径。
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

JOINT_COUNT = 7
DEFAULT_JOINT_NAMES = [f'joint{i}' for i in range(1, JOINT_COUNT + 1)]


class TrajectoryBridgeNode(Node):

    def __init__(self):
        super().__init__('trajectory_bridge_node')
        self.declare_parameter('input_topic', '/arm_teleop/joint_command')
        self.declare_parameter('output_topic', '/arm_controller/joint_trajectory')
        self.declare_parameter('joint_names', DEFAULT_JOINT_NAMES)

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self._joint_names = list(self.get_parameter('joint_names').get_parameter_value().string_array_value)

        self._pub = self.create_publisher(JointTrajectory, output_topic, 10)
        self._sub = self.create_subscription(Float64MultiArray, input_topic, self._on_command, 10)

        self.get_logger().info(f'订阅: {input_topic}')
        self.get_logger().info(f'转发到: {output_topic}, joint_names={self._joint_names}')

    def _on_command(self, msg: Float64MultiArray):
        data = list(msg.data)
        n = len(self._joint_names)
        if len(data) != n + 1:
            self.get_logger().error(
                f'收到 {len(data)} 个数，期望 {n} 个关节角度 + 1 个时长，已丢弃该指令')
            return

        positions = data[:n]
        duration_s = data[n]
        if duration_s <= 0:
            self.get_logger().error(f'时长 {duration_s} 非法(<=0)，已丢弃该指令')
            return

        point = JointTrajectoryPoint()
        point.positions = positions
        sec = int(duration_s)
        nanosec = int(round((duration_s - sec) * 1e9))
        point.time_from_start = Duration(sec=sec, nanosec=nanosec)

        traj = JointTrajectory()
        traj.joint_names = self._joint_names
        traj.points = [point]

        self._pub.publish(traj)
        self.get_logger().info(f'已转发: positions={["%.4f" % p for p in positions]}, duration={duration_s:.2f}s')


def main():
    rclpy.init()
    node = TrajectoryBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
