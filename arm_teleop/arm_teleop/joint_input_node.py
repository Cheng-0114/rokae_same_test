"""节点 A：从终端读取关节角度指令，发布到 /arm_teleop/joint_command。

输入格式（空格分隔，弧度）：
    j1 j2 j3 j4 j5 j6 j7 [duration_s]
- 前 7 个数是 AR5L/AR5R 的 joint1~joint7 目标角度（弧度）。
- 第 8 个数可选，是到达目标的运动时长（秒），不填则用 --duration 参数的默认值。
- 输入 q 或 Ctrl+D 退出。
"""
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

JOINT_COUNT = 7


class JointInputNode(Node):

    def __init__(self):
        super().__init__('joint_input_node')
        self.declare_parameter('topic', '/arm_teleop/joint_command')
        self.declare_parameter('duration', 2.0)
        topic = self.get_parameter('topic').get_parameter_value().string_value
        self._default_duration = self.get_parameter('duration').get_parameter_value().double_value
        self._pub = self.create_publisher(Float64MultiArray, topic, 10)
        self.get_logger().info(f'发布话题: {topic}')
        self.get_logger().info(f'默认运动时长: {self._default_duration:.2f}s（可在输入行末尾追加第 8 个数覆盖）')

    def parse_line(self, line: str):
        parts = line.split()
        if len(parts) not in (JOINT_COUNT, JOINT_COUNT + 1):
            raise ValueError(f'需要 {JOINT_COUNT} 或 {JOINT_COUNT + 1} 个数字，实际输入 {len(parts)} 个')
        values = [float(p) for p in parts]
        positions = values[:JOINT_COUNT]
        duration = values[JOINT_COUNT] if len(values) == JOINT_COUNT + 1 else self._default_duration
        if duration <= 0:
            raise ValueError('运动时长必须大于 0')
        return positions, duration

    def publish_command(self, positions, duration):
        msg = Float64MultiArray()
        msg.data = [float(p) for p in positions] + [float(duration)]
        self._pub.publish(msg)


def main():
    rclpy.init()
    node = JointInputNode()

    print(f'输入 {JOINT_COUNT} 个关节角度(rad)，可选追加运动时长(s)，例如:')
    print('  0.1 0.2 0.0 -0.3 0.0 0.5 0.0 2.0')
    print('输入 q 退出。\n')

    try:
        while rclpy.ok():
            try:
                line = input('joint_command> ').strip()
            except EOFError:
                break
            if not line:
                continue
            if line.lower() in ('q', 'quit', 'exit'):
                break
            try:
                positions, duration = node.parse_line(line)
            except ValueError as e:
                print(f'输入有误: {e}')
                continue

            print(f'目标关节角度(rad): {["%.4f" % p for p in positions]}, 时长: {duration:.2f}s')
            confirm = input('确认发送到真实机械臂? [y/N] ').strip().lower()
            if confirm != 'y':
                print('已取消')
                continue

            node.publish_command(positions, duration)
            print('已发送\n')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
