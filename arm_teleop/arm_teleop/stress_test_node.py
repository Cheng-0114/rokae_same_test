"""节点 C：压力测试节点（交互式关节渐进爬升模式）。

背景：早期版本用固定正弦振荡做压测，在较高振幅/频率下机械臂在往返运动中
反复变向加减速，瞬时电流需求叠加节点 B 对轨迹的频繁抢占(每条新指令都从
当前状态重新规划到新目标，默认零速度边界条件)，被机械臂自身的过流/过载
保护判定为异常并断线。

改为这种模式：交互式输入 目标关节角增量(相对当前实际位置, rad) + 发送
频率(Hz) + 总运动时长(s)，把这段位移拆成 N = round(频率*时长) 个小步，
第 k 步的目标 = 基准位置 + k*(目标角度/N)，每步之间轨迹时长固定为 1/频率，
直到第 N 步到达目标点位为止——全程只朝一个方向爬升，没有振荡/反复变向，
所以不会再有振荡模式那种连续变向的电流冲击。

仍然残留的风险(回答"是否还会触发过流"时要说清楚，不要假装完全没有风险)：
- ros2_control 的 JointTrajectoryController 对每条单点轨迹默认按"起止速度
  均为 0"来规划，也就是说哪怕是单方向爬升，每一小步内部依然是"加速-减速
  到 0"，不是真正连续的匀速直线运动。在零速度边界假设下，单步峰值角加速度
  可以估计为 ≈ 4 * 频率 * 目标角度 / 总时长——注意这个量随频率线性增大，
  跟目标角度、总时长构成的"平均角速度"是两回事：频率设得越高，每步分到的
  时间越短，为了在更短时间内完成同样一小步位移，需要的峰值加速度反而越大，
  高频率+大角度+短时长的组合仍可能造成电流冲击触发保护。
- 因此建议：先用保守组合验证(小角度、长时长、适中频率)，观察成功率和机械
  臂是否顺畅，再逐步加大目标角度或提高频率，不要一上来就大角度+高频率。

安全设计：
- 目标角度增量有硬编码上限 MAX_TARGET_DELTA_RAD，防止误输入大角度。
- 默认 enable=false（dry_run）：只计算/打印，不会真正发布指令，需要显式加
  `-p enable:=true` 才会真正驱动机械臂。
- 每次执行前需要终端输入 y 确认才会真正发送，避免误触发真实运动。
- 基准位置持续跟随真实关节状态更新(不是只取一次)，每次新测试都从机械臂
  当前的真实位置开始爬升，角度增量是相对基准的，不是绝对坐标。

成功率的定义(目前是"发送 + 本地不报错"，还不是真正的端到端确认——见下)：
- `publish()` 本身是发后不理，调用不报错只代表消息交给了 DDS，不代表节点B/
  控制器/机械臂真的收到了。曾经尝试过两种改进方向，都还有各自的缺口，
  已经讨论清楚但先不实现，保持当前这个最简单直接的版本：
    1) "等 /arm_teleop/joint_state 更新一次"当确认——测的是硬件接口 read()
       这条读状态通路还活不活，跟某一次具体的 write()/sendCommand() 有没有
       被接收是两回事，read()/write() 是独立的两个循环。
    2) 在硬件接口 write() 里让 SDK sendCommand() 抛异常时发布一个失败计数
       话题——这个只能证明"本地这次调用没报错"，证明不了这条具体指令有没有
       真被发送(joint_position_command_ 是被反复覆写的共享缓冲区)、更证明
       不了机械臂端真的开始响应了。SDK 本身没有应用层的"收到确认"回执，
       这个缺口纯软件层面填不平。
  如果以后要重新做端到端确认，大概率得把这两个信号(SDK发送异常 + 真实
  关节状态是否朝目标方向有物理响应)叠加起来判断，但这依然是推断，不是
  协议层面的握手。

用法：
    ros2 run arm_teleop stress_test_node --ros-args -p enable:=true
    连接建立后按提示输入: 目标角度增量(rad) 频率(Hz) 总时长(s)
    例如: 1 10 10   -> 分 100 步爬升到 base+1.0 rad，每步间隔 0.1s
    输入 q 退出。
"""
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

JOINT_COUNT = 7
DEFAULT_JOINT_NAMES = [f'joint{i}' for i in range(1, JOINT_COUNT + 1)]
MAX_TARGET_DELTA_RAD = 1.0  # 硬上限(~57.3°)，目标角度增量参数无论填多大都会被拒绝


class StressTestNode(Node):

    def __init__(self):
        super().__init__('stress_test_node')
        self.declare_parameter('command_topic', '/arm_teleop/joint_command')
        self.declare_parameter('joint_state_topic', '/arm_teleop/joint_state')
        self.declare_parameter('joint_index', -1)  # -1 = 所有关节同步爬升
        self.declare_parameter('enable', False)  # 安全开关：false 时只打印不真的发送

        self._command_topic = self.get_parameter('command_topic').get_parameter_value().string_value
        self._joint_state_topic = self.get_parameter('joint_state_topic').get_parameter_value().string_value
        self._joint_index = self.get_parameter('joint_index').get_parameter_value().integer_value
        self._enable = self.get_parameter('enable').get_parameter_value().bool_value

        if not (-1 <= self._joint_index < JOINT_COUNT):
            raise ValueError(f'joint_index={self._joint_index} 越界，应为 -1 或 0~{JOINT_COUNT - 1}')

        self._base_positions = None
        self._base_lock = threading.Lock()

        self._pub = self.create_publisher(Float64MultiArray, self._command_topic, 10)
        self._joint_state_sub = self.create_subscription(
            JointState, self._joint_state_topic, self._on_joint_state, 10)

        self.get_logger().info(
            f'压力测试节点(C)已启动: topic={self._command_topic}, joint_index={self._joint_index}, '
            f'enable={self._enable}')
        if not self._enable:
            self.get_logger().warn(
                '当前为 dry_run 模式(enable=false)：只会计算/打印，不会真正发布指令到机械臂。'
                '确认安全后加 -p enable:=true 才会真正驱动。')

    def _on_joint_state(self, msg: JointState):
        if len(msg.name) < JOINT_COUNT or len(msg.position) < JOINT_COUNT:
            return
        by_name = dict(zip(msg.name, msg.position))
        base = [by_name.get(j) for j in DEFAULT_JOINT_NAMES]
        if any(p is None for p in base):
            return
        with self._base_lock:
            self._base_positions = base  # 持续更新，每次测试都以最新真实位置为基准

    def get_base_positions(self):
        with self._base_lock:
            return list(self._base_positions) if self._base_positions is not None else None

    def publish_step(self, positions, duration_s):
        msg = Float64MultiArray()
        msg.data = list(positions) + [duration_s]
        self._pub.publish(msg)


def run_ramp_test(node: StressTestNode, target_delta: float, freq_hz: float, duration_s: float):
    base = node.get_base_positions()
    if base is None:
        print('还没有收到真实关节状态，请确认 bringup.launch.py 已连上机械臂')
        return

    n_steps = max(1, round(freq_hz * duration_s))
    step_delta = target_delta / n_steps
    period_s = 1.0 / freq_hz
    avg_vel = abs(target_delta) / duration_s
    # 零速度边界假设下的单步峰值角加速度估计，用于测试前给出风险提示
    peak_accel_est = 4.0 * freq_hz * abs(target_delta) / duration_s

    print(f'基准位姿(rad): {["%.4f" % p for p in base]}')
    print(f'将分 {n_steps} 步爬升 {target_delta:+.4f} rad，每步间隔 {period_s * 1000:.1f}ms，'
          f'平均角速度≈{avg_vel:.4f} rad/s，估计单步峰值角加速度≈{peak_accel_est:.4f} rad/s²')
    if not node._enable:
        print('当前 enable=false，dry_run，不会真正发送')
    confirm = input('确认执行? [y/N] ').strip().lower()
    if confirm != 'y':
        print('已取消\n')
        return

    success = 0
    fail = 0
    start = time.monotonic()
    for k in range(1, n_steps + 1):
        offset = k * step_delta
        positions = list(base)
        if node._joint_index < 0:
            positions = [p + offset for p in positions]
        else:
            positions[node._joint_index] += offset

        try:
            if node._enable:
                node.publish_step(positions, period_s)
            success += 1
        except Exception as e:  # noqa: BLE001 - 压测要统计任何发布异常，不能让它中断循环
            fail += 1
            print(f'第 {k} 步发布失败: {e}')

        target_time = start + k * period_s
        remaining = target_time - time.monotonic()
        if remaining > 0:
            time.sleep(remaining)

    elapsed = time.monotonic() - start
    success_rate = success / n_steps * 100.0 if n_steps > 0 else 0.0
    print(
        '========== 本次爬升测试结束 ==========\n'
        f'  目标角度增量            : {target_delta:+.4f} rad\n'
        f'  频率/总步数              : {freq_hz:.2f}Hz / {n_steps}\n'
        f'  计划时长/实际耗时        : {duration_s:.2f}s / {elapsed:.2f}s\n'
        f'  成功/失败                : {success} / {fail}\n'
        f'  成功率                   : {success_rate:.2f}%\n'
        f'  enable(是否真实驱动)     : {node._enable}\n'
        '========================================\n')


def main():
    rclpy.init()
    node = StressTestNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print('等待机械臂真实关节状态...')
    while node.get_base_positions() is None:
        if not rclpy.ok():
            return
        time.sleep(0.1)
    print('已获取到真实关节状态，可以开始测试。\n')
    print('输入格式: 目标角度增量(rad) 频率(Hz) 总时长(s)，例如: 1 10 10')
    print('输入 q 退出。\n')

    try:
        while rclpy.ok():
            try:
                line = input('stress_test> ').strip()
            except EOFError:
                break
            if not line:
                continue
            if line.lower() in ('q', 'quit', 'exit'):
                break

            parts = line.split()
            if len(parts) != 3:
                print('输入有误: 需要 3 个数字(目标角度 频率 时长)，用空格分隔')
                continue
            try:
                target_delta, freq_hz, duration_s = (float(p) for p in parts)
            except ValueError:
                print('输入有误: 必须是数字')
                continue

            if freq_hz <= 0 or duration_s <= 0:
                print('输入有误: 频率和时长必须 > 0')
                continue
            if abs(target_delta) > MAX_TARGET_DELTA_RAD:
                print(f'输入有误: 目标角度增量超过硬上限 ±{MAX_TARGET_DELTA_RAD} rad，已拒绝，请重新输入')
                continue

            run_ramp_test(node, target_delta, freq_hz, duration_s)
    except KeyboardInterrupt:
        pass
    finally:
        # 必须先 shutdown 让后台 spin 线程的 rclpy.spin() 返回并 join 完，
        # 再 destroy_node()，否则 spin 线程可能还在用正被销毁的 node 触发 abort。
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()


if __name__ == '__main__':
    main()
