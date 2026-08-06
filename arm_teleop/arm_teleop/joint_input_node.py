"""节点 A：可视化关节遥操作面板，连续发布到 /arm_teleop/joint_command。

跟旧版终端交互(单次输入目标角+运动时长)不同，这是连续流式控制：拖动滑块
实时设定目标，程序按固定频率不断把"当前应该在哪"发出去，因此**没有"移动
时长"这个概念**——频率本身就是节奏来源。这个设计只对接节点 B 的
control_mode=streaming 路径（streaming_position_controller），配合
bringup.launch.py 的 control_mode:=streaming enable_servoj:=true 使用。
如果下游还是 control_mode=trajectory，每次滑块变化都会打断上一条正在
执行的样条轨迹，会顿挫，不建议这样用。

安全设计：
- 7 个滑块的范围用 URDF 里 AR5L 的真实关节限位(见 JOINT_LIMITS)，物理上
  不可能拖出超限的目标。
- 滑块目标(人手可以瞬间拖到底)和实际发送值分开：实际发送值每个周期只朝
  滑块目标靠近最多 MAX_SLEW_RATE_RAD_S/频率 这么多，避免手快导致瞬时大跳变。
- 默认 enable=false（dry_run）：只计算/刷新界面，不会真正发布指令，需要
  勾选界面上的"使能"复选框才会真正驱动机械臂。
- 程序启动后滑块被禁用，等收到第一帧真实关节状态、把滑块和发送值同步到
  真实位置后才解锁，避免从一个瞎猜的默认位置突然开始流式发送。
- 频率只能在点击"开始"之前修改，开始后锁定，跟发送节奏绑定的参数不允许
  运行中途改，避免运行时改变来源不明的抖动。

消息格式沿用节点 B 现有协议(Float64MultiArray: 7 个位置 + 1 个占位时长)，
时长字段本身在 streaming 模式下会被节点 B 忽略，这里固定填 1/频率，
纯粹是为了满足节点 B 消息格式校验(要求最后一个数 >0)，界面上不暴露这个值。
"""
import threading
import time
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

JOINT_COUNT = 7
JOINT_NAMES = [f'joint{i}' for i in range(1, JOINT_COUNT + 1)]
# AR5L 真实关节限位(rad)，取自 AR5-5_07L-W4C4A2.urdf.xacro 的 <limit lower.../upper.../>
JOINT_LIMITS = [
    (-3.1067, 3.1067),
    (-2.0944, 2.0944),
    (-3.1067, 3.1067),
    (-1.0472, 2.5307),
    (-3.1067, 3.1067),
    (-1.0472, 1.0472),
    (-1.0472, 1.0472),
]
MAX_SLEW_RATE_RAD_S = 0.3  # 实际发送值朝滑块目标靠近的最大速率，独立于人手拖动速度
DEFAULT_FREQ_HZ = 20.0
UI_REFRESH_MS = 100


class JointGuiNode(Node):
    """只负责 ROS 收发；界面状态(滑块/按钮)全部留在 Tk 主线程里。"""

    def __init__(self):
        super().__init__('joint_input_node')
        self.declare_parameter('command_topic', '/arm_teleop/joint_command')
        self.declare_parameter('joint_state_topic', '/arm_teleop/joint_state')

        command_topic = self.get_parameter('command_topic').get_parameter_value().string_value
        joint_state_topic = self.get_parameter('joint_state_topic').get_parameter_value().string_value

        self._pub = self.create_publisher(Float64MultiArray, command_topic, 10)
        self._sub = self.create_subscription(
            JointState, joint_state_topic, self._on_joint_state, 10)

        self._lock = threading.Lock()
        self._current_positions = None  # 最新真实关节角度，由订阅回调持续更新

        self.get_logger().info(f'发布指令到: {command_topic}(streaming 模式)')
        self.get_logger().info(f'订阅真实关节状态: {joint_state_topic}')

    def _on_joint_state(self, msg: JointState):
        if len(msg.name) < JOINT_COUNT or len(msg.position) < JOINT_COUNT:
            return
        by_name = dict(zip(msg.name, msg.position))
        positions = [by_name.get(j) for j in JOINT_NAMES]
        if any(p is None for p in positions):
            return
        with self._lock:
            self._current_positions = positions

    def get_current_positions(self):
        with self._lock:
            return list(self._current_positions) if self._current_positions is not None else None

    def publish_positions(self, positions, freq_hz):
        msg = Float64MultiArray()
        msg.data = list(positions) + [1.0 / freq_hz]
        self._pub.publish(msg)


class TeleopGui:

    def __init__(self, root: tk.Tk, node: JointGuiNode):
        self._root = root
        self._node = node
        self._start_time = time.monotonic()

        self._streaming = False
        self._commanded = None  # 实际发送值(限速后)，None 表示还没同步到真实位置
        self._tick_count = 0
        self._success_count = 0

        self._enable_var = tk.BooleanVar(value=False)
        self._freq_var = tk.StringVar(value=str(DEFAULT_FREQ_HZ))
        self._current_vars = [tk.StringVar(value='--') for _ in range(JOINT_COUNT)]
        self._target_vars = [tk.DoubleVar(value=0.0) for _ in range(JOINT_COUNT)]
        self._success_rate_var = tk.StringVar(value='--')
        self._runtime_var = tk.StringVar(value='00:00:00')
        self._status_var = tk.StringVar(value='等待机械臂真实关节状态...')

        self._build_widgets()
        root.protocol('WM_DELETE_WINDOW', self._on_close)
        root.after(UI_REFRESH_MS, self._on_ui_tick)

    def _build_widgets(self):
        root = self._root
        root.title('关节遥操作面板 (节点 A)')

        top = ttk.Frame(root, padding=8)
        top.grid(row=0, column=0, sticky='ew')
        ttk.Checkbutton(top, text='使能(真正驱动机械臂)', variable=self._enable_var).grid(
            row=0, column=0, padx=4)
        ttk.Label(top, text='发送频率(Hz):').grid(row=0, column=1, padx=(16, 4))
        self._freq_entry = ttk.Entry(top, textvariable=self._freq_var, width=8)
        self._freq_entry.grid(row=0, column=2)
        self._start_btn = ttk.Button(top, text='开始', command=self._on_start, state='disabled')
        self._start_btn.grid(row=0, column=3, padx=(16, 4))
        self._stop_btn = ttk.Button(top, text='停止', command=self._on_stop, state='disabled')
        self._stop_btn.grid(row=0, column=4, padx=4)
        self._sync_btn = ttk.Button(top, text='同步目标到当前位置', command=self._on_sync, state='disabled')
        self._sync_btn.grid(row=0, column=5, padx=(16, 4))

        table = ttk.Frame(root, padding=8)
        table.grid(row=1, column=0, sticky='ew')
        ttk.Label(table, text='关节', width=8).grid(row=0, column=0)
        ttk.Label(table, text='当前角度(rad)', width=14).grid(row=0, column=1)
        ttk.Label(table, text='目标角度(rad)  —  拖动滑块设定').grid(row=0, column=2, sticky='w')

        self._sliders = []
        for i in range(JOINT_COUNT):
            lower, upper = JOINT_LIMITS[i]
            ttk.Label(table, text=JOINT_NAMES[i]).grid(row=i + 1, column=0, pady=2)
            ttk.Label(table, textvariable=self._current_vars[i], width=14).grid(row=i + 1, column=1)
            row_frame = ttk.Frame(table)
            row_frame.grid(row=i + 1, column=2, sticky='ew')
            slider = ttk.Scale(
                row_frame, from_=lower, to=upper, orient='horizontal', length=420,
                variable=self._target_vars[i], state='disabled')
            slider.pack(side='left')
            value_label = ttk.Label(row_frame, width=8)
            value_label.pack(side='left', padx=6)
            self._target_vars[i].trace_add(
                'write',
                lambda *_a, lbl=value_label, var=self._target_vars[i]: lbl.config(text=f'{var.get():.4f}'))
            value_label.config(text=f'{self._target_vars[i].get():.4f}')
            self._sliders.append(slider)

        bottom = ttk.Frame(root, padding=8)
        bottom.grid(row=2, column=0, sticky='ew')
        ttk.Label(bottom, text='信号传输成功率:').grid(row=0, column=0)
        ttk.Label(bottom, textvariable=self._success_rate_var).grid(row=0, column=1, padx=(4, 24))
        ttk.Label(bottom, text='程序运行时间:').grid(row=0, column=2)
        ttk.Label(bottom, textvariable=self._runtime_var).grid(row=0, column=3, padx=4)

        status = ttk.Frame(root, padding=(8, 0, 8, 8))
        status.grid(row=3, column=0, sticky='ew')
        ttk.Label(status, textvariable=self._status_var, foreground='#a06000').pack(anchor='w')

    def _on_sync(self):
        current = self._node.get_current_positions()
        if current is None:
            return
        for i in range(JOINT_COUNT):
            self._target_vars[i].set(current[i])
        self._commanded = list(current)
        self._status_var.set('已同步目标到当前真实位置')

    def _on_start(self):
        try:
            freq = float(self._freq_var.get())
        except ValueError:
            self._status_var.set('频率必须是数字')
            return
        if freq <= 0:
            self._status_var.set('频率必须 > 0')
            return
        if self._commanded is None:
            self._status_var.set('还没有真实关节状态，无法开始')
            return

        self._streaming = True
        self._tick_count = 0
        self._success_count = 0
        self._freq_entry.config(state='disabled')
        self._start_btn.config(state='disabled')
        self._stop_btn.config(state='normal')
        self._status_var.set(f'流式发送中... enable={self._enable_var.get()}')
        self._period_ms = max(1, round(1000.0 / freq))
        self._send_tick()

    def _on_stop(self):
        self._streaming = False
        self._freq_entry.config(state='normal')
        self._start_btn.config(state='normal')
        self._stop_btn.config(state='disabled')
        self._status_var.set('已停止')

    def _send_tick(self):
        if not self._streaming:
            return
        try:
            freq = float(self._freq_var.get())
        except ValueError:
            freq = DEFAULT_FREQ_HZ
        max_step = MAX_SLEW_RATE_RAD_S / freq

        targets = [self._target_vars[i].get() for i in range(JOINT_COUNT)]
        for i in range(JOINT_COUNT):
            delta = targets[i] - self._commanded[i]
            if delta > max_step:
                delta = max_step
            elif delta < -max_step:
                delta = -max_step
            self._commanded[i] += delta

        self._tick_count += 1
        try:
            if self._enable_var.get():
                self._node.publish_positions(self._commanded, freq)
            self._success_count += 1
        except Exception as e:  # noqa: BLE001 - 流式发送要统计异常，不能让它中断循环
            self._status_var.set(f'发布失败: {e}')

        self._root.after(self._period_ms, self._send_tick)

    def _on_ui_tick(self):
        current = self._node.get_current_positions()
        if current is not None:
            for i in range(JOINT_COUNT):
                self._current_vars[i].set(f'{current[i]:.4f}')
            if self._commanded is None:
                # 第一次收到真实位置：同步滑块和发送基准，解锁控件
                self._commanded = list(current)
                for i in range(JOINT_COUNT):
                    self._target_vars[i].set(current[i])
                for s in self._sliders:
                    s.config(state='normal')
                self._start_btn.config(state='normal')
                self._sync_btn.config(state='normal')
                self._status_var.set('已获取真实关节状态，可以点击"开始"')

        if self._tick_count > 0:
            rate = self._success_count / self._tick_count * 100.0
            self._success_rate_var.set(f'{rate:.1f}%  ({self._success_count}/{self._tick_count})')

        elapsed = int(time.monotonic() - self._start_time)
        h, rem = divmod(elapsed, 3600)
        m, s = divmod(rem, 60)
        self._runtime_var.set(f'{h:02d}:{m:02d}:{s:02d}')

        self._root.after(UI_REFRESH_MS, self._on_ui_tick)

    def _on_close(self):
        self._streaming = False
        self._root.destroy()


def main():
    rclpy.init()
    node = JointGuiNode()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    root = tk.Tk()
    TeleopGui(root, node)
    try:
        root.mainloop()
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()


if __name__ == '__main__':
    main()
