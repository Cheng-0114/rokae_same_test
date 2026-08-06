#!/usr/bin/env bash
# 清理 arm_teleop 相关的残留 ROS2 进程。
#
# 用途：ros2_control_node 崩溃(比如真机连接失败导致 abort)后，
# bringup.launch.py 拉起的那一整棵进程树会自动跟着一起退出(见
# real_moveit.launch.py 里的 OnProcessExit 处理)，不需要再手动清理。
# 但节点 A(joint_input_node GUI)、节点 C(stress_test_node) 通常是在
# 单独的终端里用 `ros2 run` 启动的，不属于那棵 launch 树，崩溃或者终端被
# 强制关掉时不会被自动带走——这个脚本就是给这种情况兜底用的。
#
# 用法：
#   bash scripts/cleanup.sh          # 先列出匹配到的残留进程，确认后再杀
#   bash scripts/cleanup.sh -y       # 跳过确认，直接杀（脚本/CI 场景用）

set -u

FORCE=0
if [ "${1:-}" = "-y" ]; then
    FORCE=1
fi

PATTERNS=(
    "ros2 launch arm_teleop bringup.launch.py"
    "controller_manager/ros2_control_node"
    "robot_state_publisher/robot_state_publisher"
    "arm_teleop/lib/arm_teleop/trajectory_bridge_node"
    "arm_teleop/lib/arm_teleop/joint_input_node"
    "arm_teleop/lib/arm_teleop/stress_test_node"
)

echo "=== 匹配到的残留进程 ==="
found=0
for p in "${PATTERNS[@]}"; do
    pids=$(pgrep -f "$p" 2>/dev/null || true)
    if [ -n "$pids" ]; then
        found=1
        echo "--- 匹配 '$p' ---"
        # shellcheck disable=SC2086
        ps -o pid,etime,cmd -p $pids
    fi
done

if [ "$found" -eq 0 ]; then
    echo "没有找到残留进程，无需清理"
    exit 0
fi

if [ "$FORCE" -ne 1 ]; then
    echo
    read -r -p "确认要结束以上进程吗? [y/N] " confirm
    if [ "$confirm" != "y" ] && [ "$confirm" != "Y" ]; then
        echo "已取消"
        exit 0
    fi
fi

for p in "${PATTERNS[@]}"; do
    pkill -f "$p" 2>/dev/null || true
done
sleep 1
# 兜底：SIGTERM 后 1 秒还活着的话强制 SIGKILL
for p in "${PATTERNS[@]}"; do
    pkill -9 -f "$p" 2>/dev/null || true
done

echo "清理完成"
