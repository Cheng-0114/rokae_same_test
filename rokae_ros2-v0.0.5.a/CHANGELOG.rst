^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Rokae ROS 2 软件包变更记录
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

本文件为 **rokae_ros2 软件包** 的唯一发版变更说明。
各功能包版本号与本软件包版本对齐，不再在各功能包内维护 CHANGELOG。

0.0.5 (2026-07-24)
------------------
* 新增协作机型适配：AR3L、AR3R、AR5L08、AR5R08（七轴 `xMateErProRobot`）。
  - 型号映射：AR5R08 → ``AR5-5_08R-W4C1C5``，AR5L08 → ``AR5-5_08L-W4C1C5``，
    AR3R → ``AR3-3_07R-W4C5C5``，AR3L → ``AR3-3_07L-W4C5C5``（与既有 AR5R/AR5L 互不影响）。
* 新增工业标准机型适配：XB7s、XB7h、NB25s、NB25h、NB12s、NB12h、EB4、NB4、XB4s、XB4h。
  - 真机硬件接口使用 SDK ``StandardRobot`` / ``RtMotionControlIndustrial<6>``；
    通过 ``robot_class:=standard`` 与 ``xMate_type`` / ``robot_type`` 选择机型。
  - **说明：工业标准机型的 Gazebo 仿真功能暂时没有适配。**
* 同步提供上述机型的 URDF/xacro、ros2_control、mesh、RViz、控制器 yaml 及对应 MoveIt 2 配置包。
* 更新 ``rokae_hardware`` 驱动与硬件接口、统一 launch（含 ``moveit_planning_utils``），
  以及使用手册中的机型说明与启动示例。
* 文档：使用手册补充珞石在线文档入口 https://docs.rokae.com/docs/ROS2 。
* 文档：使用手册补充 **PREEMPT_RT 实时内核**的作用、安装步骤与内核验证操作。
* 多机型 MoveIt ``joint_limits`` 默认速度/加速度缩放参数调整。

0.0.4 (2026-05-15)
------------------
* 新增 Gazebo 仿真支持（``rokae_gazebo``、description 相关 launch 与资源路径）。

0.0.3 (2026-05-06)
------------------
* 适配 xMate CR35（URDF/ros2_control、控制器、硬件接口、MoveIt 配置与文档）。

0.0.2 (2026-04-10)
------------------
* 统一软件包栈版本为 0.0.2，并整理各功能包版本对齐。
