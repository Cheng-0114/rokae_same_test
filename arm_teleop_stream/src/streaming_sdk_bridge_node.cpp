/**
 * streaming 模式桥接节点：绕过 ros2_control/controller_manager，直接用 rokae SDK
 * 做连续流式关节位置控制。
 *
 * 背景：bringup.launch.py 原本设计成 control_mode:=streaming 时改走
 * streaming_position_controller(ros2_control 里的 JointGroupPositionController)，
 * 但现在这份 rokae_ros2 源码里 real_moveit.launch.py 从没实现过这个控制器的
 * spawn 逻辑（也没在 controllers.yaml 里定义），streaming 模式实际根本跑不通。
 * 这个节点是绕开那条路径的替代方案：参考 rokae_example 的 servoj_demo.cpp，
 * 节点自己直接持有 SDK 的机器人连接，按固定周期把收到的目标关节角度喂给
 * rt_con->sendCommand()，不经过 ros2_control。
 *
 * 重要：这个节点自己独占一条到机器人控制器的 RT 连接，**不能**跟
 * bringup.launch.py(会另起一条 ros2_control 的连接)同时对同一台机械臂运行，
 * 会连接冲突。streaming 模式请只用这个节点，不要再起 bringup.launch.py。
 *
 * 话题契约延用现有协议：订阅 /arm_teleop/joint_command
 * (std_msgs/Float64MultiArray: 7 个关节位置 + 1 个占位数，占位数本节点忽略)，
 * 发布 /arm_teleop/joint_state(sensor_msgs/JointState) 真实关节状态，
 * 跟节点A/节点B在 trajectory 模式下的话题名完全一致。
 *
 * 安全设计：
 * - 连接成功后先读一次真实关节位置作为初始目标"悬停"在原地，不做 MoveJ，
 *   避免上电瞬间往预设点跳变；收到第一条外部指令后才切换成跟随外部目标。
 * - 没有做任何限速/限幅——安全完全依赖发送端(节点A的滑块限速)，这点跟
 *   bringup.launch.py 文档里对 streaming 模式的安全说明一致。
 */
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <rokae/data_types.h>
#include <rokae/motion_control_rt.h>
#include <rokae/robot.h>

#include <cinttypes>
#include <cstdint>

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace {
constexpr size_t kJointCount = 7;
}

class StreamingSdkBridgeNode : public rclcpp::Node {
public:
  StreamingSdkBridgeNode() : rclcpp::Node("streaming_sdk_bridge_node") {
    robot_ip_ = declare_parameter<std::string>("robot_ip", "192.168.9.160");
    local_ip_ = declare_parameter<std::string>("local_ip", "192.168.9.50");
    command_topic_ = declare_parameter<std::string>("command_topic", "/arm_teleop/joint_command");
    joint_state_topic_ = declare_parameter<std::string>("joint_state_topic", "/arm_teleop/joint_state");
    joint_names_ = declare_parameter<std::vector<std::string>>(
        "joint_names",
        std::vector<std::string>{"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"});
    cycle_ms_ = declare_parameter<int>("cycle_ms", 20);
    servoj_lookahead_s_ = declare_parameter<double>("servoj_lookahead_s", 0.06);
    servoj_kp_ = declare_parameter<double>("servoj_kp", 1.0);

    if (joint_names_.size() != kJointCount) {
      throw std::runtime_error("joint_names 参数长度必须是 7");
    }
    if (cycle_ms_ <= 0) {
      throw std::runtime_error("cycle_ms 必须 > 0");
    }

    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(joint_state_topic_, 10);
    command_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        command_topic_, 10,
        std::bind(&StreamingSdkBridgeNode::onCommand, this, std::placeholders::_1));

    if (!connectAndStartStreaming()) {
      throw std::runtime_error("连接机器人或启动 streaming 失败，详见上面的错误日志");
    }

    cycle_timer_ = create_wall_timer(
        std::chrono::milliseconds(cycle_ms_),
        std::bind(&StreamingSdkBridgeNode::onCycle, this));

    RCLCPP_INFO(get_logger(),
        "节点已启动: 订阅=%s 发布=%s cycle=%dms(直连SDK，不经过ros2_control)",
        command_topic_.c_str(), joint_state_topic_.c_str(), cycle_ms_);
  }

  ~StreamingSdkBridgeNode() override {
    if (rt_con_) {
      rt_con_->stopServoJoint();
      rt_con_->stopMove();
    }
    if (robot_) {
      std::error_code ec;
      robot_->setMotionControlMode(rokae::MotionControlMode::Idle, ec);
      robot_->setPowerState(false, ec);
    }
  }

private:
  bool connectAndStartStreaming() {
    std::error_code ec;
    robot_ = std::make_shared<rokae::xMateErProRobot>(robot_ip_, local_ip_);

    robot_->connectToRobot(ec);
    if (ec) {
      RCLCPP_ERROR(get_logger(), "连接机器人失败: %s", ec.message().c_str());
      return false;
    }
    RCLCPP_INFO(get_logger(), "机器人连接成功: robot_ip=%s local_ip=%s",
                robot_ip_.c_str(), local_ip_.c_str());

    robot_->setOperateMode(rokae::OperateMode::automatic, ec);
    if (ec) {
      RCLCPP_ERROR(get_logger(), "设置自动模式失败: %s", ec.message().c_str());
      return false;
    }

    robot_->setMotionControlMode(rokae::MotionControlMode::RtCommand, ec);
    if (ec) {
      RCLCPP_ERROR(get_logger(), "切换实时模式失败: %s", ec.message().c_str());
      return false;
    }

    robot_->setPowerState(true, ec);
    if (ec) {
      RCLCPP_ERROR(get_logger(), "上电失败: %s", ec.message().c_str());
      return false;
    }

    rt_con_ = robot_->getRtMotionController().lock();
    if (!rt_con_) {
      RCLCPP_ERROR(get_logger(), "获取实时控制器失败");
      return false;
    }

    robot_->startReceiveRobotState(
        std::chrono::milliseconds(1),
        {rokae::RtSupportedFields::jointPos_m, rokae::RtSupportedFields::jointVel_m});

    // 读一次真实关节位置做起点，不做 MoveJ，避免上电瞬间往预设点跳变；
    // 刚建立 RT 状态流可能要等一两个周期才有数据，重试几次。
    std::vector<double> current;
    bool got_initial = false;
    for (int attempt = 0; attempt < 20 && !got_initial; attempt++) {
      int ret = robot_->getStateData(rokae::RtSupportedFields::jointPos_m, current);
      if (ret == 0 && current.size() == kJointCount) {
        got_initial = true;
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    if (!got_initial) {
      RCLCPP_ERROR(get_logger(), "读取初始关节位置失败(重试20次仍无有效数据)");
      return false;
    }
    {
      std::lock_guard<std::mutex> lock(target_mutex_);
      latest_target_ = current;
      have_target_ = true;
    }

    const double cycle_s = static_cast<double>(cycle_ms_) / 1000.0;
    rt_con_->setServoJoint(cycle_s, servoj_lookahead_s_, servoj_kp_, ec);
    if (ec) {
      RCLCPP_WARN(get_logger(), "setServoJoint 失败: %s，仅用 startMove 继续", ec.message().c_str());
    }
    rt_con_->startMove(rokae::RtControllerMode::jointPosition);
    RCLCPP_INFO(get_logger(), "streaming 已启动，先悬停在当前真实位置，等待外部指令");
    return true;
  }

  void onCommand(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg->data.size() < kJointCount) {
      RCLCPP_ERROR(get_logger(), "收到 %zu 个数，期望至少 %zu 个关节角度，已丢弃",
                   msg->data.size(), kJointCount);
      return;
    }
    std::lock_guard<std::mutex> lock(target_mutex_);
    for (size_t i = 0; i < kJointCount; i++) {
      latest_target_[i] = msg->data[i];
    }
    have_target_ = true;
  }

  void onCycle() {
    std::vector<double> target;
    {
      std::lock_guard<std::mutex> lock(target_mutex_);
      if (!have_target_) {
        return;
      }
      target = latest_target_;
    }

    try {
      rokae::JointPosition cmd(target);
      rt_con_->sendCommand(cmd);
    } catch (const rokae::RealtimeMotionException &e) {
      RCLCPP_ERROR(get_logger(), "sendCommand 实时异常: %s", e.what());
    }

    /* getStateData() 只读 SDK 内部状态缓冲区，不会自己刷新——必须先调
     * updateRobotState() 把缓冲区从 RT 状态流里"泵"新一次，否则会一直拿到
     * 建连接那一刻的旧快照(getStateData 仍然返回成功，只是数据不新鲜)。
     * 这个坑是照着 rokae_hardware_interface.cpp::read() 里的用法补的。 */
    try {
      robot_->updateRobotState(std::chrono::milliseconds(1));
    } catch (const rokae::RealtimeMotionException &e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
          "updateRobotState 实时异常: %s", e.what());
      return;
    }

    std::vector<double> pos, vel;
    int ret_pos = robot_->getStateData(rokae::RtSupportedFields::jointPos_m, pos);
    if (ret_pos != 0 || pos.size() != kJointCount) {
      state_read_fail_count_++;
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
          "getStateData(jointPos_m) 失败: ret=%d size=%zu(期望%zu)，"
          "已连续失败%" PRIu64 "次，本周期不发布 joint_state",
          ret_pos, pos.size(), kJointCount, state_read_fail_count_);
      return;
    }
    if (state_read_fail_count_ > 0) {
      RCLCPP_WARN(get_logger(), "getStateData 恢复正常(此前连续失败%" PRIu64 "次)",
                  state_read_fail_count_);
      state_read_fail_count_ = 0;
    }
    int ret_vel = robot_->getStateData(rokae::RtSupportedFields::jointVel_m, vel);

    sensor_msgs::msg::JointState state_msg;
    state_msg.header.stamp = now();
    state_msg.name = joint_names_;
    state_msg.position = pos;
    if (ret_vel == 0 && vel.size() == kJointCount) {
      state_msg.velocity = vel;
    }
    joint_state_pub_->publish(state_msg);
  }

  std::string robot_ip_;
  std::string local_ip_;
  std::string command_topic_;
  std::string joint_state_topic_;
  std::vector<std::string> joint_names_;
  int cycle_ms_ = 20;
  double servoj_lookahead_s_ = 0.06;
  double servoj_kp_ = 1.0;

  std::shared_ptr<rokae::xMateErProRobot> robot_;
  std::shared_ptr<rokae::RtMotionControlCobot<kJointCount>> rt_con_;

  std::mutex target_mutex_;
  std::vector<double> latest_target_ = std::vector<double>(kJointCount, 0.0);
  bool have_target_ = false;
  uint64_t state_read_fail_count_ = 0;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr command_sub_;
  rclcpp::TimerBase::SharedPtr cycle_timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  int ret = 0;
  try {
    auto node = std::make_shared<StreamingSdkBridgeNode>();
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("streaming_sdk_bridge_node"), "初始化失败: %s", e.what());
    ret = 1;
  }
  rclcpp::shutdown();
  return ret;
}
