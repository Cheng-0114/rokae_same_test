/**
 * 节点 D：主臂 USB 电机反馈桥接节点。
 *
 * 只负责"USB <-> ROS"这一层转换，不做电机控制逻辑：
 * - 启动后自动 CMD_CONNECT 建立协议连接、维持 1Hz 心跳，让固件愿意把下行数据发出来
 *   （固件侧证据：task_usb_send.c 里 usb_cdc_get_connected()==false 时会直接丢弃所有
 *   下行数据，包括反馈帧）。
 * - 不自动发送 CMD_DM_ENABLE：电机使能是会让主臂真的通电出力的动作，交给
 *   ~/enable_motors 服务显式触发，不能因为"起了这个节点"就自动生效。
 * - 收到 CMD_DM_FB 帧就整帧打包成一条 sensor_msgs/JointState 发布，
 *   name="joint{id}", position=pos, velocity=speed, effort=torque。
 */
#include <cstring>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "arm_teleop_usb/protocol.h"
#include "arm_teleop_usb/usb_cdc.h"

#include <libusb-1.0/libusb.h>

class UsbMotorBridgeNode;

namespace {

/* 单帧最多容纳的电机数，超过则截断并告警（正常主臂关节数远小于此值） */
constexpr uint8_t kMaxJointsPerFrame = 16;

/* USBCDCDevice::setRecvCallback 只接受裸函数指针，没有 user_data 参数，
 * 用静态指针把回调转发回节点实例（本节点每进程只应实例化一个）。 */
UsbMotorBridgeNode *g_node_for_callback = nullptr;
void onUsbRecvTrampoline(const uint8_t *data, uint16_t len);

}  // namespace

class UsbMotorBridgeNode : public rclcpp::Node {
public:
  UsbMotorBridgeNode() : rclcpp::Node("usb_motor_bridge_node") {
    joint_state_topic_ = declare_parameter<std::string>(
        "joint_state_topic", "/arm_teleop/master_joint_state");
    const double heartbeat_period_s = declare_parameter<double>("heartbeat_period_s", 1.0);
    const bool usb_debug_output = declare_parameter<bool>("usb_debug_output", false);

    pub_ = create_publisher<sensor_msgs::msg::JointState>(joint_state_topic_, 10);
    enable_srv_ = create_service<std_srvs::srv::SetBool>(
        "~/enable_motors",
        std::bind(&UsbMotorBridgeNode::onEnableMotors, this,
                  std::placeholders::_1, std::placeholders::_2));

    std::memset(&recv_buf_, 0, sizeof(recv_buf_));

    device_.setDebugOutput(usb_debug_output);
    device_.setRecvCallback(&onUsbRecvTrampoline);
    g_node_for_callback = this;

    RCLCPP_INFO(get_logger(), "启动前先扫描一次USB设备:");
    scanUsbDevices();

    if (!device_.startListen()) {
      RCLCPP_FATAL(get_logger(), "USB 热插拔监听启动失败，请检查 libusb 权限/udev 规则");
      throw std::runtime_error("USBCDCDevice::startListen failed");
    }

    link_timer_ = create_wall_timer(
        std::chrono::milliseconds(200), std::bind(&UsbMotorBridgeNode::onLinkTimer, this));
    heartbeat_timer_ = create_wall_timer(
        std::chrono::duration<double>(heartbeat_period_s),
        std::bind(&UsbMotorBridgeNode::onHeartbeatTimer, this));
    scan_timer_ = create_wall_timer(
        std::chrono::seconds(3), std::bind(&UsbMotorBridgeNode::onScanTimer, this));

    RCLCPP_INFO(get_logger(), "节点D已启动: 发布话题=%s, 等待 USB 设备连接(VID=%04x PID=%04x)...",
                joint_state_topic_.c_str(), USB_VENDOR_ID, USB_PRODUCT_ID);
  }

  ~UsbMotorBridgeNode() override {
    if (device_.isConnected()) {
      uint8_t buf[32];
      uint16_t n = protocol_build_connect(0, buf, sizeof(buf));
      if (n > 0) {
        device_.sendBinary(buf, n);
      }
    }
    device_.stop();
    g_node_for_callback = nullptr;
  }

  /* 在 USBCDCDevice 的 receiveLoop 线程上下文中调用 */
  void onUsbRecv(const uint8_t *data, uint16_t len) {
    uint16_t remaining = PROTOCOL_RECV_BUF_SIZE - recv_buf_.wp;
    if (len > remaining) {
      RCLCPP_WARN(get_logger(), "接收缓冲区溢出，丢弃 %u 字节", len);
      return;
    }
    std::memcpy(&recv_buf_.buf[recv_buf_.wp], data, len);
    recv_buf_.wp += len;

    for (;;) {
      int32_t msg_len = static_cast<int32_t>(recv_buf_.wp) - static_cast<int32_t>(recv_buf_.rp);
      if (msg_len <= 0) {
        break;
      }

      uint32_t ret = search_one_frame(&recv_buf_.buf[recv_buf_.rp], static_cast<uint16_t>(msg_len));
      uint16_t parsed_len = ret & ~FRAME_SHIELD;

      switch (ret & FRAME_SHIELD) {
        case FRAME_OK: {
          const auto *f = reinterpret_cast<const struct frame *>(&recv_buf_.buf[recv_buf_.rp]);
          dispatchFrame(f);
          recv_buf_.rp += parsed_len;
          break;
        }
        case FRAME_ERR:
          recv_buf_.rp += (parsed_len == 0) ? 1 : parsed_len;
          break;
        case FRAME_LESS:
        default:
          goto parse_done;
      }
    }

  parse_done:
    if (recv_buf_.rp >= recv_buf_.wp) {
      recv_buf_.rp = 0;
      recv_buf_.wp = 0;
    } else if (recv_buf_.rp > 0) {
      neaten_comm_buffer(&recv_buf_);
    }
  }

private:
  void dispatchFrame(const struct frame *f) {
    switch (f->msg) {
      case CMD_DM_FB:
        handleDmFbFrame(f->payload, f->length);
        break;
      case CMD_ACK:
        if (f->length >= 1) {
          RCLCPP_DEBUG(get_logger(), "收到 ACK，应答命令=0x%02x", f->payload[0]);
        }
        break;
      default:
        RCLCPP_DEBUG(get_logger(), "收到未处理的消息类型 0x%02x", f->msg);
        break;
    }
  }

  void handleDmFbFrame(const uint8_t *payload, uint16_t payload_len) {
    if (payload_len < 1) {
      return;
    }
    uint8_t count = payload[0];
    uint16_t expected = 1 + static_cast<uint16_t>(count) * sizeof(struct dm_joint_fb_info);
    if (payload_len < expected) {
      RCLCPP_WARN(get_logger(), "CMD_DM_FB 载荷长度不足: got=%u expected=%u", payload_len, expected);
      return;
    }

    uint8_t n = count;
    if (n > kMaxJointsPerFrame) {
      RCLCPP_WARN(get_logger(), "CMD_DM_FB 电机数 %u 超过上限 %u，已截断", count, kMaxJointsPerFrame);
      n = kMaxJointsPerFrame;
    }

    /* 用 memcpy 拷进本地对齐好的数组，避免直接 reinterpret_cast 未对齐的 payload 指针 */
    struct dm_joint_fb_info joints[kMaxJointsPerFrame];
    std::memcpy(joints, &payload[1], static_cast<size_t>(n) * sizeof(struct dm_joint_fb_info));

    sensor_msgs::msg::JointState msg;
    msg.header.stamp = now();
    msg.name.reserve(n);
    msg.position.reserve(n);
    msg.velocity.reserve(n);
    msg.effort.reserve(n);
    for (uint8_t i = 0; i < n; i++) {
      msg.name.push_back("joint" + std::to_string(joints[i].id));
      msg.position.push_back(joints[i].pos);
      msg.velocity.push_back(joints[i].speed);
      msg.effort.push_back(joints[i].torque);
    }
    pub_->publish(msg);
  }

  /* 主动扫描当前所有 USB 设备，找出 VID=GigaDevice(0x28e9) 的设备，逐个报告
   * PID/产品名，并指出哪个才是目标 CDC_ACM 通信口——不是被动等热插拔事件。
   * 用途：这块板子实测会在两种USB身份间切换(通信模式 018a / 调试器模式等
   * 其它PID)，光等 018a 的热插拔事件，设备插着但处于错误模式时完全看不出
   *线索；扫描能把"设备在，但不是目标身份"和"设备根本不在"这两种情况区分开。
   * 用独立的 libusb_context，不影响 USBCDCDevice 自己内部的连接状态。 */
  void scanUsbDevices() {
    libusb_context *scan_ctx = nullptr;
    if (libusb_init(&scan_ctx) < 0) {
      RCLCPP_WARN(get_logger(), "USB扫描: libusb_init 失败，跳过本次扫描");
      return;
    }

    libusb_device **list = nullptr;
    ssize_t count = libusb_get_device_list(scan_ctx, &list);
    bool found_target = false;
    int gigadevice_count = 0;

    for (ssize_t i = 0; i < count; i++) {
      libusb_device *dev = list[i];
      struct libusb_device_descriptor desc;
      if (libusb_get_device_descriptor(dev, &desc) != 0) {
        continue;
      }
      if (desc.idVendor != USB_VENDOR_ID) {
        continue;
      }
      gigadevice_count++;

      std::string product = "(无法读取产品名)";
      libusb_device_handle *handle = nullptr;
      if (libusb_open(dev, &handle) == 0) {
        unsigned char buf[256];
        if (desc.iProduct != 0 &&
            libusb_get_string_descriptor_ascii(handle, desc.iProduct, buf, sizeof(buf)) > 0) {
          product = reinterpret_cast<char *>(buf);
        }
        libusb_close(handle);
      }

      const bool is_target = (desc.idProduct == USB_PRODUCT_ID);
      if (is_target) {
        found_target = true;
      }
      RCLCPP_INFO(get_logger(), "USB扫描: 发现 GigaDevice 设备 PID=0x%04x 产品=\"%s\" %s",
                  desc.idProduct, product.c_str(),
                  is_target ? "← 就是目标 GD32-CDC_ACM" : "(不是目标设备)");
    }

    if (count >= 0) {
      libusb_free_device_list(list, 1);
    }
    libusb_exit(scan_ctx);

    if (!found_target) {
      if (gigadevice_count > 0) {
        RCLCPP_WARN(get_logger(),
            "USB扫描: 找到 %d 个 GigaDevice 设备，但没有一个是目标 CDC_ACM(VID=0x%04x "
            "PID=0x%04x)——板子大概率插着但处于别的模式(比如调试器模式)，不是通信模式",
            gigadevice_count, USB_VENDOR_ID, USB_PRODUCT_ID);
      } else {
        RCLCPP_WARN(get_logger(), "USB扫描: 一个 GigaDevice(VID=0x%04x) 设备都没找到，检查线缆/供电",
                    USB_VENDOR_ID);
      }
    }
  }

  void onLinkTimer() {
    const bool connected = device_.isConnected();
    if (connected && !was_connected_) {
      RCLCPP_INFO(get_logger(), "USB 设备已连接，发送 CMD_CONNECT 建立协议连接");
      uint8_t buf[32];
      uint16_t n = protocol_build_connect(1, buf, sizeof(buf));
      if (n > 0) {
        device_.sendBinary(buf, n);
      }
      was_connected_ = true;
    } else if (!connected && was_connected_) {
      RCLCPP_WARN(get_logger(), "USB 设备已断开");
      was_connected_ = false;
    }
  }

  /* 只在还没连上时才周期性重新扫描打日志，连上之后没必要一直扫、刷屏 */
  void onScanTimer() {
    if (device_.isConnected()) {
      return;
    }
    scanUsbDevices();
  }

  void onHeartbeatTimer() {
    if (!device_.isConnected()) {
      return;
    }
    uint8_t buf[32];
    uint16_t n = protocol_build_hb(hb_counter_++, buf, sizeof(buf));
    if (n > 0) {
      device_.sendBinary(buf, n);
    }
  }

  void onEnableMotors(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                       std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
    if (!device_.isConnected()) {
      res->success = false;
      res->message = "USB 设备未连接";
      return;
    }
    uint8_t buf[32];
    uint16_t n = protocol_build_dm_enable(req->data ? 1 : 0, buf, sizeof(buf));
    if (n == 0) {
      res->success = false;
      res->message = "构建 CMD_DM_ENABLE 帧失败";
      return;
    }
    device_.sendBinary(buf, n);
    res->success = true;
    res->message = req->data ? "已发送电机使能请求" : "已发送电机失能请求";
    RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
  }

  USBCDCDevice device_;
  struct protocol_buf recv_buf_;
  std::string joint_state_topic_;
  bool was_connected_ = false;
  uint16_t hb_counter_ = 0;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_srv_;
  rclcpp::TimerBase::SharedPtr link_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  rclcpp::TimerBase::SharedPtr scan_timer_;
};

namespace {
void onUsbRecvTrampoline(const uint8_t *data, uint16_t len) {
  if (g_node_for_callback) {
    g_node_for_callback->onUsbRecv(data, len);
  }
}
}  // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  {
    auto node = std::make_shared<UsbMotorBridgeNode>();
    rclcpp::spin(node);
  }
  rclcpp::shutdown();
  return 0;
}
