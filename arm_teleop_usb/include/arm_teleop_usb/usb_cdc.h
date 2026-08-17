#ifndef USB_CDC_H
#define USB_CDC_H

#include <libusb-1.0/libusb.h>
#include <string>
#include <thread>
#include <atomic>
#include <mutex>
#include <queue>
#include <condition_variable>
#include <cstdint>

#define USB_VENDOR_ID       0x28e9      // 厂商ID
#define USB_PRODUCT_ID      0x018a      // 产品ID (0表示匹配任意产品)
#define USB_READ_TIMEOUT    1000        // 读取超时（毫秒）
#define USB_WRITE_TIMEOUT   1000        // 写入超时（毫秒）
#define USB_MAX_PACKET_SIZE 512         // 最大包大小

/* 接收回调: 当USB收到数据时调用, 在receiveLoop线程上下文中执行 */
typedef void (*usb_recv_callback_t)(const uint8_t *data, uint16_t len);

/**
 * USB CDC 设备管理类
 */
class USBCDCDevice {
public:
    USBCDCDevice();
    ~USBCDCDevice();

    /**
     * 启动热插拔监听
     */
    bool startListen();

    /**
     * 停止监听
     */
    void stop();

    /**
     * 发送二进制数据 (线程安全)
     * @param data  数据指针
     * @param len   数据长度(字节)
     */
    void sendBinary(const uint8_t *data, uint16_t len);

    /**
     * 发送字符串数据 (线程安全, 兼容旧接口)
     */
    void sendData(const std::string& data);

    /**
     * 设置接收数据回调
     * @param cb  回调函数指针 (NULL取消回调)
     *            在receiveLoop线程中调用, 需注意线程安全
     */
    void setRecvCallback(usb_recv_callback_t cb);

    /**
     * 开启/关闭 USB 收发调试输出 (默认: 开启)
     * 高速吞吐测试时应关闭, 避免 printf 拖慢链路
     */
    void setDebugOutput(bool enabled);

    /**
     * 查询设备是否已连接
     */
    bool isConnected() const;

    /**
     * 等待设备连接 (阻塞)
     * @param  timeout_ms  超时毫秒数, -1表示无限等待
     * @return true=已连接, false=超时
     */
    bool waitForConnection(int timeout_ms);

private:
    libusb_context* ctx;
    libusb_device_handle* handle;  // USB设备句柄
    struct libusb_device_descriptor dev_desc;
    uint8_t endpoint_in = 0;        // 批量 IN 端点(数据接收)
    uint8_t endpoint_out = 0;       // 批量 OUT 端点(数据发送)
    uint8_t endpoint_int = 0;       // 中断 IN 端点(状态通知)
    uint8_t nb_ifaces = 0;          // 接口数量

    //线程
    std::thread hotplug_thread;         // 事件循环线程
    std::thread recv_thread;            // 接收数据线程
    std::thread send_thread;            // 发送数据线程
    std::atomic<bool> running;          // 事件循环运行标志
    std::atomic<bool> recvrunning;      // 接收线程运行标志
    std::atomic<bool> sendrunning;      // 发送线程运行标志

    // 发送队列
    std::queue<std::string> send_queue;
    std::mutex send_mutex;
    std::condition_variable send_cv;

    // 连接状态
    std::atomic<bool> device_connected{false};
    std::mutex connect_mutex;
    std::condition_variable connect_cv;

    // 接收回调
    usb_recv_callback_t recv_callback = nullptr;

    // 调试输出开关 (默认关闭, 需要时通过 setDebugOutput(true) 开启)
    bool debug_output = false;

    static int hotplugCallback(libusb_context* ctx, libusb_device* dev,
                                libusb_hotplug_event event, void* user_data);
    void eventLoop();
    void receiveLoop();                     // 接收数据循环
    void sendLoop();                        // 发送数据循环
    bool openDevice(libusb_device* dev);    // 打开USB设备
    void closeDevice();                     // 关闭USB设备
};

#endif // USB_CDC_H
