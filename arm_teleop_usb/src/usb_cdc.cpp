/* 原样复制自 remote_control-8ee3e55/pc_usb/usb_cdc/usb_cdc.cpp，仅调整 include 路径，
 * 收发逻辑不做修改。 */
#include "arm_teleop_usb/usb_cdc.h"
#include <iostream>
#include <libusb-1.0/libusb.h>
#include <cstring>
#include <cstdio>
#include <chrono>

USBCDCDevice::USBCDCDevice()
    : ctx(NULL), handle(NULL), endpoint_in(0x81), endpoint_out(0x01), endpoint_int(0x82),
      nb_ifaces(2), running(false), recvrunning(false), sendrunning(false) {
}

USBCDCDevice::~USBCDCDevice() {
    stop();
}

bool USBCDCDevice::openDevice(libusb_device* dev) {
    libusb_open(dev, &handle);
    if (handle == NULL) {
        std::cout << "open fail" << std::endl;
        return false;
    }

    libusb_set_auto_detach_kernel_driver(handle, 1);

    /* 读取设备描述符获取接口数量 */
    libusb_get_device_descriptor(dev, &dev_desc);
    struct libusb_config_descriptor *config = NULL;
    int ret_cfg = libusb_get_active_config_descriptor(dev, &config);
    if (ret_cfg == 0 && config != NULL) {
        nb_ifaces = config->bNumInterfaces;
        printf("    bNumInterfaces = %d\n", nb_ifaces);

        /* 遍历接口和端点，发现批量 IN/OUT 和中断 IN 端点 */
        bool found_bulk_in = false, found_bulk_out = false, found_int_in = false;
        for (uint8_t i = 0; i < config->bNumInterfaces && !(found_bulk_in && found_bulk_out && found_int_in); i++) {
            const struct libusb_interface *iface = &config->interface[i];
            for (int alt = 0; alt < iface->num_altsetting; alt++) {
                const struct libusb_interface_descriptor *idesc = &iface->altsetting[alt];
                for (uint8_t ep = 0; ep < idesc->bNumEndpoints; ep++) {
                    const struct libusb_endpoint_descriptor *edesc = &idesc->endpoint[ep];
                    uint8_t addr = edesc->bEndpointAddress;
                    uint8_t attr = edesc->bmAttributes & 0x03;
                    bool is_in = (addr & LIBUSB_ENDPOINT_IN) != 0;

                    if (attr == LIBUSB_TRANSFER_TYPE_BULK && is_in && !found_bulk_in) {
                        endpoint_in = addr;
                        found_bulk_in = true;
                        printf("    found BULK IN  endpoint: 0x%02x (iface %d)\n", addr, i);
                    } else if (attr == LIBUSB_TRANSFER_TYPE_BULK && !is_in && !found_bulk_out) {
                        endpoint_out = addr;
                        found_bulk_out = true;
                        printf("    found BULK OUT endpoint: 0x%02x (iface %d)\n", addr, i);
                    } else if (attr == LIBUSB_TRANSFER_TYPE_INTERRUPT && is_in && !found_int_in) {
                        endpoint_int = addr;
                        found_int_in = true;
                        printf("    found INT  IN  endpoint: 0x%02x (iface %d)\n", addr, i);
                    }
                }
            }
        }
        libusb_free_config_descriptor(config);
    }

    /* 显式 detach 内核驱动, 确保 libusb 能独占接口.
     * libusb_set_auto_detach_kernel_driver 在某些内核版本可能不生效. */
    for (int i = 0; i < nb_ifaces; i++) {
        libusb_detach_kernel_driver(handle, i);
    }

    // 尝试 claim 接口,如果失败则强制 retry
    for (int attempt = 0; attempt < 2; attempt++) {
        bool claim_success = true;
        for (int i = 0; i < nb_ifaces; i++) {
            int ret = libusb_claim_interface(handle, i);
            if (ret == LIBUSB_SUCCESS) {
                printf("    interface %d Success\n", i);
            } else {
                printf("    interface %d Failed: %s\n", i, libusb_strerror((enum libusb_error)ret));
                claim_success = false;
            }
        }

        if (claim_success) break;

        if (attempt == 0) {
            printf("    Force detach kernel driver and retry...\n");
            for (int i = 0; i < nb_ifaces; i++) {
                libusb_detach_kernel_driver(handle, i);
            }
        }
    }

    /* CDC ACM 标准初始化序列:
     *   1. SET_LINE_CODING  - 设置波特率/数据位/停止位/校验位
     *   2. SET_CONTROL_LINE_STATE - DTR/RTS 置位, 通知从机主机已就绪
     * GD32 CDC 固件在 SET_CONTROL_LINE_STATE 之后 cl_state=1,
     * 但批量 OUT 端点需要 FreeRTOS 任务调度到 usb_rec_task()
     * 调用 cdc_acm_data_receive() 才会充能。这里加入延时等待. */
    {
        int ret;
        uint8_t line_coding[7] = {
            0x00, 0xC2, 0x01, 0x00,  // dwDTERate = 115200 (little-endian)
            0x00,                      // bCharFormat: 1 stop bit
            0x00,                      // bParityType: None
            0x08                       // bDataBits: 8
        };
        ret = libusb_control_transfer(handle,
            0x21, 0x20, 0x00, 0, line_coding, sizeof(line_coding), 1000);
        if (ret < 0)
            printf("SET_LINE_CODING failed: %s\n", libusb_strerror((enum libusb_error)ret));
        else
            printf("SET_LINE_CODING success (115200 8N1)\n");

        ret = libusb_control_transfer(handle,
            0x21, 0x22, 0x03, 0, NULL, 0, 1000);
        if (ret < 0)
            printf("SET_CONTROL_LINE_STATE failed: %s\n", libusb_strerror((enum libusb_error)ret));
        else
            printf("SET_CONTROL_LINE_STATE success (DTR=1, RTS=1)\n");
    }

    /* 等待 GD32 FreeRTOS 调度 usb_rec_task 充能 bulk OUT 端点.
     * GD32 任务优先级仅 tskIDLE_PRIORITY+1, 需要至少一个 tick
     * 的调度延迟 (tick 通常 1ms, 这里给 50ms 充分余量). */
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    /* 读取中断 IN 端点 (SERIAL_STATE 通知).
     * 部分 CDC ACM 设备在中断端点有挂起的通知时会影响批量端点行为. */
    if (endpoint_int != 0) {
        uint8_t int_buf[16];
        int transferred = 0;
        int ret2 = libusb_interrupt_transfer(handle, endpoint_int,
            int_buf, sizeof(int_buf), &transferred, 100);
        if (ret2 >= 0 && transferred > 0 && debug_output) {
            printf("[USB Int] %d bytes: ", transferred);
            for (int i = 0; i < transferred; i++)
                printf("%02x ", int_buf[i]);
            printf("\n");
        }
    }

    // 启动接收线程
    recvrunning = true;
    recv_thread = std::thread(&USBCDCDevice::receiveLoop, this);

    // 启动发送线程
    sendrunning = true;
    send_thread = std::thread(&USBCDCDevice::sendLoop, this);

    // 通知连接成功
    device_connected = true;
    connect_cv.notify_all();
    printf("Device connected and ready.\n");

    return true;
}

void USBCDCDevice::closeDevice() {
    if (handle == NULL) return;

    device_connected = false;

    // 停止收发线程
    if (recvrunning) {
        recvrunning = false;
        if (recv_thread.joinable()) {
            recv_thread.join();
        }
    }
    if (sendrunning) {
        sendrunning = false;
        send_cv.notify_all();  // 唤醒发送线程
        if (send_thread.joinable()) {
            send_thread.join();
        }
    }

    // 清空发送队列
    std::queue<std::string> empty;
    std::swap(send_queue, empty);

    for (int i = 0; i < nb_ifaces; i++) {
        libusb_release_interface(handle, i);
    }

    printf("Closing device...\n");
    libusb_close(handle);
    handle = NULL;
}

void USBCDCDevice::stop() {
    // 停止事件循环
    if (running) {
        running = false;
        if (hotplug_thread.joinable()) {
            hotplug_thread.join();
        }
    }

    // 先通知从机断开(仅当 handle 有效时)
    if (handle) {
        int ret = libusb_control_transfer(
            handle,
            0x21,              // bmRequestType: Class, Interface, Host-to-Device
            0x22,              // bRequest: SET_CONTROL_LINE_STATE
            0x00,              // wValue: DTR=0, RTS=0
            0,                 // wIndex: 接口0
            NULL, 0, 1000
        );
        if (ret < 0) {
            printf("SET_CONTROL_LINE_STATE (disconnect) failed: %s\n", libusb_strerror((enum libusb_error)ret));
        } else {
            printf("SET_CONTROL_LINE_STATE (disconnect) success\n");
        }
    }

    // 关闭设备(会停止收发线程)
    closeDevice();

    // 释放 libusb
    if (ctx) {
        libusb_exit(ctx);
        ctx = NULL;
    }
}

// 热插拔回调
int USBCDCDevice::hotplugCallback(libusb_context*, libusb_device* dev,
                                   libusb_hotplug_event event, void* user_data) {
    auto* device = static_cast<USBCDCDevice*>(user_data);

    struct libusb_device_descriptor desc;
    int rc = libusb_get_device_descriptor(dev, &desc);

    if (LIBUSB_SUCCESS == rc) {
        printf("Device attached: %04x:%04x\n", desc.idVendor, desc.idProduct);
    }

    if (event == LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED) {
        std::cout << "检测到设备插入" << std::endl;
        device->openDevice(dev);
    } else if (event == LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT) {
        std::cout << "检测到设备拔出" << std::endl;
        device->closeDevice();
    }

    return 0;
}

bool USBCDCDevice::startListen() {
    // 初始化 libusb
    if (libusb_init(&ctx) < 0) {
        std::cerr << "无法初始化 libusb" << std::endl;
        return false;
    }

    // 检查是否支持热插拔
    if (!libusb_has_capability(LIBUSB_CAP_HAS_HOTPLUG)) {
        std::cerr << "不支持热插拔" << std::endl;
        libusb_exit(ctx);
        ctx = NULL;
        return false;
    }

    // 注册热插拔回调
    int ret = libusb_hotplug_register_callback(
        ctx,
        (libusb_hotplug_event)(LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED |
                               LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT),
        LIBUSB_HOTPLUG_ENUMERATE,
        USB_VENDOR_ID,
        USB_PRODUCT_ID,
        LIBUSB_HOTPLUG_MATCH_ANY,
        hotplugCallback,
        this,
        NULL
    );

    if (ret < 0) {
        std::cerr << "注册热插拔失败" << std::endl;
        libusb_exit(ctx);
        ctx = NULL;
        return false;
    }

    // 启动事件循环线程
    running = true;
    hotplug_thread = std::thread(&USBCDCDevice::eventLoop, this);

    return true;
}

void USBCDCDevice::eventLoop() {
    while (running) {
        struct timeval tv = {1, 0};
        libusb_handle_events_timeout(ctx, &tv);
    }
}

void USBCDCDevice::receiveLoop() {
    unsigned char buffer[USB_MAX_PACKET_SIZE];

    while (recvrunning) {
        int actual_length = 0;
        int ret = libusb_bulk_transfer(
            handle,
            endpoint_in,       // 0x81 批量 IN 端点
            buffer,
            sizeof(buffer),
            &actual_length,
            100                // timeout (ms)
        );

        if (ret == LIBUSB_ERROR_TIMEOUT) {
            continue;  // 超时继续等待
        }

        if (ret < 0) {
            printf("Recv failed: %s\n", libusb_strerror((enum libusb_error)ret));
            continue;
        }

        if (actual_length > 0) {
            if (debug_output) {
                printf("[USB Recv] %d bytes: ", actual_length);
                for (int i = 0; i < actual_length && i < 64; i++) {
                    printf("%02x ", buffer[i]);
                }
                if (actual_length > 64) printf("...");
                printf("\n");
            }

            // 调用上层回调 (协议解析)
            if (recv_callback) {
                recv_callback(buffer, (uint16_t)actual_length);
            }
        }
    }
}

void USBCDCDevice::sendLoop() {
    while (sendrunning) {
        std::string data;

        // 等待队列中有数据
        {
            std::unique_lock<std::mutex> lock(send_mutex);
            send_cv.wait(lock, [this] { return !send_queue.empty() || !sendrunning; });

            if (!sendrunning) break;

            if (!send_queue.empty()) {
                data = send_queue.front();
                send_queue.pop();
            }
        }

        // 发送数据 (超时时自动重试, 给 GD32 端点初始化留出时间)
        if (!data.empty() && handle != NULL) {
            int actual_length = 0;
            int ret = LIBUSB_ERROR_TIMEOUT;

            for (int attempt = 0; attempt < 5 && ret == LIBUSB_ERROR_TIMEOUT; attempt++) {
                ret = libusb_bulk_transfer(
                    handle,
                    endpoint_out,
                    (unsigned char*)data.c_str(),
                    data.length(),
                    &actual_length,
                    1000
                );

                if (ret == LIBUSB_ERROR_TIMEOUT && attempt < 4) {
                    if (debug_output)
                        printf("Send timeout, retry %d/5...\n", attempt + 1);
                    std::this_thread::sleep_for(std::chrono::milliseconds(200));
                }
            }

            if (ret < 0) {
                printf("Send failed: %s\n", libusb_strerror((enum libusb_error)ret));
            } else {
                if (debug_output) {
                    printf("[USB Send] %d bytes: ", actual_length);
                    for (int i = 0; i < actual_length && i < 32; i++) {
                        printf("%02x ", (unsigned char)data[i]);
                    }
                    if (actual_length > 32) printf("...");
                    printf("\n");
                }
            }
        }
    }
}

/* ==================== 新增接口实现 ==================== */

void USBCDCDevice::sendBinary(const uint8_t *data, uint16_t len) {
    if (!sendrunning || !data || len == 0) return;

    /* 使用带长度的string构造函数, 支持二进制数据(含\0) */
    std::string binary_data((const char*)data, len);
    sendData(binary_data);
}

void USBCDCDevice::sendData(const std::string& data) {
    if (!sendrunning) return;  // 线程没运行就不入队

    {
        std::lock_guard<std::mutex> lock(send_mutex);
        send_queue.push(data);
    }
    send_cv.notify_one();  // 通知发送线程有新数据
}

void USBCDCDevice::setRecvCallback(usb_recv_callback_t cb) {
    recv_callback = cb;
}

void USBCDCDevice::setDebugOutput(bool enabled) {
    debug_output = enabled;
}

bool USBCDCDevice::isConnected() const {
    return device_connected && handle != NULL;
}

bool USBCDCDevice::waitForConnection(int timeout_ms) {
    std::unique_lock<std::mutex> lock(connect_mutex);

    if (device_connected) {
        return true;  // 已经连接
    }

    if (timeout_ms < 0) {
        /* 无限等待 */
        connect_cv.wait(lock, [this] { return device_connected.load(); });
        return device_connected;
    } else {
        /* 限时等待 */
        auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
        while (!device_connected) {
            if (connect_cv.wait_until(lock, deadline) == std::cv_status::timeout) {
                return false;  // 超时
            }
        }
        return true;
    }
}
