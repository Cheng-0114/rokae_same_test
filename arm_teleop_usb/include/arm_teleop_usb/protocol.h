/**
 * @file    protocol.h
 * @brief   PC端协议层 
 *          帧格式: STXA(1) STXB(1) SEQ(1) MSG(1) LEN(2) PAYLOAD(n) CRC16(2)
 *          CRC16-CCITT, 种子0xFFFF, 校验范围 seq~payload
 */

#ifndef _PROTOCOL_PC_H_
#define _PROTOCOL_PC_H_

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== 帧头定义 ==================== */
#define STXA                    0xAA
#define STXB                    0x55
#define FRAME_MIN_SIZE          0x08        /* 帧最小长度: stxa+stxb+seq+msg+len+crc */
#define PROTOCOL_RECV_BUF_SIZE  1024        /* 接收缓冲区大小 */
#define PROTOCOL_SEND_BUF_SIZE  512         /* 发送缓冲区大小 */

/* ==================== 帧解析返回值 ==================== */
#define FRAME_SHIELD            0xFFFF0000
#define FRAME_OK                0x00010000
#define FRAME_ERR               0x00020000
#define FRAME_LESS              0x00030000

/* ==================== 字节序宏 ==================== */
#define LOBYTE(w)               ((uint8_t)(w))
#define HIBYTE(w)               ((uint8_t)((uint16_t)(w) >> 8))
#define MAKEWORD(low, high)     ((uint16_t)((uint8_t)(low) | (((uint16_t)((uint8_t)(high))) << 8)))

/* ==================== 命令字 ==================== */
enum CMD {
    CMD_ACK             = 0x00,     /* 应答 */
    CMD_HB              = 0x01,     /* 心跳 (双向, 1s周期) */
    CMD_CONNECT         = 0x02,     /* 连接/断开 */
    CMD_DM_PARAM_REQ    = 0x10,     /* DM电机参数请求 */
    CMD_DM_ENABLE       = 0x11,     /* DM电机使能 */
    CMD_DM_CTRL         = 0x12,     /* DM电机MIT控制 */
    CMD_DM_FB           = 0x13,     /* DM电机反馈 (MCU -> PC) */
    CMD_ECHO_REQ        = 0x20,     /* 吞吐测试: PC -> MCU */
    CMD_ECHO_RSP        = 0x21,     /* 吞吐测试: MCU -> PC (回传) */
};

/* ==================== 帧结构  ==================== */
#pragma pack(push, 1)
struct frame {
    uint8_t     stxa;               /* 帧头1: 0xAA */
    uint8_t     stxb;               /* 帧头2: 0x55 */
    uint8_t     seq;                /* 包序号 (判断丢包) */
    uint8_t     msg;                /* 消息类型 (CMD枚举) */
    uint16_t    length;             /* payload长度 (字节) */
    uint8_t     payload[0];         /* 柔性数组: 协议内容 */
};
#pragma pack(pop)

/* ==================== DM电机参数信息 ==================== */
#pragma pack(push, 1)
struct dm_joint_param_info {
    float dmap;                     /* 粘滞系数 */
    float inertia;                  /* 转动惯量 */
    float flux;                     /* 磁链值 */
    float gr;                       /* 减速比 */
};

/* DM电机反馈数据 (MCU -> PC, 1kHz) */
struct dm_joint_fb_info {
    uint8_t id;                     /* 关节ID */
    uint8_t state;                  /* 状态: 0=离线 1=空闲 2=运行中 3=故障 */
    float   pos;                    /* 当前位置 */
    float   speed;                  /* 当前速度 */
    float   torque;                 /* 当前转矩 */
};

/* DM电机控制参数 */
struct dm_joint_ctrl_info {
    float target_pos;               /* 目标位置 */
    float target_speed;             /* 目标速度 */
    float target_torque;            /* 目标转矩 */
    float k_pos;                    /* 位置环增益 */
    float k_speed;                  /* 速度环增益 */
};
#pragma pack(pop)

/* ==================== 协议接收缓冲区 ==================== */
struct protocol_buf {
    uint8_t  buf[PROTOCOL_RECV_BUF_SIZE];
    uint16_t wp;                    /* 写指针 */
    uint16_t rp;                    /* 读指针 */
};

/* ==================== CRC16-CCITT ==================== */

/**
 * @brief  CRC16-CCITT 计算
 * @param   seed  初始种子 
 * @param   src   数据源
 * @param   len   数据长度
 * @return  CRC16 校验值
 */
uint16_t crc16_ccitt(uint16_t seed, const uint8_t *src, size_t len);

/* ==================== 帧搜索与解析 ==================== */

/**
 * @brief  在缓冲区中搜索双字节帧头
 * @param  buf   缓冲区
 * @param  len   缓冲区有效长度
 * @param  key1  第一个字节 (STXA)
 * @param  key2  第二个字节 (STXB)
 * @return 帧头偏移量 (0表示buf起始就是帧头); 未找到返回len
 */
uint16_t search_head(const uint8_t *buf, uint16_t len, uint8_t key1, uint8_t key2);

/**
 * @brief  整理接收缓冲区 (将未处理数据移到buf起始位置)
 * @param  buf  协议缓冲区
 */
void neaten_comm_buffer(struct protocol_buf *buf);

/**
 * @brief  从缓冲区搜索并验证一个完整帧
 * @param  buf  数据缓冲区 (帧头必须在buf[0]处)
 * @param  len  缓冲区有效数据长度
 * @return FRAME_OK|frame_len   - 找到合法帧, 低16位=帧总长度
 *         FRAME_ERR|skip_count - CRC错误或帧头不在起始, 低16位=需跳过的字节数
 *         FRAME_LESS           - 数据不足, 等待更多数据
 */
uint32_t search_one_frame(const uint8_t *buf, uint16_t len);

/* ==================== 帧构建 ==================== */

/**
 * @brief  构建完整协议帧到缓冲区
 * @param  msg          命令字
 * @param  payload      载荷数据 (可为NULL如果payload_len==0)
 * @param  payload_len  载荷长度
 * @param  out_buf      输出缓冲区 (≥ payload_len + FRAME_MIN_SIZE)
 * @param  buf_size     输出缓冲区大小
 * @return 帧总长度 (0表示缓冲区不足)
 */
uint16_t protocol_build_frame(uint8_t msg, const uint8_t *payload, uint16_t payload_len,
                              uint8_t *out_buf, uint16_t buf_size);

/* ==================== 便捷构建函数 ==================== */

/**
 * @brief  构建 CMD_CONNECT 帧
 * @param  flag      1=连接, 0=断开
 * @param  out_buf   输出缓冲区
 * @param  buf_size  缓冲区大小
 * @return 帧总长度
 */
uint16_t protocol_build_connect(uint8_t flag, uint8_t *out_buf, uint16_t buf_size);

/**
 * @brief  构建 CMD_HB 心跳帧
 * @param  counter   心跳计数值 (uint16, 累计)
 * @param  out_buf   输出缓冲区
 * @param  buf_size  缓冲区大小
 * @return 帧总长度
 */
uint16_t protocol_build_hb(uint16_t counter, uint8_t *out_buf, uint16_t buf_size);

/**
 * @brief  构建 CMD_DM_PARAM_REQ 帧
 * @param  joint_id  关节ID (从1开始)
 * @param  out_buf   输出缓冲区
 * @param  buf_size  缓冲区大小
 * @return 帧总长度
 */
uint16_t protocol_build_dm_param_req(uint8_t joint_id, uint8_t *out_buf, uint16_t buf_size);

/**
 * @brief  构建 CMD_DM_ENABLE 帧
 * @param  enable    1=使能, 0=失能
 * @param  out_buf   输出缓冲区
 * @param  buf_size  缓冲区大小
 * @return 帧总长度
 */
uint16_t protocol_build_dm_enable(uint8_t enable, uint8_t *out_buf, uint16_t buf_size);

/**
 * @brief  构建 CMD_DM_CTRL (MIT控制) 帧
 * @param  joint_id  关节ID (从1开始)
 * @param  ctrl      控制参数结构体指针
 * @param  out_buf   输出缓冲区
 * @param  buf_size  缓冲区大小
 * @return 帧总长度
 */
uint16_t protocol_build_dm_ctrl(uint8_t joint_id, const struct dm_joint_ctrl_info *ctrl,
                                uint8_t *out_buf, uint16_t buf_size);

/**
 * @brief  构建 CMD_ECHO_REQ 丢包测试帧
 *         payload(240B): echo_seq(2) + padding(238)
 * @param  echo_seq  回环序列号
 * @param  out_buf   输出缓冲区 (>= 248B)
 * @param  buf_size  缓冲区大小
 * @return 帧总长度 (0=失败)
 */
uint16_t protocol_build_echo_req(uint16_t echo_seq, uint8_t *out_buf, uint16_t buf_size);

#ifdef __cplusplus
}
#endif

#endif /* _PROTOCOL_PC_H_ */
