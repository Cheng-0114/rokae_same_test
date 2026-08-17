/**
 * @file    protocol.c
 * @brief   PC端协议层实现 - CRC16、帧搜索、帧构建
 *          
 */

/* 原样复制自 remote_control-8ee3e55/pc_usb/parser/protocol.c，仅调整 include 路径，
 * 帧解析/CRC/帧构建逻辑不做修改。 */
#include "arm_teleop_usb/protocol.h"
#include <string.h>

/* ==================== CRC16-CCITT ==================== */

/**
 * @brief  CRC16-CCITT 计算
 * @note
 *         多项式: 0x1021 (XMODEM变体)
 *         初始种子: 0xFFFF
 */
uint16_t crc16_ccitt(uint16_t seed, const uint8_t *src, size_t len)
{
    for (; len > 0; len--) {
        uint8_t e, f;

        e = seed ^ *src++;
        f = e ^ (e << 4);
        seed = (seed >> 8) ^ ((uint16_t)f << 8) ^ ((uint16_t)f << 3) ^ ((uint16_t)f >> 4);
    }

    return seed;
}

/* ==================== 帧头搜索 ==================== */

/**
 * @brief  在缓冲区中搜索帧头 0xAA 0x55
 */
uint16_t search_head(const uint8_t *buf, uint16_t len, uint8_t key1, uint8_t key2)
{
    uint16_t i;

    for (i = 0; i < len - 1; i++) {
        if (buf[i] == key1 && buf[i + 1] == key2) {
            return i;
        }
    }

    return i;
}

/* ==================== 接收缓冲区管理 ==================== */

/**
 * @brief  整理接收缓冲区,将未处理数据移到起始位置
 */
void neaten_comm_buffer(struct protocol_buf *buf)
{
    uint16_t i, j;

    /* 缓冲区写满则重置 */
    if (buf->wp >= PROTOCOL_RECV_BUF_SIZE) {
        buf->rp = 0;
        buf->wp = 0;
        return;
    }

    /* 读指针在起始位置,无需整理 */
    if (buf->rp == 0) {
        return;
    }

    /* 异常状态,重置 */
    if (buf->rp >= buf->wp) {
        buf->rp = 0;
        buf->wp = 0;
        return;
    }

    /* 将 [rp, wp) 区间数据移到起始位置 */
    i = 0;
    j = buf->rp;
    while (j < buf->wp) {
        buf->buf[i++] = buf->buf[j++];
    }

    buf->rp = 0;
    buf->wp = i;
}

/* ==================== 帧解析 ==================== */

/**
 * @brief  从缓冲区搜索并校验一个完整帧
 *         帧格式: [STXA][STXB][SEQ][MSG][LEN_L][LEN_H][PAYLOAD...][CRC_L][CRC_H]
 *         CRC校验范围: SEQ ~ PAYLOAD末尾 (共 4 + payload_len 字节)
 *         帧总长度: FRAME_MIN_SIZE + payload_len = 8 + payload_len
 */
uint32_t search_one_frame(const uint8_t *buf, uint16_t len)
{
    uint16_t ret;
    const struct frame *tmp;
    uint16_t frame_len, payload_len;
    uint16_t crc_remote, crc_local;

    /* 数据长度至少要有最小帧长 */
    if (len < FRAME_MIN_SIZE) {
        return FRAME_LESS;
    }

    ret = search_head(buf, len, STXA, STXB);
    if (ret) {
        return FRAME_ERR | ret;
    }

    tmp = (const struct frame *)buf;
    payload_len = tmp->length;

    frame_len = payload_len + FRAME_MIN_SIZE;
    if (len < frame_len) {
        return FRAME_LESS;
    }

    /* 提取远端CRC (小端: 低字节在前) */
    crc_remote = MAKEWORD(buf[frame_len - 2], buf[frame_len - 1]);

    /* 本地计算CRC: 从seq开始(偏移2), 覆盖 4+payload_len 字节 */
    crc_local = crc16_ccitt(0xFFFF, &tmp->seq, frame_len - 4);

    if (crc_remote != crc_local) {
        return FRAME_ERR | 2;   /* CRC错误, 跳过2字节(帧头)重试 */
    }

    return FRAME_OK | frame_len;
}

/* ==================== 帧构建 ==================== */

/* 全局发送序号  */
static uint8_t g_send_seq = 0;

/**
 * @brief  构建完整协议帧
 * @note   帧结构: [STXA][STXB][seq][msg][len_l][len_h][payload...][crc_l][crc_h]
 *         CRC校验范围: seq ~ payload末尾, 种子 0xFFFF
 * @param  msg          命令字
 * @param  payload      载荷数据指针
 * @param  payload_len  载荷长度
 * @param  out_buf      输出缓冲区
 * @param  buf_size     输出缓冲区大小
 * @return 帧总长度 (0表示失败: 缓冲区不足)
 */
uint16_t protocol_build_frame(uint8_t msg, const uint8_t *payload, uint16_t payload_len,
                              uint8_t *out_buf, uint16_t buf_size)
{
    uint16_t frame_len = FRAME_MIN_SIZE + payload_len;
    uint16_t crc;

    /* 检查缓冲区大小 */
    if (buf_size < frame_len) {
        return 0;
    }

    /* 填充帧头 */
    out_buf[0] = STXA;                      /* 帧头1 */
    out_buf[1] = STXB;                      /* 帧头2 */
    out_buf[2] = g_send_seq++;              /* 包序号 (自增) */
    out_buf[3] = msg;                       /* 命令字 */
    out_buf[4] = LOBYTE(payload_len);       /* 长度低字节 */
    out_buf[5] = HIBYTE(payload_len);       /* 长度高字节 */

    /* 填充载荷 */
    if (payload_len > 0 && payload != NULL) {
        memcpy(&out_buf[6], payload, payload_len);
    }

    /* CRC校验: 从seq(偏移2)开始, 覆盖 seq+msg+len+payload = 4+payload_len 字节 */
    crc = crc16_ccitt(0xFFFF, &out_buf[2], 4 + payload_len);

    /* 填充CRC (小端: 低字节在前) */
    out_buf[frame_len - 2] = LOBYTE(crc);
    out_buf[frame_len - 1] = HIBYTE(crc);

    return frame_len;
}

/* ==================== 便捷构建函数 ==================== */

/**
 * @brief  构建 CMD_CONNECT 帧
 *         payload: uint8_t flag (1=连接, 0=断开)
 */
uint16_t protocol_build_connect(uint8_t flag, uint8_t *out_buf, uint16_t buf_size)
{
    return protocol_build_frame(CMD_CONNECT, &flag, sizeof(flag), out_buf, buf_size);
}

/**
 * @brief  构建 CMD_HB 心跳帧
 *         payload: uint16_t counter (累计计数值)
 */
uint16_t protocol_build_hb(uint16_t counter, uint8_t *out_buf, uint16_t buf_size)
{
    uint8_t payload[2];
    payload[0] = LOBYTE(counter);
    payload[1] = HIBYTE(counter);
    return protocol_build_frame(CMD_HB, payload, sizeof(payload), out_buf, buf_size);
}

/**
 * @brief  构建 CMD_DM_PARAM_REQ 帧
 *         payload: uint8_t joint_id
 */
uint16_t protocol_build_dm_param_req(uint8_t joint_id, uint8_t *out_buf, uint16_t buf_size)
{
    return protocol_build_frame(CMD_DM_PARAM_REQ, &joint_id, sizeof(joint_id), out_buf, buf_size);
}

/**
 * @brief  构建 CMD_DM_ENABLE 帧
 *         payload: uint8_t enable (1=使能, 0=失能)
 */
uint16_t protocol_build_dm_enable(uint8_t enable, uint8_t *out_buf, uint16_t buf_size)
{
    return protocol_build_frame(CMD_DM_ENABLE, &enable, sizeof(enable), out_buf, buf_size);
}

/**
 * @brief  构建 CMD_DM_CTRL 帧
 *         payload: uint8_t joint_id + dm_joint_ctrl_info (21字节)
 */
uint16_t protocol_build_dm_ctrl(uint8_t joint_id, const struct dm_joint_ctrl_info *ctrl,
                                uint8_t *out_buf, uint16_t buf_size)
{
    /* 手动拼接: joint_id(1) + ctrl_info(20) = 21字节 */
    uint8_t payload[21];
    payload[0] = joint_id;
    memcpy(&payload[1], ctrl, sizeof(struct dm_joint_ctrl_info));
    return protocol_build_frame(CMD_DM_CTRL, payload, sizeof(payload), out_buf, buf_size);
}

/* ==================== 吞吐测试帧 ==================== */

/**
 * @brief  构建 CMD_ECHO_REQ 帧
 *         payload(240B): echo_seq(2) + padding(238)
 *         帧总长: 8 + 240 = 248 字节 (MCU kfifo_out 上限256内)
 */
uint16_t protocol_build_echo_req(uint16_t echo_seq,
                                 uint8_t *out_buf, uint16_t buf_size)
{
    uint8_t payload[240];
    memset(payload, 0, sizeof(payload));
    payload[0] = LOBYTE(echo_seq);
    payload[1] = HIBYTE(echo_seq);
    return protocol_build_frame(CMD_ECHO_REQ, payload, sizeof(payload), out_buf, buf_size);
}
