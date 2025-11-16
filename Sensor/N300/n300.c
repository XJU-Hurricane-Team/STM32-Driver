/**
 * @file    n300.c
 * @author  Didi
 * @brief 
 * @version 0.1
 * @date    2025-04-30
 */

#include "n300.h"
#include "crc/crc.h"
#include "my_math/my_math.h"

/**
 * @brief 处理n300数据 
 * 
 * @param n300_handle 句柄
 */
static void n300_get_data(n300_handle_t *n300_handle) {
    static float yaw_data;
    if (n300_handle->frame.instruction_type != N300_MSG_AHRS) {
        n300_handle->recv_len = 0;
        return;
    }
    /* crc8 帧头校验 */
    uint8_t *p_crc8 = &n300_handle->frame.head;
    if (n300_handle->frame.crc8_val == calc_crc8(p_crc8, 4)) {
        n300_handle->recv_len = 0;
        return;
    }
    /* crc16 数据帧校验 */
    uint8_t *p_crc16 = (uint8_t *)&n300_handle->frame.roll_speed;
    if (n300_handle->frame.crc16_val ==
        calc_crc16(p_crc16, n300_handle->frame.data_len)) {
        n300_handle->recv_len = 0;
        return;
    }
    yaw_data = RAD2DEG(n300_handle->frame.yaw);
    if (yaw_data > 180.0f) {
        yaw_data -= 360;
    }
    n300_handle->yaw = yaw_data;
    n300_handle->roll = RAD2DEG(n300_handle->frame.roll);
    n300_handle->pitch = RAD2DEG(n300_handle->frame.pitch);
}

/**
 * @brief 处理接收数据帧
 * 
 * @param n300_handle 句柄 
 * @param data 数据
 * @param len 长度
 */
void n300_prase(n300_handle_t *n300_handle, uint8_t *data, uint32_t len) {
    uint8_t *p = (uint8_t *)&n300_handle->frame;
    for (uint32_t i = 0; i < len; ++i) {
        if (n300_handle->recv_len >= sizeof(n300_handle->frame)) {
            if (n300_handle->frame.tail == 0xFD) {
                /* 数据处理 */
                n300_get_data(n300_handle);
            }
            n300_handle->recv_len = 0;
        }

        p[n300_handle->recv_len] = data[i];
        n300_handle->recv_len++;

        if (n300_handle->frame.head != 0xFC) {
            /* 不是正确的头就数据清除 */
            n300_handle->recv_len = 0;
        }
    }
}
