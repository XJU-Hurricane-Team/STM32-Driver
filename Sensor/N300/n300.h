/**
 * @file n300.h
 * @author CV-Engineer-Chen
 * @brief WHEELLTEC N300 IMU数据解析
 * @version 0.1
 * @date 2025-04-30
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef __N300_H
#define __N300_H

#include <stdint.h>

typedef enum __attribute((packed)) {
    N300_MSG_IMU = 0x40,
    N300_MSG_AHRS = 0x41,
    N300_MSG_INS_GPS = 0x42,
    N300_MSG_RAW_SENSORS = 0x58
} n300_instruction_type;

struct __attribute((packed)) n300_frame {
    uint8_t head;
    n300_instruction_type instruction_type;
    uint8_t data_len;
    uint8_t send_count;
    uint8_t crc8_val;
    uint16_t crc16_val;

    struct __attribute((packed)) {
        float roll_speed;  /* unit: rad/s */
        float pitch_speed; /* unit: rad/s */
        float yaw_speed;   /* unit: rad/s */
        float roll;        /* unit: rad */
        float pitch;       /* unit: rad */
        float yaw;         /* unit: rad */
        float qw;          /* w */
        float qx;          /* x */
        float qy;          /* y */
        float qz;          /* z */
        int64_t timestamp; /* unit: us */
    };

    uint8_t tail; /* 尾 */
};

typedef struct {
    struct n300_frame frame; /* 帧格式 */
    uint32_t recv_len;

    float yaw;   /* -180~180 */
    float pitch; /* -180~180 */
    float roll;  /* -180~180 */
} n300_handle_t;

void n300_prase(n300_handle_t *n300_handle, uint8_t *data, uint32_t len);

#endif /* __N300_H */
