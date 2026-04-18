/**
 ****************************************************************************************************
 * @file        smd.c
 * @author      正点原子团队(ALIENTEK),PickingChip
 * @version     V1.1
 * @date        2026-04-18
 * @brief       步进电机驱动器 控制指令代码
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 * @note        v1.1 增加了电机结构体方便存储电机状态，将电机信息打印到消息缓存区
 *
 ****************************************************************************************************
 */

#include "./smd.h"
#if COMM_TYPE == 0
#include "./smd_usart.h"
#elif COMM_TYPE == 1
#include "./smd_can.h"
#endif

#include "stdio.h"
#include "string.h"
#include "stdlib.h"

/* 电机列表 */
static smd_motor_t *smd_motor_list[MOTOR_NUM_MAX] = {0};

/* 添加联合体 便于存储浮点数 */
union
{
    float f;
    uint8_t b[4];
} data_u;

/**
 * @brief       发送SMD协议数据发送实现
 * @param       data    指向要发送的数据缓冲区
 * @param       len     要发送的数据长度
 * @retval      无
 */
void smd_send_data(uint8_t *data, uint8_t len)
{
/* 当前仅实现了485通信 */
#if COMM_TYPE == 0
    smd_usart_send_cmd( data, len); 
#elif COMM_TYPE == 1
    can_send_long_msg(CAN_EXTID, data, len);
#endif
}



/*************************************************************************************************
 * @defgroup process_frame
 * @{
 */

/* 将读取到的数据转成浮点数 */
typedef union
{
    float f;
    uint8_t b[4];
} float_union_t;


/**
 * @brief       将4个字节的数据转换为 float 类型
 * @param       buf     指向包含4字节数据的缓冲区（小端格式：低字节在前）
 * @return      转换后的 float 浮点数值
 */
float bytes_to_float(uint8_t *buf)
{
    float_union_t u;

    /* 将字节逐一写入联合体中的字节数组（假设小端顺序） */
    u.b[0] = buf[3];   /* 低字节 */
    u.b[1] = buf[2];
    u.b[2] = buf[1];
    u.b[3] = buf[0];   /* 高字节 */

    /* 返回组合后的浮点数 */
    return u.f;
}

/**
 * @brief 初始化电机
 * @param motor 电机结构体指针
 * @param motor_id 电机ID（地址）
 *
 */
uint8_t smd_motor_init(smd_motor_t *motor, uint8_t motor_id) {
    if (motor == NULL || motor_id >= MOTOR_NUM_MAX) {
        return 1;
    }

    motor->slave_addr = motor_id;
    motor->info = malloc(SMD_INFO_BUF_SIZE); /* 分配信息缓冲区 */
    if (motor->info == NULL) {
        return 2; /* 内存分配失败 */
    }
    memset(motor->info, 0, SMD_INFO_BUF_SIZE);
    motor->valid_mask = 0;
    smd_motor_list[motor_id] = motor;
    return 0;
}


/**
 * @brief   串口数据帧处理函数
 * @param   buffer: 输入缓冲区
 * @param   len: 缓冲区长度
 * @param   frame: 输出解析结果
 * @retval  无
 */
bool serial_frame_process(uint8_t *buffer, uint8_t len, SERIAL_FRAME *frame) {
    size_t off = 0;

    if (buffer == NULL || frame == NULL || len < 6) {
        return false;
    }

    if (buffer[0] != FRAME_HEAD || buffer[len - 1] != FRAME_TAIL) {
        return false;
    }

    /* 初始化输出结构 */
    memset(frame, 0, sizeof(SERIAL_FRAME));

    /* 填充解析结果 */
    frame->slave_addr = buffer[1];
    frame->function_code = buffer[2];
    frame->error_code = buffer[3];
    frame->checksum = buffer[len - 2];

    /* 计算数据部分长度（排除固定部分） */
    frame->data_len =
        len -
        6; /* 总长 - (固定部分：头(1) + 地址(1) + 功能码(1) + 错误码（1） + 校验和(1) + 尾(1) = 6字节) */

    if (frame->data_len > 0 && frame->data_len <= sizeof(frame->data)) {
        memcpy(frame->data, &buffer[4], frame->data_len);
    } else {
        frame->data_len = 0;
    }

    /* 查找对应的电机 */
    if (frame->slave_addr >= MOTOR_NUM_MAX) {
        return false;
    }

    smd_motor_t *motor = smd_motor_list[frame->slave_addr];
    if (motor == NULL || motor->info == NULL) {
        return false;
    }
    memset(motor->info, 0, SMD_INFO_BUF_SIZE); /* 清空电机信息缓冲区 */
    motor->valid_mask = 0;        /* 重置有效字段掩码 */

    if (frame->error_code != ACK_SUCCEED) {
        /* 根据错误码处理 */
        switch (frame->error_code) {
            case ACK_FRAME_TOO_SHORT:
                snprintf((char *)motor->info, SMD_INFO_BUF_SIZE, "帧长度不足");
                break;
            case ACK_INVALID_HEADER:
                snprintf((char *)motor->info, SMD_INFO_BUF_SIZE, "帧头有误");
                break;
            case ACK_INVALID_FOOTER:
                snprintf((char *)motor->info, SMD_INFO_BUF_SIZE, "帧尾有误");
                break;
            case ACK_CHECKSUM_MISMATCH:
                snprintf((char *)motor->info, SMD_INFO_BUF_SIZE, "校验和错误");
                break;
            case ACK_UNSUPPORTED_FUNCTION:
                snprintf((char *)motor->info, SMD_INFO_BUF_SIZE, "不支持的功能码");
                break;
            case ACK_ERR_ILLEGAL_VAL:
                snprintf((char *)motor->info, SMD_INFO_BUF_SIZE, "数据不合法");
                break;
            default:
                snprintf((char *)motor->info, SMD_INFO_BUF_SIZE,
                         "未知错误码:0x%02X", frame->error_code);
                break;
        }
        motor->last_error = frame->error_code; /* 更新电机的last_error字段 */
        motor->valid_mask |= (SMD_MASK_INFO | SMD_MASK_LAST_ERROR);
        return false;
    }

    /* 根据功能码处理 */
    switch (frame->function_code) {
        case FCT_CAL_ENCODER:
            if (frame->data[0] == 1) {
                snprintf((char *)motor->info, SMD_INFO_BUF_SIZE, "校准中！");
            } else if (frame->data[0] == 2) {
                snprintf((char *)motor->info, SMD_INFO_BUF_SIZE, "校准失败");
            } else if (frame->data[0] == 3) {
                snprintf((char *)motor->info, SMD_INFO_BUF_SIZE, "校准成功");
            }
            motor->valid_mask |= SMD_MASK_INFO;
            break;

        case FCT_RESTART:
            snprintf((char *)motor->info, SMD_INFO_BUF_SIZE, "复位成功");
            motor->valid_mask |= SMD_MASK_INFO;
            break;

        case FCT_RESET_FACTORY:
            snprintf((char *)motor->info, SMD_INFO_BUF_SIZE,
                     "恢复出厂设置成功，请等待重新识别参数，电机停止则识别完成");
            motor->valid_mask |= SMD_MASK_INFO;
            break;

        case FCT_PARAM_SAVE:
            snprintf((char *)motor->info, SMD_INFO_BUF_SIZE, "参数保存成功");
            motor->valid_mask |= SMD_MASK_INFO;
            break;

        case FCT_READ_SOFT_HARD_VER:
                snprintf((char *)motor->info, SMD_INFO_BUF_SIZE,
                    "软件版本：V%d.%d, 硬件版本：V%d.%d",
                    frame->data[0] / 10, frame->data[0] % 10,
                    frame->data[1] / 10, frame->data[1] % 10);
                motor->valid_mask |= SMD_MASK_INFO;
            break;

        case FCT_READ_PSI: {
            float psi = bytes_to_float((uint8_t *)&frame->data[0]);
            snprintf((char *)motor->info, SMD_INFO_BUF_SIZE, "磁链：%.2fmWb", psi);
            motor->valid_mask |= SMD_MASK_INFO;
            break;
        }
        case FCT_READ_PHASE_RES_IND: {
            float rs = bytes_to_float((uint8_t *)&frame->data[0]);
            float ls = bytes_to_float((uint8_t *)&frame->data[4]);
            snprintf((char *)motor->info, SMD_INFO_BUF_SIZE,
                     "相电阻：%.2fΩ, 相电感：%.2fmH", rs, ls);
            motor->valid_mask |= SMD_MASK_INFO;
            break;
        }

        case FCT_READ_PHASE_MA: {
            int16_t iq =
                (int16_t)((int16_t)frame->data[0] << 8 | frame->data[1]);
            snprintf((char *)motor->info, SMD_INFO_BUF_SIZE, "相电流：%dmA", iq);
            motor->valid_mask |= SMD_MASK_INFO;
            break;
        }

        case FCT_READ_VOL: {
            float power = bytes_to_float((uint8_t *)&frame->data[0]);
            snprintf((char *)motor->info, SMD_INFO_BUF_SIZE, "总线电压：%.1fV", power);
            motor->bus_volt = power;
            motor->valid_mask |= (SMD_MASK_INFO | SMD_MASK_BUS_VOLT);
            break;
        }

        case FCT_READ_MA_PID: {
            float q_kp = bytes_to_float((uint8_t *)&frame->data[0]);
            float q_ki = bytes_to_float((uint8_t *)&frame->data[4]);
            float d_kp = bytes_to_float((uint8_t *)&frame->data[8]);
            float d_ki = bytes_to_float((uint8_t *)&frame->data[12]);
                snprintf((char *)motor->info, SMD_INFO_BUF_SIZE,
                    "电流环DQ轴PI参数：q_kp:%.5f, q_ki:%.5f, d_kp:%.5f, "
                    "d_ki:%.5f",
                    q_kp, q_ki, d_kp, d_ki);
                motor->valid_mask |= SMD_MASK_INFO;
            break;
        }

        case FCT_READ_SPEED_PID: {
            uint32_t kp = (uint32_t)((uint32_t)frame->data[0] << 24 |
                                     (uint32_t)frame->data[1] << 16 |
                                     (uint32_t)frame->data[2] << 8 |
                                     (uint32_t)frame->data[3] << 0);
            uint32_t ki = (uint32_t)((uint32_t)frame->data[4] << 24 |
                                     (uint32_t)frame->data[5] << 16 |
                                     (uint32_t)frame->data[6] << 8 |
                                     (uint32_t)frame->data[7] << 0);
            uint32_t kd = (uint32_t)((uint32_t)frame->data[8] << 24 |
                                     (uint32_t)frame->data[9] << 16 |
                                     (uint32_t)frame->data[10] << 8 |
                                     (uint32_t)frame->data[11] << 0);
                snprintf((char *)motor->info, SMD_INFO_BUF_SIZE,
                    "速度环PID参数：kp:%lu, ki:%lu, kd:%lu", (unsigned long)kp,
                    (unsigned long)ki, (unsigned long)kd);
                motor->valid_mask |= SMD_MASK_INFO;
            break;
        }

        case FCT_READ_POS_PID: {
            uint32_t kp = (uint32_t)((uint32_t)frame->data[0] << 24 |
                                     (uint32_t)frame->data[1] << 16 |
                                     (uint32_t)frame->data[2] << 8 |
                                     (uint32_t)frame->data[3] << 0);
            uint32_t ki = (uint32_t)((uint32_t)frame->data[4] << 24 |
                                     (uint32_t)frame->data[5] << 16 |
                                     (uint32_t)frame->data[6] << 8 |
                                     (uint32_t)frame->data[7] << 0);
            uint32_t kd = (uint32_t)((uint32_t)frame->data[8] << 24 |
                                     (uint32_t)frame->data[9] << 16 |
                                     (uint32_t)frame->data[10] << 8 |
                                     (uint32_t)frame->data[11] << 0);
                snprintf((char *)motor->info, SMD_INFO_BUF_SIZE,
                    "位置环PID参数：kp:%lu, ki:%lu, kd:%lu", (unsigned long)kp,
                    (unsigned long)ki, (unsigned long)kd);
                motor->valid_mask |= SMD_MASK_INFO;
            break;
        }
        case FCT_READ_TOTAL_PULSE: {
            int32_t pulse_cnt = (int32_t)((int32_t)frame->data[0] << 24 |
                                          (int32_t)frame->data[1] << 16 |
                                          (int32_t)frame->data[2] << 8 |
                                          (int32_t)frame->data[3] << 0);
            snprintf((char *)motor->info, SMD_INFO_BUF_SIZE, "累计脉冲数：%ld",
                     (long)pulse_cnt);
            motor->pulse_cnt = pulse_cnt; /* 更新电机的pulse_cnt字段 */
            motor->valid_mask |= (SMD_MASK_INFO | SMD_MASK_PULSE_CNT);
            break;
        }

        case FCT_READ_ROTATE_SPEED: {
            int16_t rpm = (int16_t)((int16_t)frame->data[0] << 8 |
                                    (int16_t)frame->data[1] << 0);
            snprintf((char *)motor->info, SMD_INFO_BUF_SIZE, "实时转速：%dRPM", rpm);
            motor->speed_rpm = rpm; /* 更新电机的speed_rpm字段 */
            motor->valid_mask |= (SMD_MASK_INFO | SMD_MASK_SPEED_RPM);
            break;
        }

        case FCT_READ_POS: {
            int32_t pos = (int32_t)((int32_t)frame->data[0] << 24 |
                                    (int32_t)frame->data[1] << 16 |
                                    (int32_t)frame->data[2] << 8 |
                                    (int32_t)frame->data[3] << 0);
            snprintf((char *)motor->info, SMD_INFO_BUF_SIZE,
                     "实时位置（51200为一圈）：%ld", (long)pos);
            motor->real_pos = pos; /* 更新电机的real_pos字段 */
            motor->valid_mask |= (SMD_MASK_INFO | SMD_MASK_REAL_POS);
            break;
        }

        case FCT_READ_POS_ERROR: {
            int32_t pos_err = (int32_t)((int32_t)frame->data[0] << 24 |
                                        (int32_t)frame->data[1] << 16 |
                                        (int32_t)frame->data[2] << 8 |
                                        (int32_t)frame->data[3] << 0);
            snprintf((char *)motor->info, SMD_INFO_BUF_SIZE,
                     "位置误差(51200=1圈): %ld", (long)pos_err);
            motor->pos_err = pos_err; /* 更新电机的pos_err字段 */
            motor->valid_mask |= (SMD_MASK_INFO | SMD_MASK_POS_ERR);
            break;
        }

        case FCT_READ_MOTOR_STA: {
            if (frame->data[0] == 0) {
                snprintf((char *)motor->info, SMD_INFO_BUF_SIZE, "电机状态：空闲态");
            } else if (frame->data[0] == 1) {
                snprintf((char *)motor->info, SMD_INFO_BUF_SIZE, "电机状态：已完成");
            } else if (frame->data[0] == 2) {
                snprintf((char *)motor->info, SMD_INFO_BUF_SIZE, "电机状态：正在运行");
            } else if (frame->data[0] == 3) {
                snprintf((char *)motor->info, SMD_INFO_BUF_SIZE, "电机状态：过载");
            } else if (frame->data[0] == 4) {
                snprintf((char *)motor->info, SMD_INFO_BUF_SIZE, "电机状态：堵转");
            } else if (frame->data[0] == 5) {
                snprintf((char *)motor->info, SMD_INFO_BUF_SIZE, "电机状态：欠压");
            }
            motor->motor_sta = frame->data[0]; /* 更新电机的motor_sta字段 */
            motor->valid_mask |= (SMD_MASK_INFO | SMD_MASK_MOTOR_STA);
            break;
        }

        case FCT_READ_CLOG_FLAG: {
            if (frame->data[0] == 0) {
                snprintf((char *)motor->info, SMD_INFO_BUF_SIZE, "未堵转");
            } else if (frame->data[0] == 1) {
                snprintf((char *)motor->info, SMD_INFO_BUF_SIZE, "堵转");
            }
            motor->clog_flag = frame->data[0]; /* 更新电机的clog_flag字段 */
            motor->valid_mask |= (SMD_MASK_INFO | SMD_MASK_CLOG_FLAG);
            break;
        }

        case FCT_READ_CLOG_CUR: {
            int16_t stall_ma = (int16_t)((int16_t)frame->data[0] << 8 |
                                         (int16_t)frame->data[1] << 0);
            snprintf((char *)motor->info, SMD_INFO_BUF_SIZE, "堵转电流：%dmA",
                     stall_ma);
            motor->valid_mask |= SMD_MASK_INFO;
            break;
        }

        case FCT_READ_ENABLE_STA: {
            if (frame->data[0] == 0) {
                snprintf((char *)motor->info, SMD_INFO_BUF_SIZE, "使能");
            } else if (frame->data[0] == 1) {
                snprintf((char *)motor->info, SMD_INFO_BUF_SIZE, "失能");
            }
            motor->enable_sta = frame->data[0]; /* 更新电机的enable_sta字段 */
            motor->valid_mask |= (SMD_MASK_INFO | SMD_MASK_ENABLE_STA);
            break;
        }

        case FCT_READ_ARRIVED_STA: {
            if (frame->data[0] == 0) {
                snprintf((char *)motor->info, SMD_INFO_BUF_SIZE, "未到位");
            } else if (frame->data[0] == 1) {
                snprintf((char *)motor->info, SMD_INFO_BUF_SIZE, "到位");
            }
            motor->arrived_sta = frame->data[0]; /* 更新电机的arrived_sta字段 */
            motor->valid_mask |= (SMD_MASK_INFO | SMD_MASK_ARRIVED_STA);
            break;
        }

        case FCT_READ_SYS_PARAM: {
            float power = bytes_to_float((uint8_t *)&frame->data[0]);
            int16_t iq = (int16_t)((int16_t)frame->data[4] << 8 |
                                   (int16_t)frame->data[5] << 0);
            float psi = bytes_to_float((uint8_t *)&frame->data[6]);
            float rs = bytes_to_float((uint8_t *)&frame->data[10]);
            float ls = bytes_to_float((uint8_t *)&frame->data[14]);
            int16_t rpm = (int16_t)((int16_t)frame->data[18] << 8 |
                                    (int16_t)frame->data[19] << 0);
            int32_t pos_tar = (int32_t)((int32_t)frame->data[20] << 24 |
                                        (int32_t)frame->data[21] << 16 |
                                        (int32_t)frame->data[22] << 8 |
                                        (int32_t)frame->data[23] << 0);
            int32_t pos = (int32_t)((int32_t)frame->data[24] << 24 |
                                    (int32_t)frame->data[25] << 16 |
                                    (int32_t)frame->data[26] << 8 |
                                    (int32_t)frame->data[27] << 0);
            int32_t pos_err = (int32_t)((int32_t)frame->data[28] << 24 |
                                        (int32_t)frame->data[29] << 16 |
                                        (int32_t)frame->data[30] << 8 |
                                        (int32_t)frame->data[31] << 0);
            int32_t pulse_cnt = (int32_t)((int32_t)frame->data[32] << 24 |
                                          (int32_t)frame->data[33] << 16 |
                                          (int32_t)frame->data[34] << 8 |
                                          (int32_t)frame->data[35] << 0);

            motor->bus_volt = power;
            motor->speed_rpm = rpm;
            motor->target_pos = pos_tar;
            motor->real_pos = pos;
            motor->pos_err = pos_err;
            motor->pulse_cnt = pulse_cnt;
            motor->enable_sta = frame->data[36] ? 1 : 0;
            motor->arrived_sta = frame->data[37] ? 1 : 0;
            motor->clog_flag = frame->data[38] ? 1 : 0;

            off += (size_t)snprintf((char *)motor->info + off,
                                    SMD_INFO_BUF_SIZE - off,
                                    "总线电压:%.1fV, 相电流:%dmA, 磁链:%.2fmWb, 相电阻:%.2fΩ, 相电感:%.2fmH\n",
                                    power, iq, psi, rs, ls);
            if (off < SMD_INFO_BUF_SIZE) {
                off += (size_t)snprintf((char *)motor->info + off,
                                        SMD_INFO_BUF_SIZE - off,
                                        "实时转速:%dRPM, 目标位置:%ld, 实时位置:%ld, 位置误差:%ld, 累计脉冲:%ld\n",
                                        rpm, (long)pos_tar, (long)pos,
                                        (long)pos_err, (long)pulse_cnt);
            }
            if (off < SMD_INFO_BUF_SIZE) {
                (void)snprintf((char *)motor->info + off,
                               SMD_INFO_BUF_SIZE - off,
                               "电机%s, %s, %s, %s",
                               frame->data[36] ? "失能" : "使能",
                               frame->data[37] ? "到位" : "未到位",
                               frame->data[38] ? "堵转" : "未堵转",
                               frame->data[39] ? "分组模式" : "从机模式");
            }

            motor->valid_mask |= (SMD_MASK_INFO | SMD_MASK_BUS_VOLT |
                                  SMD_MASK_SPEED_RPM | SMD_MASK_TARGET_POS |
                                  SMD_MASK_REAL_POS | SMD_MASK_POS_ERR |
                                  SMD_MASK_PULSE_CNT | SMD_MASK_ENABLE_STA |
                                  SMD_MASK_ARRIVED_STA | SMD_MASK_CLOG_FLAG);

            break;
        }
        case FCT_READ_DRIVE_PARAMS: {
            const char *mode_str = "未知模式";
            uint32_t uart_baud = (uint32_t)((uint32_t)frame->data[2] << 24 |
                                            (uint32_t)frame->data[3] << 16 |
                                            (uint32_t)frame->data[4] << 8 |
                                            (uint32_t)frame->data[5] << 0);
            uint16_t can_baud = (uint16_t)((uint16_t)frame->data[6] << 8 |
                                           (uint16_t)frame->data[7] << 0);
            uint16_t step = (uint16_t)((uint16_t)frame->data[10] << 8 |
                                       (uint16_t)frame->data[11] << 0);
            int16_t pos_ma = (int16_t)((int32_t)frame->data[12] << 8 |
                                       (int32_t)frame->data[13] << 0);
            uint16_t stall_ma = (int16_t)((int16_t)frame->data[26] << 8 |
                                          (int16_t)frame->data[27] << 0);

            switch (frame->data[0]) {
                case 0:
                    mode_str = "通信位置模式";
                    break;
                case 1:
                    mode_str = "通信速度模式";
                    break;
                case 2:
                    mode_str = "通信力矩模式";
                    break;
                case 3:
                    mode_str = "脉冲模式";
                    break;
                case 4:
                    mode_str = "脉宽位置模式";
                    break;
                case 5:
                    mode_str = "脉宽速度模式";
                    break;
                case 6:
                    mode_str = "脉宽力矩模式";
                    break;
                case 7:
                    mode_str = "回零模式";
                    break;
                case 8:
                    mode_str = "开环速度模式";
                    break;
                case 9:
                    mode_str = "开环位置模式";
                    break;
                case 10:
                    mode_str = "开环脉冲模式";
                    break;
                default:
                    break;
            }

            uint32_t s_kp = (uint32_t)((uint32_t)frame->data[30] << 24 |
                                       (uint32_t)frame->data[31] << 16 |
                                       (uint32_t)frame->data[32] << 8 |
                                       (uint32_t)frame->data[33] << 0);
            uint32_t s_ki = (uint32_t)((uint32_t)frame->data[34] << 24 |
                                       (uint32_t)frame->data[35] << 16 |
                                       (uint32_t)frame->data[36] << 8 |
                                       (uint32_t)frame->data[37] << 0);
            uint32_t s_kd = (uint32_t)((uint32_t)frame->data[38] << 24 |
                                       (uint32_t)frame->data[39] << 16 |
                                       (uint32_t)frame->data[40] << 8 |
                                       (uint32_t)frame->data[41] << 0);
            snprintf((char *)motor->info, SMD_INFO_BUF_SIZE,
                     "模式:%s, 回响:%s, UART:%lu, CAN:%uK, DIR:%s, EN:%u, 细分:%u, 位置环力矩:%d, 堵转电流:%u, 速度环PID:%lu/%lu/%lu, 自动熄屏:%s, IO启停:%s",
                     mode_str, frame->data[1] ? "不回响" : "回响",
                     (unsigned long)uart_baud, (unsigned int)can_baud,
                     frame->data[8] ? "低电平正转" : "高电平正转",
                     (unsigned int)frame->data[9], (unsigned int)step, pos_ma,
                     (unsigned int)stall_ma, (unsigned long)s_kp,
                     (unsigned long)s_ki, (unsigned long)s_kd,
                     frame->data[42] ? "开启" : "关闭",
                     frame->data[43] ? "高电平启动" : "低电平启动");
            motor->valid_mask |= SMD_MASK_INFO;
            break;
        }

        case FCT_SET_SLAVE_ADD:
            snprintf((char *)motor->info, SMD_INFO_BUF_SIZE,
                     "成功设置从机地址：0x%02X", frame->data[0]);
            motor->valid_mask |= SMD_MASK_INFO;
            break;

        case FCT_SET_GROUP_ADD:
            snprintf((char *)motor->info, SMD_INFO_BUF_SIZE,
                     "成功设置分组地址：0x%02X", frame->data[0]);
            motor->valid_mask |= SMD_MASK_INFO;
            break;

        case FCT_SET_MODE:
            if (frame->data[0] == 0) {
                snprintf((char *)motor->info, SMD_INFO_BUF_SIZE,
                         "成功设置电机工作模式为：通信位置模式");
            } else if (frame->data[0] == 1) {
                snprintf((char *)motor->info, SMD_INFO_BUF_SIZE,
                         "成功设置电机工作模式为：通信速度模式");
            } else if (frame->data[0] == 2) {
                snprintf((char *)motor->info, SMD_INFO_BUF_SIZE,
                         "成功设置电机工作模式为：通信力矩模式");
            } else if (frame->data[0] == 3) {
                snprintf((char *)motor->info, SMD_INFO_BUF_SIZE,
                         "成功设置电机工作模式为：脉冲模式");
            } else if (frame->data[0] == 4) {
                snprintf((char *)motor->info, SMD_INFO_BUF_SIZE,
                         "成功设置电机工作模式为：脉宽位置模式");
            } else if (frame->data[0] == 5) {
                snprintf((char *)motor->info, SMD_INFO_BUF_SIZE,
                         "成功设置电机工作模式为：脉宽速度模式");
            } else if (frame->data[0] == 6) {
                snprintf((char *)motor->info, SMD_INFO_BUF_SIZE,
                         "成功设置电机工作模式为：脉宽力矩模式");
            } else if (frame->data[0] == 7) {
                snprintf((char *)motor->info, SMD_INFO_BUF_SIZE,
                         "成功设置电机工作模式为：回零模式");
            } else if (frame->data[0] == 8) {
                snprintf((char *)motor->info, SMD_INFO_BUF_SIZE,
                         "成功设置电机工作模式为：开环速度模式");
            } else if (frame->data[0] == 9) {
                snprintf((char *)motor->info, SMD_INFO_BUF_SIZE,
                         "成功设置电机工作模式为：开环位置模式");
            } else if (frame->data[0] == 10) {
                snprintf((char *)motor->info, SMD_INFO_BUF_SIZE,
                         "成功设置电机工作模式为：开环脉冲模式");
            }
            motor->valid_mask |= SMD_MASK_INFO;
            break;

        case FCT_SET_POS_PID:
            snprintf((char *)motor->info, SMD_INFO_BUF_SIZE,
                   "成功设置位置环PID参数：P:%lu,I:%lu,D:%lu",
                   (uint32_t)(frame->data[0] << 24 | frame->data[1] << 16 |
                              frame->data[2] << 8 | frame->data[3] << 0),
                   (uint32_t)(frame->data[4] << 24 | frame->data[5] << 16 |
                              frame->data[6] << 8 | frame->data[7] << 0),
                   (uint32_t)(frame->data[8] << 24 | frame->data[9] << 16 |
                              frame->data[10] << 8 | frame->data[11] << 0));
            motor->valid_mask |= SMD_MASK_INFO;
            break;

        case FCT_SET_POS_TORQUE:
            snprintf((char *)motor->info, SMD_INFO_BUF_SIZE,
                   "成功设置位置环力矩：%dmA",
                   (int16_t)(frame->data[0] << 8 | frame->data[1] << 0));
            motor->valid_mask |= SMD_MASK_INFO;
            break;

        case FCT_SET_STEP:
            snprintf((char *)motor->info, SMD_INFO_BUF_SIZE,
                   "成功设置细分为：%u",
                   (uint16_t)(frame->data[0] << 8 | frame->data[1] << 0));
            motor->valid_mask |= SMD_MASK_INFO;
            break;

        case FCT_SET_MA:
            snprintf((char *)motor->info, SMD_INFO_BUF_SIZE,
                   "成功设置目标电流为：%dmA",
                   (int16_t)(frame->data[0] << 8 | frame->data[1] << 0));
            motor->valid_mask |= SMD_MASK_INFO;
            break;

        case FCT_SET_UART_BAUD:
            snprintf((char *)motor->info, SMD_INFO_BUF_SIZE,
                   "成功设置串口波特率为：%lu",
                   (uint32_t)(frame->data[0] << 24 | frame->data[1] << 16 |
                              frame->data[2] << 8 | frame->data[3] << 0));
            motor->valid_mask |= SMD_MASK_INFO;
            break;

        case FCT_SET_CAN_BAUD:
            snprintf((char *)motor->info, SMD_INFO_BUF_SIZE,
                   "成功设置CAN波特率为：%uK",
                   (uint16_t)(frame->data[0] << 8 | frame->data[1] << 0));
            motor->valid_mask |= SMD_MASK_INFO;
            break;

        case FCT_SET_MODBUS:
            if (frame->data[0] == 0) {
                snprintf((char *)motor->info, SMD_INFO_BUF_SIZE,
                         "使用自定义协议");
            } else if (frame->data[0] == 1) {
                snprintf((char *)motor->info, SMD_INFO_BUF_SIZE,
                         "使用modbus协议");
            }
            motor->valid_mask |= SMD_MASK_INFO;
            break;

        case FCT_SET_CLOG_PRO:
            if (frame->data[0] == 0) {
                snprintf((char *)motor->info, SMD_INFO_BUF_SIZE,
                         "成功关闭堵转保护");
            } else if (frame->data[0] == 1) {
                snprintf((char *)motor->info, SMD_INFO_BUF_SIZE,
                         "成功开启堵转保护");
            }
            motor->valid_mask |= SMD_MASK_INFO;
            break;

        case FCT_SET_CLOG_CUR:
            snprintf((char *)motor->info, SMD_INFO_BUF_SIZE,
                   "成功设置堵转电流为：%dmA",
                   (int16_t)(frame->data[0] << 8 | frame->data[1] << 0));
            motor->valid_mask |= SMD_MASK_INFO;
            break;

        case FCT_SET_CAN_ID:
            snprintf((char *)motor->info, SMD_INFO_BUF_SIZE,
                   "设置CAN发送ID为：0x%08lX",
                   (uint32_t)(frame->data[0] << 24 | frame->data[1] << 16 |
                              frame->data[2] << 8 | frame->data[3] << 0));
            motor->valid_mask |= SMD_MASK_INFO;
            break;

        case FCT_SET_DIR_LEVEL:
            if (frame->data[0] == 0) {
                snprintf((char *)motor->info, SMD_INFO_BUF_SIZE,
                         "成功设置旋转方向为：高电平正转");
            } else if (frame->data[0] == 1) {
                snprintf((char *)motor->info, SMD_INFO_BUF_SIZE,
                         "成功设置旋转方向为：高电平反转");
            }
            motor->valid_mask |= SMD_MASK_INFO;
            break;

        case FCT_SET_EN_LEVEL:
            if (frame->data[0] == 0) {
                snprintf((char *)motor->info, SMD_INFO_BUF_SIZE,
                         "成功设置EN脚低电平有效");
            } else if (frame->data[0] == 1) {
                snprintf((char *)motor->info, SMD_INFO_BUF_SIZE,
                         "成功设置EN脚高电平有效");
            } else if (frame->data[0] == 2) {
                snprintf((char *)motor->info, SMD_INFO_BUF_SIZE,
                         "成功设置EN脚保持有效");
            }
            motor->valid_mask |= SMD_MASK_INFO;
            break;

        case FCT_SET_KEY_LOCK:
            if (frame->data[0] == 0) {
                snprintf((char *)motor->info, SMD_INFO_BUF_SIZE,
                         "成功解锁按键");
            } else if (frame->data[0] == 1) {
                snprintf((char *)motor->info, SMD_INFO_BUF_SIZE,
                         "成功上锁按键");
            }
            motor->valid_mask |= SMD_MASK_INFO;
            break;

        case FCT_SET_AUTO_NOT_DISPLAY:
            if (frame->data[0] == 0) {
                snprintf((char *)motor->info, SMD_INFO_BUF_SIZE,
                         "成功关闭自动熄屏");
            } else if (frame->data[0] == 1) {
                snprintf((char *)motor->info, SMD_INFO_BUF_SIZE,
                         "成功开启自动熄屏");
            }
            motor->valid_mask |= SMD_MASK_INFO;
            break;

        case FCT_SET_IO_START_LEVEL:
            if (frame->data[0] == 0) {
                snprintf((char *)motor->info, SMD_INFO_BUF_SIZE,
                         "设置IO低电平启动");
            } else if (frame->data[0] == 1) {
                snprintf((char *)motor->info, SMD_INFO_BUF_SIZE,
                         "设置IO高电平启动");
            }
            motor->valid_mask |= SMD_MASK_INFO;
            break;

        case FCT_SET_SPEED_PID:
            snprintf((char *)motor->info, SMD_INFO_BUF_SIZE,
                   "成功设置速度环PID参数：P:%lu,I:%lu,D:%lu",
                   (uint32_t)(frame->data[0] << 24 | frame->data[1] << 16 |
                              frame->data[2] << 8 | frame->data[3] << 0),
                   (uint32_t)(frame->data[4] << 24 | frame->data[5] << 16 |
                              frame->data[6] << 8 | frame->data[7] << 0),
                   (uint32_t)(frame->data[8] << 24 | frame->data[9] << 16 |
                              frame->data[10] << 8 | frame->data[11] << 0));
            motor->valid_mask |= SMD_MASK_INFO;
            break;

        case FCT_ORIGIN_SET_LEFT_POS:
            snprintf((char *)motor->info, SMD_INFO_BUF_SIZE,
                   "成功设置左限位原点为：%ld",
                   (int32_t)(frame->data[0] << 24 | frame->data[1] << 16 |
                             frame->data[2] << 8 | frame->data[3] << 0));
            motor->valid_mask |= SMD_MASK_INFO;
            break;

        case FCT_ORIGIN_LIMIT_HOME:
            if (frame->data[0] == 0) {
                snprintf((char *)motor->info, SMD_INFO_BUF_SIZE, "无限位找零");
            } else if (frame->data[0] == 1) {
                snprintf((char *)motor->info, SMD_INFO_BUF_SIZE, "有限位找零");
            }
            motor->valid_mask |= SMD_MASK_INFO;
            break;

        case FCT_ORIGIN_TRIG:
            if (frame->data[0] == 0) {
                snprintf((char *)motor->info, SMD_INFO_BUF_SIZE, "单圈回零");
            } else if (frame->data[0] == 1) {
                snprintf((char *)motor->info, SMD_INFO_BUF_SIZE, "就近回零");
            } else if (frame->data[0] == 2) {
                snprintf((char *)motor->info, SMD_INFO_BUF_SIZE, "多圈回零");
            }
            motor->valid_mask |= SMD_MASK_INFO;
            break;

        case FCT_ORIGIN_BREAK:
            snprintf((char *)motor->info, SMD_INFO_BUF_SIZE,
                     "强制退出回零操作成功");
            motor->valid_mask |= SMD_MASK_INFO;
            break;

        case FCT_ORIGIN_READ_PARAMS:
            snprintf((char *)motor->info, SMD_INFO_BUF_SIZE,
                     "上电回零:%s, 回零状态:%u, 碰撞电流:%d mA, 左原点:%ld, 回零超时:%lu ms, 右原点:%ld, 限位开关:%s",
                     frame->data[0] ? "自动" : "不自动",
                     (unsigned int)frame->data[1],
                     (int16_t)((frame->data[2] << 8) | frame->data[3]),
                     (long)((int32_t)(frame->data[4] << 24 | frame->data[5] << 16 |
                                      frame->data[6] << 8 | frame->data[7] << 0)),
                     (unsigned long)((uint32_t)(frame->data[8] << 24 | frame->data[9] << 16 |
                                                frame->data[10] << 8 | frame->data[11] << 0)),
                     (long)((int32_t)(frame->data[12] << 24 | frame->data[13] << 16 |
                                      frame->data[14] << 8 | frame->data[15] << 0)),
                     frame->data[16] ? "开启" : "关闭");
            motor->valid_mask |= SMD_MASK_INFO;
            break;

        case FCT_ORIGIN_SET_PARAMS:
            snprintf((char *)motor->info, SMD_INFO_BUF_SIZE,
                   "成功设置找零点超时时间为：%lu",
                   (uint32_t)(frame->data[0] << 24 | frame->data[1] << 16 |
                              frame->data[2] << 8 | frame->data[3] << 0));
            motor->valid_mask |= SMD_MASK_INFO;
            break;

        case FCT_ORIGIN_READ_STA:
            if (frame->data[0] == 0) {
                snprintf((char *)motor->info, SMD_INFO_BUF_SIZE, "空闲态");
            } else if (frame->data[0] == 1) {
                snprintf((char *)motor->info, SMD_INFO_BUF_SIZE, "找零点中");
            } else if (frame->data[0] == 2) {
                snprintf((char *)motor->info, SMD_INFO_BUF_SIZE, "成功找到零点");
            } else if (frame->data[0] == 3) {
                snprintf((char *)motor->info, SMD_INFO_BUF_SIZE,
                         "错误状态 未找到零点");
            }
            motor->valid_mask |= SMD_MASK_INFO;
            break;

        case FCT_ORIGIN_AOTO_ZERO:
            if (frame->data[0] == 0) {
                snprintf((char *)motor->info, SMD_INFO_BUF_SIZE,
                         "上电不自动回零");
            } else if (frame->data[0] == 1) {
                snprintf((char *)motor->info, SMD_INFO_BUF_SIZE,
                         "上电自动回零");
            }
            motor->valid_mask |= SMD_MASK_INFO;
            break;

        case FCT_ORIGIN_SET_RIGHT_POS:
            snprintf((char *)motor->info, SMD_INFO_BUF_SIZE,
                   "成功设置右限位原点为：%ld",
                   (int32_t)(frame->data[0] << 24 | frame->data[1] << 16 |
                             frame->data[2] << 8 | frame->data[3] << 0));
            motor->valid_mask |= SMD_MASK_INFO;
            break;

        case FCT_ORIGIN_SWITCH:
            if (frame->data[0] == 0) {
                snprintf((char *)motor->info, SMD_INFO_BUF_SIZE, "关闭左右限位");
            } else if (frame->data[0] == 1) {
                snprintf((char *)motor->info, SMD_INFO_BUF_SIZE, "开启左右限位");
            }
            motor->valid_mask |= SMD_MASK_INFO;
            break;

        case FCT_TORQUE_MODE:
            snprintf((char *)motor->info, SMD_INFO_BUF_SIZE, "设置力矩模式成功");
            motor->valid_mask |= SMD_MASK_INFO;
            break;

        case FCT_SPEED_MODE:
            snprintf((char *)motor->info, SMD_INFO_BUF_SIZE, "设置速度模式成功");
            motor->valid_mask |= SMD_MASK_INFO;
            break;

        case FCT_POS_MODE:
            snprintf((char *)motor->info, SMD_INFO_BUF_SIZE,
                     "绝对位置模式设置: ok");
            motor->valid_mask |= SMD_MASK_INFO;
            break;

        case FCT_POS_REL_MODE:
            snprintf((char *)motor->info, SMD_INFO_BUF_SIZE,
                     "设置相对位置模式成功");
            motor->valid_mask |= SMD_MASK_INFO;
            break;

        case FCT_PULSES_MODE:
            snprintf((char *)motor->info, SMD_INFO_BUF_SIZE, "设置脉冲模式成功");
            motor->valid_mask |= SMD_MASK_INFO;
            break;

        case FCT_PULSE_WIDTH_POS_MODE:
            snprintf((char *)motor->info, SMD_INFO_BUF_SIZE,
                     "设置脉宽位置模式成功");
            motor->valid_mask |= SMD_MASK_INFO;
            break;

        case FCT_PULSE_WIDTH_MA_MODE:
            snprintf((char *)motor->info, SMD_INFO_BUF_SIZE,
                     "设置脉宽力矩模式成功");
            motor->valid_mask |= SMD_MASK_INFO;
            break;

        case FCT_PULSE_WIDTH_SPEED_MODE:
            snprintf((char *)motor->info, SMD_INFO_BUF_SIZE,
                     "设置脉宽速度模式成功");
            motor->valid_mask |= SMD_MASK_INFO;
            break;

        case FCT_OL_SPEED_MODE:
            snprintf((char *)motor->info, SMD_INFO_BUF_SIZE,
                     "设置开环速度模式成功");
            motor->valid_mask |= SMD_MASK_INFO;
            break;

        case FCT_OL_POS_MODE:
            snprintf((char *)motor->info, SMD_INFO_BUF_SIZE,
                     "设置开环绝对位置模式成功");
            motor->valid_mask |= SMD_MASK_INFO;
            break;

        case FCT_OL_POS_REL_MODE:
            snprintf((char *)motor->info, SMD_INFO_BUF_SIZE,
                     "设置开环相对位置模式成功");
            motor->valid_mask |= SMD_MASK_INFO;
            break;

        case FCT_OL_PULSES_MODE:
            snprintf((char *)motor->info, SMD_INFO_BUF_SIZE,
                     "设置开环脉冲模式成功");
            motor->valid_mask |= SMD_MASK_INFO;
            break;

        case FCT_IO_RUN_MODE:
            snprintf((char *)motor->info, SMD_INFO_BUF_SIZE,
                     "设置IO启停模式成功");
            motor->valid_mask |= SMD_MASK_INFO;
            break;

        case FCT_ANGLE_ZERO:
            snprintf((char *)motor->info, SMD_INFO_BUF_SIZE, "清除当前位置成功");
            motor->valid_mask |= SMD_MASK_INFO;
            break;

        case FCT_CLEAR_CLOG_PRO:
            snprintf((char *)motor->info, SMD_INFO_BUF_SIZE, "成功清除堵转状态");
            motor->valid_mask |= SMD_MASK_INFO;
            break;

        case FCT_MOTOR_ENABLE:
            if (frame->data[0] == 0) {
                snprintf((char *)motor->info, SMD_INFO_BUF_SIZE, "使能电机");
            } else if (frame->data[0] == 1) {
                snprintf((char *)motor->info, SMD_INFO_BUF_SIZE, "失能电机");
            }
            motor->enable_sta = frame->data[0];
            motor->valid_mask |= (SMD_MASK_INFO | SMD_MASK_ENABLE_STA);
            break;

        case FCT_CLEAR_STATE:
            snprintf((char *)motor->info, SMD_INFO_BUF_SIZE,
                     "成功清除电机状态（刹车、堵转、失能）");
            motor->valid_mask |= SMD_MASK_INFO;
            break;

        case FCT_STOP_NOW:
            snprintf((char *)motor->info, SMD_INFO_BUF_SIZE, "成功刹停");
            motor->valid_mask |= SMD_MASK_INFO;
            break;

        default:

            return false;
    }
    return true;
}

/**
 * @} process_frame
 */



/*************************************************************************************************
 * @defgroup smd_send 
 * @brief    smd协议发送函数
 * @{
 */

/**
 * @brief   计算校验和函数
 * @param   data: 数据缓冲区
 * @param   len:  长度
 * @retval  校验和
 */
uint8_t smd_checksum(const uint8_t *data, uint8_t length)
{
    uint8_t sum = 0;
    for (uint8_t i = 0; i < length; i++)
    {
        sum += data[i];
    }
    return sum;
}

/**
 * @brief   校准编码器
* @param    addr     : 电机地址
* @retval   从机应答  : 帧头 + 地址 + 功能码 + 命令状态 + 校验字节 + 帧尾
 */
void smd_cal_encoder(uint8_t addr)
{
    uint8_t cmd[16] = {0};

    /* 装载命令 */
    cmd[0] =  FRAME_HEAD;                 /* 帧头 */
    cmd[1] =  addr;                       /* 地址 */
    cmd[2] =  FCT_CAL_ENCODER;            /* 功能码 */
    cmd[3] =  smd_checksum(cmd, 3);       /* 校验和 */
    cmd[4] =  FRAME_TAIL;                 /* 帧尾 */

    /* 发送命令 */
    smd_send_data(cmd, 5);
}

/**
 * @brief    复位重启
 * @param    addr    : 电机地址
 * @retval   从机应答  : 帧头 + 地址 + 功能码 + 命令状态 + 校验字节 + 帧尾
 */
void smd_restart(uint8_t addr)
{
    uint8_t cmd[16] = {0};

    /* 装载命令 */
    cmd[0] =  FRAME_HEAD;                 /* 帧头 */
    cmd[1] =  addr;                       /* 地址 */
    cmd[2] =  FCT_RESTART;                /* 功能码 */
    cmd[3] =  smd_checksum(cmd, 3);       /* 校验和 */
    cmd[4] =  FRAME_TAIL;                 /* 帧尾 */

    /* 发送命令 */
    smd_send_data(cmd, 5);
}

/**
 * @brief    恢复出厂设置
 * @param    addr    :  电机地址
 * @retval   从机应答  : 帧头 + 地址 + 功能码 + 命令状态 + 校验字节 + 帧尾
 */
void smd_reset_factory(uint8_t addr)
{
    uint8_t cmd[16] = {0};

    /* 装载命令 */
    cmd[0] =  FRAME_HEAD;                 /* 帧头 */
    cmd[1] =  addr;                       /* 地址 */
    cmd[2] =  FCT_RESET_FACTORY;          /* 功能码 */
    cmd[3] =  smd_checksum(cmd, 3);       /* 校验和 */
    cmd[4] =  FRAME_TAIL;                 /* 帧尾 */

    /* 发送命令 */
    smd_send_data(cmd, 5);
}

/**
 * @brief    参数保存
 * @param    addr     :  电机地址
 * @retval   从机应答  :  帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_param_save(uint8_t addr)
{
    uint8_t cmd[16] = {0};

    /* 装载命令 */
    cmd[0] =  FRAME_HEAD;                 /* 帧头 */
    cmd[1] =  addr;                       /* 地址 */
    cmd[2] =  FCT_PARAM_SAVE;             /* 功能码 */
    cmd[3] =  smd_checksum(cmd, 3);       /* 校验和 */
    cmd[4] =  FRAME_TAIL;                 /* 帧尾 */

    /* 发送命令 */
    smd_send_data(cmd, 5);
}

/**
 * @brief    读取软硬件版本信息
 * @param    addr    :  电机地址
 * @retval   从机应答  : 帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_read_soft_hard_ver(uint8_t addr)
{
    uint8_t cmd[16] = {0};

    /* 装载命令 */
    cmd[0] =  FRAME_HEAD;                 /* 帧头 */
    cmd[1] =  addr;                       /* 地址 */
    cmd[2] =  FCT_READ_SOFT_HARD_VER;     /* 功能码 */
    cmd[3] =  smd_checksum(cmd, 3);       /* 校验和 */
    cmd[4] =  FRAME_TAIL;                 /* 帧尾 */

    /* 发送命令 */
    smd_send_data(cmd, 5);
}

/**
 * @brief    读取相电阻和相电感
 * @param    addr    :  电机地址
 * @retval   从机应答  : 帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_read_psi(uint8_t addr)
{
    uint8_t cmd[16] = {0};

    /* 装载命令 */
    cmd[0] =  FRAME_HEAD;                 /* 帧头 */
    cmd[1] =  addr;                       /* 地址 */
    cmd[2] =  FCT_READ_PSI;               /* 功能码 */
    cmd[3] =  smd_checksum(cmd, 3);       /* 校验和 */
    cmd[4] =  FRAME_TAIL;                 /* 帧尾 */

    /* 发送命令 */
    smd_send_data(cmd, 5);
}

/**
 * @brief    读取相电阻和相电感
 * @param    addr    :  电机地址
 * @retval   从机应答  : 帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_read_phase_res_ind(uint8_t addr)
{
    uint8_t cmd[16] = {0};

    /* 装载命令 */
    cmd[0] =  FRAME_HEAD;                 /* 帧头 */
    cmd[1] =  addr;                       /* 地址 */
    cmd[2] =  FCT_READ_PHASE_RES_IND;     /* 功能码 */
    cmd[3] =  smd_checksum(cmd, 3);       /* 校验和 */
    cmd[4] =  FRAME_TAIL;                 /* 帧尾 */

    /* 发送命令 */
    smd_send_data(cmd, 5);
}

/**
 * @brief    读取相电流
 * @param    addr    :  电机地址
 * @retval   从机应答  : 帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_read_phase_ma(uint8_t addr)
{
    uint8_t cmd[16] = {0};

    /* 装载命令 */
    cmd[0] =  FRAME_HEAD;                 /* 帧头 */
    cmd[1] =  addr;                       /* 地址 */
    cmd[2] =  FCT_READ_PHASE_MA;          /* 功能码 */
    cmd[3] =  smd_checksum(cmd, 3);       /* 校验和 */
    cmd[4] =  FRAME_TAIL;                 /* 帧尾 */

    /* 发送命令 */
    smd_send_data(cmd, 5);
}

/**
 * @brief    读取总线电压
 * @param    addr    :  电机地址
 * @retval   从机应答  : 帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_read_vol(uint8_t addr)
{
    uint8_t cmd[16] = {0};

    /* 装载命令 */
    cmd[0] =  FRAME_HEAD;                 /* 帧头 */
    cmd[1] =  addr;                       /* 地址 */
    cmd[2] =  FCT_READ_VOL;               /* 功能码 */
    cmd[3] =  smd_checksum(cmd, 3);       /* 校验和 */
    cmd[4] =  FRAME_TAIL;                 /* 帧尾 */

    /* 发送命令 */
    smd_send_data(cmd, 5);
}

/**
 * @brief    读取电流环PID参数
 * @param    addr    :  电机地址
 * @retval   从机应答  : 帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_read_ma_pid(uint8_t addr)
{
    uint8_t cmd[16] = {0};

    /* 装载命令 */
    cmd[0] =  FRAME_HEAD;                 /* 帧头 */
    cmd[1] =  addr;                       /* 地址 */
    cmd[2] =  FCT_READ_MA_PID;            /* 功能码 */
    cmd[3] =  smd_checksum(cmd, 3);       /* 校验和 */
    cmd[4] =  FRAME_TAIL;                 /* 帧尾 */

    /* 发送命令 */
    smd_send_data(cmd, 5);
}

/**
 * @brief    读取速度环PID参数
 * @param    addr    :  电机地址
 * @retval   从机应答  : 帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_read_speed_pid(uint8_t addr)
{
    uint8_t cmd[16] = {0};

    /* 装载命令 */
    cmd[0] =  FRAME_HEAD;                 /* 帧头 */
    cmd[1] =  addr;                       /* 地址 */
    cmd[2] =  FCT_READ_SPEED_PID;         /* 功能码 */
    cmd[3] =  smd_checksum(cmd, 3);       /* 校验和 */
    cmd[4] =  FRAME_TAIL;                 /* 帧尾 */

    /* 发送命令 */
    smd_send_data(cmd, 5);
}


/**
 * @brief    读取位置环PID参数
 * @param    addr    :  电机地址
 * @retval   从机应答  : 帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_read_pos_pid(uint8_t addr)
{
    uint8_t cmd[16] = {0};

    /* 装载命令 */
    cmd[0] =  FRAME_HEAD;                 /* 帧头 */
    cmd[1] =  addr;                       /* 地址 */
    cmd[2] =  FCT_READ_POS_PID;           /* 功能码 */
    cmd[3] =  smd_checksum(cmd, 3);       /* 校验和 */
    cmd[4] =  FRAME_TAIL;                 /* 帧尾 */

    /* 发送命令 */
    smd_send_data(cmd, 5);
}

/**
 * @brief    读取输入累计脉冲数
 * @param    addr    :  电机地址
 * @retval   从机应答  : 帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_read_tatal_pulse(uint8_t addr)
{
    uint8_t cmd[16] = {0};

    /* 装载命令 */
    cmd[0] =  FRAME_HEAD;                 /* 帧头 */
    cmd[1] =  addr;                       /* 地址 */
    cmd[2] =  FCT_READ_TOTAL_PULSE;       /* 功能码 */
    cmd[3] =  smd_checksum(cmd, 3);       /* 校验和 */
    cmd[4] =  FRAME_TAIL;                 /* 帧尾 */

    /* 发送命令 */
    smd_send_data(cmd, 5);
}

/**
 * @brief    读电机实时转速
 * @param    addr    :  电机地址
 * @retval   从机应答  : 帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_read_rotate_speed(uint8_t addr)
{
    uint8_t cmd[16] = {0};

    /* 装载命令 */
    cmd[0] =  FRAME_HEAD;                 /* 帧头 */
    cmd[1] =  addr;                       /* 地址 */
    cmd[2] =  FCT_READ_ROTATE_SPEED;      /* 功能码 */
    cmd[3] =  smd_checksum(cmd, 3);       /* 校验和 */
    cmd[4] =  FRAME_TAIL;                 /* 帧尾 */

    /* 发送命令 */
    smd_send_data(cmd, 5);
}

/**
 * @brief    读取电机实时位置
 * @param    addr    :  电机地址
 * @retval   从机应答  : 帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_read_pos(uint8_t addr)
{
    uint8_t cmd[16] = {0};

    /* 装载命令 */
    cmd[0] =  FRAME_HEAD;                 /* 帧头 */
    cmd[1] =  addr;                       /* 地址 */
    cmd[2] =  FCT_READ_POS;               /* 功能码 */
    cmd[3] =  smd_checksum(cmd, 3);       /* 校验和 */
    cmd[4] =  FRAME_TAIL;                 /* 帧尾 */

    /* 发送命令 */
    smd_send_data(cmd, 5);
}

/**
 * @brief    读取电机位置误差
 * @param    addr    :  电机地址
 * @retval   从机应答  : 帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_read_pos_error(uint8_t addr)
{
    uint8_t cmd[16] = {0};

    /* 装载命令 */
    cmd[0] =  FRAME_HEAD;                 /* 帧头 */
    cmd[1] =  addr;                       /* 地址 */
    cmd[2] =  FCT_READ_POS_ERROR;         /* 功能码 */
    cmd[3] =  smd_checksum(cmd, 3);       /* 校验和 */
    cmd[4] =  FRAME_TAIL;                 /* 帧尾 */

    /* 发送命令 */
    smd_send_data(cmd, 5);
}

/**
 * @brief    读取电机运行状态
 * @param    addr    :  电机地址
 * @retval   从机应答  : 帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_read_motor_sta(uint8_t addr)
{
    uint8_t cmd[16] = {0};

    /* 装载命令 */
    cmd[0] =  FRAME_HEAD;                 /* 帧头 */
    cmd[1] =  addr;                       /* 地址 */
    cmd[2] =  FCT_READ_MOTOR_STA;         /* 功能码 */
    cmd[3] =  smd_checksum(cmd, 3);       /* 校验和 */
    cmd[4] =  FRAME_TAIL;                 /* 帧尾 */

    /* 发送命令 */
    smd_send_data(cmd, 5);
}

/**
 * @brief    读取堵转标志
 * @param    addr    :  电机地址
 * @retval   从机应答  : 帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_read_clog_flag(uint8_t addr)
{
    uint8_t cmd[16] = {0};

    /* 装载命令 */
    cmd[0] =  FRAME_HEAD;                 /* 帧头 */
    cmd[1] =  addr;                       /* 地址 */
    cmd[2] =  FCT_READ_CLOG_FLAG;         /* 功能码 */
    cmd[3] =  smd_checksum(cmd, 3);       /* 校验和 */
    cmd[4] =  FRAME_TAIL;                 /* 帧尾 */

    /* 发送命令 */
    smd_send_data(cmd, 5);
}

/**
 * @brief    读取堵转电流
 * @param    addr    :  电机地址
 * @retval   从机应答  : 帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_read_clog_current(uint8_t addr)
{
    uint8_t cmd[16] = {0};

    /* 装载命令 */
    cmd[0] =  FRAME_HEAD;                 /* 帧头 */
    cmd[1] =  addr;                       /* 地址 */
    cmd[2] =  FCT_READ_CLOG_CUR;          /* 功能码 */
    cmd[3] =  smd_checksum(cmd, 3);       /* 校验和 */
    cmd[4] =  FRAME_TAIL;                 /* 帧尾 */

    /* 发送命令 */
    smd_send_data(cmd, 5);
}

/**
 * @brief    读使能状态
 * @param    addr    :  电机地址
 * @retval   从机应答  : 帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_read_enable_sta(uint8_t addr)
{
    uint8_t cmd[16] = {0};

    /* 装载命令 */
    cmd[0] =  FRAME_HEAD;                 /* 帧头 */
    cmd[1] =  addr;                       /* 地址 */
    cmd[2] =  FCT_READ_ENABLE_STA;        /* 功能码 */
    cmd[3] =  smd_checksum(cmd, 3);       /* 校验和 */
    cmd[4] =  FRAME_TAIL;                 /* 帧尾 */

    /* 发送命令 */
    smd_send_data(cmd, 5);
}

/**
 * @brief    读到位状态
 * @param    addr    :  电机地址
 * @retval   从机应答  : 帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_read_arrived_sta(uint8_t addr)
{
    uint8_t cmd[16] = {0};

    /* 装载命令 */
    cmd[0] =  FRAME_HEAD;                 /* 帧头 */
    cmd[1] =  addr;                       /* 地址 */
    cmd[2] =  FCT_READ_ARRIVED_STA;       /* 功能码 */
    cmd[3] =  smd_checksum(cmd, 3);       /* 校验和 */
    cmd[4] =  FRAME_TAIL;                 /* 帧尾 */

    /* 发送命令 */
    smd_send_data(cmd, 5);
}

/**
 * @brief    读取系统参数
 * @param    addr    :  电机地址
 * @retval   从机应答  : 帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_read_sys_params(uint8_t addr)
{
    uint8_t cmd[16] = {0};

    /* 装载命令 */
    cmd[0] =  FRAME_HEAD;                 /* 帧头 */
    cmd[1] =  addr;                       /* 地址 */
    cmd[2] =  FCT_READ_SYS_PARAM;         /* 功能码 */
    cmd[3] =  smd_checksum(cmd, 3);       /* 校验和 */
    cmd[4] =  FRAME_TAIL;                 /* 帧尾 */

    /* 发送命令 */
    smd_send_data(cmd, 5);
}

/**
 * @brief    读取驱动参数
 * @param    addr    :  电机地址
 * @retval   从机应答  : 帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_read_drive_params(uint8_t addr)
{
    uint8_t cmd[16] = {0};

    /* 装载命令 */
    cmd[0] =  FRAME_HEAD;                 /* 帧头 */
    cmd[1] =  addr;                       /* 地址 */
    cmd[2] =  FCT_READ_DRIVE_PARAMS;      /* 功能码 */
    cmd[3] =  smd_checksum(cmd, 3);       /* 校验和 */
    cmd[4] =  FRAME_TAIL;                 /* 帧尾 */

    /* 发送命令 */
    smd_send_data(cmd, 5);
}

/**
 * @brief    设置从机地址
 * @param    addr     :  电机地址
 * @param    new_addr :  要设置的新地址
 * @retval   从机应答 :  帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_set_slave_add(uint8_t addr, uint8_t new_addr)
{
    uint8_t cmd[16] = {0};

    /* 装载命令 */
    cmd[0] =  FRAME_HEAD;                 /* 帧头 */
    cmd[1] =  addr;                       /* 地址 */
    cmd[2] =  FCT_SET_SLAVE_ADD;          /* 功能码 */
    cmd[3] =  new_addr;                   /* 电机新的地址 */
    cmd[4] =  smd_checksum(cmd, 4);       /* 校验和 */
    cmd[5] =  FRAME_TAIL;                 /* 帧尾 */

    /* 发送命令 */
    smd_send_data(cmd, 6);
}

/**
 * @brief    设置分组地址
 * @param    addr     :  电机地址
 * @param    new_addr :  要设置的新地址
 * @retval   从机应答 :  帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_set_group_add(uint8_t addr, uint8_t new_addr)
{
    uint8_t cmd[16] = {0};

    /* 装载命令 */
    cmd[0] =  FRAME_HEAD;                 /* 帧头 */
    cmd[1] =  addr;                       /* 地址 */
    cmd[2] =  FCT_SET_GROUP_ADD;          /* 功能码 */
    cmd[3] =  new_addr;                   /* 电机新的地址 */
    cmd[4] =  smd_checksum(cmd, 4);       /* 校验和 */
    cmd[5] =  FRAME_TAIL;                 /* 帧尾 */

    /* 发送命令 */
    smd_send_data(cmd, 6);
}

/**
 * @brief    设置工作模式
 * @param    addr   :   电机地址
 * @param    mode   :   工作模式1~7分别对应：通信位置模式、通信速度模式、通信力矩模式、
                                    脉宽位置模式、脉宽速度模式、脉宽力矩模式、脉冲模式
 * @retval   从机应答 :  帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_set_mode(uint8_t addr, uint8_t mode)
{
    uint8_t cmd[16] = {0};

    /* 装载命令 */
    cmd[0] =  FRAME_HEAD;                 /* 帧头 */
    cmd[1] =  addr;                       /* 地址 */
    cmd[2] =  FCT_SET_MODE;               /* 功能码 */
    cmd[3] =  mode;                       /* 工作模式 */
    cmd[4] =  smd_checksum(cmd, 4);       /* 校验和 */
    cmd[5] =  FRAME_TAIL;                 /* 帧尾 */

    /* 发送命令 */
    smd_send_data(cmd, 6);
}

/**
 * @brief    设置位置环PID参数
 * @param    addr   :   电机地址
 * @param    
 * @retval   从机应答 :  帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_set_pos_pid(uint8_t addr, uint32_t kp, uint32_t ki, uint32_t kd)
{
    uint8_t cmd[32] = {0};

    /* 装载命令 */
    cmd[0] =  FRAME_HEAD;                 /* 帧头 */
    cmd[1] =  addr;                       /* 地址 */
    cmd[2] =  FCT_SET_POS_PID;               /* 功能码 */
    /* PID参数 */
    cmd[3] = (uint8_t)((kp >> 24) & 0xFF);  
    cmd[4] = (uint8_t)((kp >> 16) & 0xFF);
    cmd[5] = (uint8_t)((kp >> 8) & 0xFF);
    cmd[6] = (uint8_t)((kp >> 0) & 0xFF);
    
    cmd[7] = (uint8_t)((ki >> 24) & 0xFF);  
    cmd[8] = (uint8_t)((ki >> 16) & 0xFF);
    cmd[9] = (uint8_t)((ki >> 8) & 0xFF);
    cmd[10] = (uint8_t)((ki >> 0) & 0xFF);

    cmd[11] = (uint8_t)((kd >> 24) & 0xFF);  
    cmd[12] = (uint8_t)((kd >> 16) & 0xFF);
    cmd[13] = (uint8_t)((kd >> 8) & 0xFF);
    cmd[14] = (uint8_t)((kd >> 0) & 0xFF);

    cmd[15] =  smd_checksum(cmd, 15);       /* 校验和 */
    cmd[16] =  FRAME_TAIL;                 /* 帧尾 */

    /* 发送命令 */
    smd_send_data(cmd, 17);
}

/**
 * @brief    设置位置环力矩限制
 * @param    addr   :   电机地址
 * @param    torque :   位置环力矩限制（100~3000mA）
 * @retval   从机应答 :  帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_set_pos_torque(uint8_t addr, int16_t torque)
{
    uint8_t cmd[16] = {0};

    /* 装载命令 */
    cmd[0] =  FRAME_HEAD;                 /* 帧头 */
    cmd[1] =  addr;                       /* 地址 */
    cmd[2] =  FCT_SET_POS_TORQUE;         /* 功能码 */
    /* 力矩参数 */
    cmd[3] = (uint8_t)((torque >> 8) & 0xFF);  
    cmd[4] = (uint8_t)((torque >> 0) & 0xFF);
    cmd[5] =  smd_checksum(cmd, 5);       /* 校验和 */
    cmd[6] =  FRAME_TAIL;                 /* 帧尾 */

    /* 发送命令 */
    smd_send_data(cmd, 7);
}

/**
 * @brief    设置细分
 * @param    addr   :  电机地址
 * @param    step   :  细分(取值范围：1~256)
 * @retval   从机应答 : 帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_set_step(uint8_t addr, uint16_t step)
{
    uint8_t cmd[16] = {0};

    /* 装载命令 */
    cmd[0] =  FRAME_HEAD;                   /* 帧头 */
    cmd[1] =  addr;                         /* 地址 */
    cmd[2] =  FCT_SET_STEP;                 /* 功能码 */
    cmd[3] =  (uint8_t)((step >> 8) & 0xFF);/* 细分 */
    cmd[4] =  (uint8_t)((step >> 0) & 0xFF);/* 细分 */
    cmd[5] =  smd_checksum(cmd, 5);         /* 校验和 */
    cmd[6] =  FRAME_TAIL;                   /* 帧尾 */

    /* 发送命令 */
    smd_send_data(cmd, 7);
}

/**
 * @brief    设置目标电流（仅力矩模式下有效）
 * @param    addr   :  电机地址
 * @param    ma     :  电流（0~3000mA）
 * @retval   从机应答 : 帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_set_ma(uint8_t addr, int16_t ma)
{
    uint8_t cmd[16] = {0};

    /* 装载命令 */
    cmd[0] =  FRAME_HEAD;                 /* 帧头 */
    cmd[1] =  addr;                       /* 地址 */
    cmd[2] =  FCT_SET_MA;                 /* 功能码 */
    /* 电流值mA */
    cmd[3] = (uint8_t)((ma >> 8) & 0xFF);  
    cmd[4] = (uint8_t)((ma >> 0) & 0xFF);
    
    cmd[5] =  smd_checksum(cmd, 5);       /* 校验和 */
    cmd[6] =  FRAME_TAIL;                 /* 帧尾 */

    /* 发送命令 */
    smd_send_data(cmd, 7);
}

/**
 * @brief    设置串口波特率
 * @param    addr    :  电机地址
 * @param    baud    :  波特率
 * @retval   从机应答：帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_set_uart_baud(uint8_t addr, uint32_t baud)
{
    uint8_t cmd[16] = {0};

    /* 装载命令 */
    cmd[0] =  FRAME_HEAD;                 /* 帧头 */
    cmd[1] =  addr;                       /* 地址 */
    cmd[2] =  FCT_SET_UART_BAUD;          /* 功能码 */
    /* 波特率 */
    cmd[3] = (uint8_t)((baud >> 24) & 0xFF);  
    cmd[4] = (uint8_t)((baud >> 16) & 0xFF);
    cmd[5] = (uint8_t)((baud >> 8) & 0xFF);
    cmd[6] = (uint8_t)((baud >> 0) & 0xFF); 
    cmd[7] =  smd_checksum(cmd, 7);       /* 校验和 */
    cmd[8] =  FRAME_TAIL;                 /* 帧尾 */

    /* 发送命令 */
    smd_send_data(cmd, 9);
}

/**
 * @brief    设置CAN波特率
 * @param    addr    :  电机地址
 * @param    baud    :  波特率（单位Kbps）
 * @retval   从机应答：帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_set_can_baud(uint8_t addr, uint16_t baud)
{
    uint8_t cmd[16] = {0};

    /* 装载命令 */
    cmd[0] =  FRAME_HEAD;                 /* 帧头 */
    cmd[1] =  addr;                       /* 地址 */
    cmd[2] =  FCT_SET_CAN_BAUD;           /* 功能码 */
    /* 波特率 */
    cmd[3] = (uint8_t)((baud >> 8) & 0xFF);  
    cmd[4] = (uint8_t)((baud >> 0) & 0xFF);
    cmd[5] =  smd_checksum(cmd, 5);       /* 校验和 */
    cmd[6] =  FRAME_TAIL;                 /* 帧尾 */

    /* 发送命令 */
    smd_send_data(cmd, 7);
}

/**
 * @brief    设置MODBUS
 * @param    addr      :  电机地址
 * @param    modbus    :  是否使用MODBUS协议
 * @retval   从机应答：帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_set_modbus(uint8_t addr, uint8_t modbus)
{
    uint8_t cmd[16] = {0};

    /* 装载命令 */
    cmd[0] =  FRAME_HEAD;                 /* 帧头 */
    cmd[1] =  addr;                       /* 地址 */
    cmd[2] =  FCT_SET_MODBUS;             /* 功能码 */
    cmd[3] =  modbus;                     /* 填充字节 */
    cmd[4] =  smd_checksum(cmd, 4);       /* 校验和 */
    cmd[5] =  FRAME_TAIL;                 /* 帧尾 */

    /* 发送命令 */
    smd_send_data(cmd, 6);
}

/**
 * @brief    设置堵转保护
 * @param    addr    :  电机地址
 * @param    en      :  1开启堵转保护 0关闭堵转保护
 * @retval   从机应答  : 帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_set_clog_pro(uint8_t addr, uint8_t en)
{
    uint8_t cmd[16] = {0};

    /* 装载命令 */
    cmd[0] =  FRAME_HEAD;                 /* 帧头 */
    cmd[1] =  addr;                       /* 地址 */
    cmd[2] =  FCT_SET_CLOG_PRO;           /* 功能码 */
    cmd[3] =  en;                         /* 堵转保护标志 */
    cmd[4] =  smd_checksum(cmd, 4);       /* 校验和 */
    cmd[5] =  FRAME_TAIL;                 /* 帧尾 */

    /* 发送命令 */
    smd_send_data(cmd, 6);
}


/**
 * @brief    设置堵转电流
 * @param    addr    :  电机地址
 * @param    ma      :  0~3000mA
 * @retval   从机应答  : 帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_set_clog_current(uint8_t addr, int16_t ma)
{
    uint8_t cmd[16] = {0};

    /* 装载命令 */
    cmd[0] =  FRAME_HEAD;                 /* 帧头 */
    cmd[1] =  addr;                       /* 地址 */
    cmd[2] =  FCT_SET_CLOG_CUR;           /* 功能码 */
    cmd[3] =  (uint8_t)((ma >> 8) & 0xFF);/* 低字节 */    
    cmd[4] =  (uint8_t)((ma >> 0) & 0xFF);/* 高字节 */
    cmd[5] =  smd_checksum(cmd, 5);       /* 校验和 */
    cmd[6] =  FRAME_TAIL;                 /* 帧尾 */

    /* 发送命令 */
    smd_send_data(cmd, 7);
}

/**
 * @brief    设置CAN_ID
 * @param    addr     :  电机地址
 * @param    id       :  29位扩展帧ID
 * @retval   从机应答  :  帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_set_can_id(uint8_t addr, uint32_t id)
{
    uint8_t cmd[16] = {0};

    /* 装载命令 */
    cmd[0] =  FRAME_HEAD;                 /* 帧头 */
    cmd[1] =  addr;                       /* 地址 */
    cmd[2] =  FCT_SET_CAN_ID;             /* 功能码 */
    cmd[3] = (uint8_t)((id >> 24) & 0xFF);        /* 低字节 */
    cmd[4] = (uint8_t)((id >> 16) & 0xFF); /* 高字节 */
    cmd[5] = (uint8_t)((id >> 8) & 0xFF);  
    cmd[6] = (uint8_t)((id >> 0) & 0xFF); 
    cmd[7] =  smd_checksum(cmd, 7);       /* 校验和 */
    cmd[8] =  FRAME_TAIL;                 /* 帧尾 */

    /* 发送命令 */
    smd_send_data(cmd, 9);
}

/**
 * @brief    设置DIR正转电平
 * @param    addr   :  电机地址
 * @param    dir    :  方向  0 高电平正转  1 低电平反转
 * @retval   从机应答：帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_set_dir_level(uint8_t addr,uint8_t dir)
{
    uint8_t cmd[16] = {0};

    /* 装载命令 */
    cmd[0] =  FRAME_HEAD;                 /* 帧头 */
    cmd[1] =  addr;                       /* 地址 */
    cmd[2] =  FCT_SET_DIR_LEVEL;          /* 功能码 */
    cmd[3] =  dir;                        /* 旋转方向 */
    cmd[4] =  smd_checksum(cmd, 4);       /* 校验和 */
    cmd[5] =  FRAME_TAIL;                 /* 帧尾 */

    /* 发送命令 */
    smd_send_data(cmd, 6);
}

/**
 * @brief    设置EN脚有效电平
 * @param    addr   :  电机地址
 * @param    en     :  0：低电平有效， 1：高电平有效， 2：保持有效
 * @retval   从机应答：帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_set_en_level(uint8_t addr,uint8_t en)
{
    uint8_t cmd[16] = {0};

    /* 装载命令 */
    cmd[0] =  FRAME_HEAD;                 /* 帧头 */
    cmd[1] =  addr;                       /* 地址 */
    cmd[2] =  FCT_SET_EN_LEVEL;           /* 功能码 */
    cmd[3] =  en;                         /* EN */
    cmd[4] =  smd_checksum(cmd, 4);       /* 校验和 */
    cmd[5] =  FRAME_TAIL;                 /* 帧尾 */

    /* 发送命令 */
    smd_send_data(cmd, 6);
}


/**
 * @brief    设置指令是否回响
 * @param    addr   :  电机地址
 * @param    echo   :  0：回响 ， 1：不回响
 * @retval   从机应答：帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_set_cmd_echo(uint8_t addr,uint8_t echo)
{
    uint8_t cmd[16] = {0};

    /* 装载命令 */
    cmd[0] =  FRAME_HEAD;                 /* 帧头 */
    cmd[1] =  addr;                       /* 地址 */
    cmd[2] =  FCT_SET_CMD_ECHO;           /* 功能码 */
    cmd[3] =  echo;                       /* 回响标志 */
    cmd[4] =  smd_checksum(cmd, 4);       /* 校验和 */
    cmd[5] =  FRAME_TAIL;                 /* 帧尾 */

    /* 发送命令 */
    smd_send_data(cmd, 6);
}

/**
 * @brief    设置按键锁定
 * @param    addr     :  电机地址
 * @param    lock     :  1 上锁 0 解锁
 * @retval   从机应答  : 帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_set_key_lock(uint8_t addr, uint8_t lock)
{
    uint8_t cmd[16] = {0};

    /* 装载命令 */
    cmd[0] =  FRAME_HEAD;                 /* 帧头 */
    cmd[1] =  addr;                       /* 地址 */
    cmd[2] =  FCT_SET_KEY_LOCK;           /* 功能码 */
    cmd[3] =  lock;                       /* 上锁标志 */
    cmd[4] =  smd_checksum(cmd, 4);       /* 校验和 */
    cmd[5] =  FRAME_TAIL;                 /* 帧尾 */

    /* 发送命令 */
    smd_send_data(cmd, 6);
}

/**
 * @brief    设置自动熄屏
 * @param    addr    :  电机地址
 * @param    en      :  1 开启自动熄屏， 0 关闭自动熄屏，
 * @retval   从机应答  : 帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_set_auto_not_display(uint8_t addr, uint8_t en)
{
    uint8_t cmd[16] = {0};

    /* 装载命令 */
    cmd[0] =  FRAME_HEAD;                 /* 帧头 */
    cmd[1] =  addr;                       /* 地址 */
    cmd[2] =  FCT_SET_AUTO_NOT_DISPLAY;   /* 功能码 */
    cmd[3] =  en;                         /* 自动熄屏标志 */
    cmd[4] =  smd_checksum(cmd, 4);       /* 校验和 */
    cmd[5] =  FRAME_TAIL;                 /* 帧尾 */

    /* 发送命令 */
    smd_send_data(cmd, 6);
}


/**
 * @brief   设置IO启动电平
 * @param    addr    :  电机地址
 * @param    level   :  0 低电平启动 1 高电平启动
 * @retval   从机应答  : 帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_set_io_start_level(uint8_t addr, uint8_t level)
{
    uint8_t cmd[16] = {0};

    /* 装载命令 */
    cmd[0] =  FRAME_HEAD;                 /* 帧头 */
    cmd[1] =  addr;                       /* 地址 */
    cmd[2] =  FCT_SET_IO_START_LEVEL;     /* 功能码 */
    cmd[3] =  level;                      /* 自动熄屏标志 */
    cmd[4] =  smd_checksum(cmd, 4);       /* 校验和 */
    cmd[5] =  FRAME_TAIL;                 /* 帧尾 */

    /* 发送命令 */
    smd_send_data(cmd, 6);
}

/**
 * @brief    设置速度环PID参数
 * @param    addr   :   电机地址
 * @param    
 * @retval   从机应答 :  帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_set_speed_pid(uint8_t addr, uint32_t kp, uint32_t ki, uint32_t kd)
{
    uint8_t cmd[32] = {0};

    /* 装载命令 */
    cmd[0] =  FRAME_HEAD;                 /* 帧头 */
    cmd[1] =  addr;                       /* 地址 */
    cmd[2] =  FCT_SET_SPEED_PID;          /* 功能码 */
    /* PID参数 */
    cmd[3] = (uint8_t)((kp >> 24) & 0xFF);  
    cmd[4] = (uint8_t)((kp >> 16) & 0xFF);
    cmd[5] = (uint8_t)((kp >> 8) & 0xFF);
    cmd[6] = (uint8_t)((kp >> 0) & 0xFF);
    
    cmd[7] = (uint8_t)((ki >> 24) & 0xFF);  
    cmd[8] = (uint8_t)((ki >> 16) & 0xFF);
    cmd[9] = (uint8_t)((ki >> 8) & 0xFF);
    cmd[10] = (uint8_t)((ki >> 0) & 0xFF);

    cmd[11] = (uint8_t)((kd >> 24) & 0xFF);  
    cmd[12] = (uint8_t)((kd >> 16) & 0xFF);
    cmd[13] = (uint8_t)((kd >> 8) & 0xFF);
    cmd[14] = (uint8_t)((kd >> 0) & 0xFF);

    cmd[15] =  smd_checksum(cmd, 15);      /* 校验和 */
    cmd[16] =  FRAME_TAIL;                 /* 帧尾 */

    /* 发送命令 */
    smd_send_data(cmd, 17);
}

/**
 * @brief    设置左限位原点位置
 * @param    addr     :  电机地址
 * @param    pos      :  零点位置
 * @retval   从机应答  : 帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_origin_set_left_pos(uint8_t addr, int32_t pos)
{
    uint8_t cmd[16] = {0};

    /* 装载命令 */
    cmd[0] =  FRAME_HEAD;                   /* 帧头 */
    cmd[1] =  addr;                         /* 地址 */
    cmd[2] =  FCT_ORIGIN_SET_LEFT_POS;      /* 功能码 */
    cmd[3] = (uint8_t)((pos >> 24) & 0xFF); /* 零点位置 */
    cmd[4] = (uint8_t)((pos >> 16) & 0xFF);
    cmd[5] = (uint8_t)((pos >> 8) & 0xFF);
    cmd[6] = (uint8_t)((pos >> 0) & 0xFF); 
    cmd[7] =  smd_checksum(cmd, 7);         /* 校验和 */
    cmd[8] =  FRAME_TAIL;                   /* 帧尾 */

    /* 发送命令 */
    smd_send_data(cmd, 9);
}

/**
 * @brief    找零点（有限位回零或无限位回零）
 * @param    addr              :  电机地址
 * @param    limit_enable      :  0 无限位  1 有限位
 * @param    dir               :  0 正转    1 反转
 * @param    speed_rpm         :  转速RPM(600RPM以内 建议100RPM)
 * @param    curr_limit        :  无限位判断到位电流 mA(仅对无限位回零有效建议200~1000)
 * @retval   从机应答  : 帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_origin_homing_by_limit(uint8_t addr, uint8_t limit_enable, uint8_t dir, int32_t speed_rpm, int16_t curr_limit)
{
    uint8_t cmd[32] = {0};

    /* 装载命令 */
    cmd[0] =  FRAME_HEAD;                 /* 帧头 */
    cmd[1] =  addr;                       /* 地址 */
    cmd[2] =  FCT_ORIGIN_LIMIT_HOME;      /* 功能码 */
    cmd[3] = limit_enable;                /* 有无限位 */
    cmd[4] = dir;                         /* 方向 */
    cmd[5] = (uint8_t)((speed_rpm >> 24) & 0xFF);    
    cmd[6] = (uint8_t)((speed_rpm >> 16) & 0xFF);
    cmd[7] = (uint8_t)((speed_rpm >> 8) & 0xFF);    
    cmd[8] = (uint8_t)((speed_rpm >> 0) & 0xFF);
    
    cmd[9] = (uint8_t)((curr_limit >> 8) & 0xFF);    
    cmd[10] = (uint8_t)((curr_limit >> 0) & 0xFF);
    
    cmd[11] =  smd_checksum(cmd, 11);     /* 校验和 */
    cmd[12] =  FRAME_TAIL;                /* 帧尾 */

    /* 发送命令 */
    smd_send_data(cmd, 13);
}

/**
 * @brief    触发回零
 * @param    addr     :  电机地址
 * @param    mode     :  0 单圈回零  1 就近回零  2 多圈回零
 * @retval   从机应答  :  帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_origin_trig(uint8_t addr, uint8_t mode)
{
    uint8_t cmd[16] = {0};

    /* 装载命令 */
    cmd[0] =  FRAME_HEAD;                 /* 帧头 */
    cmd[1] =  addr;                       /* 地址 */
    cmd[2] =  FCT_ORIGIN_TRIG;            /* 功能码 */
    cmd[3] =  mode;                       /* 回零方式 */
    cmd[4] =  smd_checksum(cmd, 4);       /* 校验和 */
    cmd[5] =  FRAME_TAIL;                 /* 帧尾 */

    /* 发送命令 */
    smd_send_data(cmd, 6);
}

/**
 * @brief    强制中断并退出回零操作
 * @param    addr     :  电机地址
 * @retval   从机应答  :  帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_origin_break(uint8_t addr)
{
    uint8_t cmd[16] = {0};

    /* 装载命令 */
    cmd[0] =  FRAME_HEAD;                 /* 帧头 */
    cmd[1] =  addr;                       /* 地址 */
    cmd[2] =  FCT_ORIGIN_BREAK;           /* 功能码 */
    cmd[3] =  smd_checksum(cmd, 3);       /* 校验和 */
    cmd[4] =  FRAME_TAIL;                 /* 帧尾 */

    /* 发送命令 */
    smd_send_data(cmd, 5);
}

/**
 * @brief    读取原点回零参数
 * @param    addr    :  电机地址
 * @retval   从机应答 :  帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_origin_read_params(uint8_t addr)
{
    uint8_t cmd[16] = {0};

    /* 装载命令 */
    cmd[0] =  FRAME_HEAD;                 /* 帧头 */
    cmd[1] =  addr;                       /* 地址 */
    cmd[2] =  FCT_ORIGIN_READ_PARAMS;     /* 功能码 */
    cmd[3] =  smd_checksum(cmd, 3);       /* 校验和 */
    cmd[4] =  FRAME_TAIL;                 /* 帧尾 */

    /* 发送命令 */
    smd_send_data(cmd, 5);
}

/**
 * @brief    修改原点回零超时时间
 * @param    addr      :  电机地址
 * @param    timout    :  找零点超时时间（单位ms）
 * @retval   从机应答  :  帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_origin_set_params(uint8_t addr, uint32_t timout)
{
    uint8_t cmd[16] = {0};

    /* 装载命令 */
    cmd[0] =  FRAME_HEAD;                 /* 帧头 */
    cmd[1] =  addr;                       /* 地址 */
    cmd[2] =  FCT_ORIGIN_SET_PARAMS;      /* 功能码 */
    cmd[3] = (uint8_t)((timout >> 24) & 0xFF);  
    cmd[4] = (uint8_t)((timout >> 16) & 0xFF);
    cmd[5] = (uint8_t)((timout >> 8) & 0xFF);
    cmd[6] = (uint8_t)((timout >> 0) & 0xFF); 
    cmd[7] =  smd_checksum(cmd, 7);       /* 校验和 */
    cmd[8] =  FRAME_TAIL;                 /* 帧尾 */

    /* 发送命令 */
    smd_send_data(cmd, 9);
}

/**
 * @brief    读取回零状态
 * @param    addr      :  电机地址
 * @retval   从机应答  :  帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_origin_read_sta(uint8_t addr)
{
    uint8_t cmd[16] = {0};

    /* 装载命令 */
    cmd[0] =  FRAME_HEAD;                 /* 帧头 */
    cmd[1] =  addr;                       /* 地址 */
    cmd[2] =  FCT_ORIGIN_READ_STA;        /* 功能码 */
    cmd[3] =  smd_checksum(cmd, 3);       /* 校验和 */
    cmd[4] =  FRAME_TAIL;                 /* 帧尾 */

    /* 发送命令 */
    smd_send_data(cmd, 5);
}

/**
 * @brief    设置上电是否需自动回零
 * @param    addr      :  电机地址
 * @param    flag      :  0 不自动回零 1 自动回零
 * @retval   从机应答  :  帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_origin_aoto_zero(uint8_t addr, uint8_t flag)
{
    uint8_t cmd[16] = {0};

    /* 装载命令 */
    cmd[0] =  FRAME_HEAD;                 /* 帧头 */
    cmd[1] =  addr;                       /* 地址 */
    cmd[2] =  FCT_ORIGIN_AOTO_ZERO;       /* 功能码 */
    cmd[3] =  flag;                       /* 设置是否需自动回零 */
    cmd[4] =  smd_checksum(cmd, 4);       /* 校验和 */
    cmd[5] =  FRAME_TAIL;                 /* 帧尾 */

    /* 发送命令 */
    smd_send_data(cmd, 6);
}

/**
 * @brief    设置右限位原点位置
 * @param    addr     :  电机地址
 * @param    pos      :  零点位置
 * @retval   从机应答  : 帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_origin_set_right_pos(uint8_t addr, int32_t pos)
{
    uint8_t cmd[16] = {0};

    /* 装载命令 */
    cmd[0] =  FRAME_HEAD;                   /* 帧头 */
    cmd[1] =  addr;                         /* 地址 */
    cmd[2] =  FCT_ORIGIN_SET_RIGHT_POS;     /* 功能码 */
    cmd[3] = (uint8_t)((pos >> 24) & 0xFF); /* 零点位置 */
    cmd[4] = (uint8_t)((pos >> 16) & 0xFF);
    cmd[5] = (uint8_t)((pos >> 8) & 0xFF);
    cmd[6] = (uint8_t)((pos >> 0) & 0xFF); 
    cmd[7] =  smd_checksum(cmd, 7);         /* 校验和 */
    cmd[8] =  FRAME_TAIL;                   /* 帧尾 */

    /* 发送命令 */
    smd_send_data(cmd, 9);
}

/**
 * @brief   设置左右限位开关状态（开启后电机运动范围受限左右限位原点位置）
 * @param    addr    :  电机地址
 * @param    level   :  0 关闭左右限位 1 开启左右限位
 * @retval   从机应答  : 帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_origin_l_r_switch(uint8_t addr, uint8_t ctrl)
{
    uint8_t cmd[16] = {0};

    /* 装载命令 */
    cmd[0] =  FRAME_HEAD;                 /* 帧头 */
    cmd[1] =  addr;                       /* 地址 */
    cmd[2] =  FCT_ORIGIN_SWITCH;          /* 功能码 */
    cmd[3] =  ctrl;                       /* 左右限位开关状态 */
    cmd[4] =  smd_checksum(cmd, 4);       /* 校验和 */
    cmd[5] =  FRAME_TAIL;                 /* 帧尾 */

    /* 发送命令 */
    smd_send_data(cmd, 6);
}

/**
 * @brief    力矩模式
 * @param    addr      :  电机地址
 * @param    dir       :  方向       ，0为CW(顺时针)，其余值为CCW(逆时针)
 * @param    current   :  电流       ，范围0 - 3000mA
 * @retval   从机应答   :  帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_torque_mode(uint8_t addr, uint8_t dir, uint16_t current)
{
    uint8_t cmd[16] = {0};

    /* 装载命令 */
    cmd[0] =  FRAME_HEAD;                 /* 帧头 */
    cmd[1] =  addr;                       /* 地址 */
    cmd[2] =  FCT_TORQUE_MODE;            /* 功能码 */
    cmd[3] =  dir;                        /* 方向 */
    cmd[4] = (uint8_t)((current >> 8) & 0xFF);  /* 电流高8位字节 */
    cmd[5] = (uint8_t)((current >> 0) & 0xFF);  /* 电流低8位字节 */ 
    cmd[6] =  smd_checksum(cmd, 6);       /* 校验和 */
    cmd[7] =  FRAME_TAIL;                 /* 帧尾 */

    /* 发送命令 */
    smd_send_data(cmd, 8);
}

/**
 * @brief    速度模式
 * @param    addr      :  电机地址
 * @param    dir       :  方向       ，0正转，1反转
 * @param    acc       :  加速度     ，范围0 - 200，单位RPM/SS 注意：0直接启动
 * @param    speed     :  速度       ，范围0.1 - 3000RPM
 * @retval   从机应答   :  帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_speed_mode(uint8_t addr, uint8_t dir, uint8_t acc, float speed)
{
    uint8_t cmd[16] = {0};

    /* 装载命令 */
    cmd[0] =  FRAME_HEAD;                 /* 帧头 */
    cmd[1] =  addr;                       /* 地址 */
    cmd[2] =  FCT_SPEED_MODE;             /* 功能码 */
    cmd[3] =  dir;                        /* 方向 */
    cmd[4] =  acc;                        /* 加速度，注意：0是直接启动 */
    data_u.f = speed;                     /* 速度(RPM) */
    cmd[5] =  data_u.b[3];                
    cmd[6] =  data_u.b[2];   
    cmd[7] =  data_u.b[1];    
    cmd[8] =  data_u.b[0];     
    cmd[9] =  smd_checksum(cmd, 9);       /* 校验和 */
    cmd[10] =  FRAME_TAIL;                /* 帧尾 */

    /* 发送命令 */
    smd_send_data(cmd, 11);
}

/**
 * @brief    绝对位置模式控制
 * @param    addr     :  电机地址
 * @param    dir      :  方向        ，0为CW(顺时针)，其余值为CCW(逆时针)
 * @param    acc      :  加速度      ，范围0 - 200，单位RPM/SS 注意：0直接启动
 * @param    speed    :  最大速度    ，范围0 - 3000RPM
 * @param    pulses   :  脉冲数      ，范围0- (2^32 - 1)个
 * @retval   从机应答  :  帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_pos_mode(uint8_t addr, uint8_t dir, uint8_t acc, uint16_t speed, uint32_t pulses)
{
    uint8_t cmd[16] = {0};

    /* 装载命令 */
    cmd[0] =  FRAME_HEAD;                   /* 帧头 */
    cmd[1] =  addr;                         /* 地址 */
    cmd[2] =  FCT_POS_MODE;                 /* 功能码 */
    cmd[3] =  dir;                          /* 方向 */
    cmd[4] =  acc;                          /* 加速度，注意：0是直接启动 */
    cmd[5] =  (uint8_t)((speed >> 8) & 0xFF);   /* 速度(RPM)高8位字节 */
    cmd[6] =  (uint8_t)((speed >> 0) & 0xFF);   /* 速度(RPM)低8位字节 */ 
    cmd[7] =  (uint8_t)((pulses >> 24) & 0xFF); /* 脉冲数(bit24 - bit31 ) */
    cmd[8] =  (uint8_t)((pulses >> 16) & 0xFF); /* 脉冲数(bit16 - bit23) */
    cmd[9] =  (uint8_t)((pulses >> 8) & 0xFF);  /* 脉冲数(bit8  - bit15) */
    cmd[10] = (uint8_t)((pulses >> 0) & 0xFF);  /* 脉冲数(bit0  - bit7) */
    cmd[11] = smd_checksum(cmd, 11);        /* 校验和 */
    cmd[12] = FRAME_TAIL;                   /* 帧尾 */

    /* 发送命令 */
    smd_send_data(cmd, 13);
}

/**
 * @brief    相对位置模式控制
 * @param    addr     :  电机地址
 * @param    dir      :  方向        ，0为CW(顺时针)，其余值为CCW(逆时针)
 * @param    acc      :  加速度      ，范围0 - 200，单位RPM/SS 注意：0直接启动
 * @param    speed    :  最大速度    ，范围0 - 3000RPM
 * @param    pulses   :  脉冲数      ，范围0- (2^32 - 1)个
 * @retval   从机应答  :  帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_pos_rel_mode(uint8_t addr, uint8_t dir, uint8_t acc, uint16_t speed, uint32_t pulses)
{
    uint8_t cmd[16] = {0};

    /* 装载命令 */
    cmd[0] =  FRAME_HEAD;                   /* 帧头 */
    cmd[1] =  addr;                         /* 地址 */
    cmd[2] =  FCT_POS_REL_MODE;                 /* 功能码 */
    cmd[3] =  dir;                          /* 方向 */
    cmd[4] =  acc;                          /* 加速度，注意：0是直接启动 */
    cmd[5] =  (uint8_t)((speed >> 8) & 0xFF);   /* 速度(RPM)高8位字节 */
    cmd[6] =  (uint8_t)((speed >> 0) & 0xFF);   /* 速度(RPM)低8位字节 */ 
    cmd[7] =  (uint8_t)((pulses >> 24) & 0xFF); /* 脉冲数(bit24 - bit31 ) */
    cmd[8] =  (uint8_t)((pulses >> 16) & 0xFF); /* 脉冲数(bit16 - bit23) */
    cmd[9] =  (uint8_t)((pulses >> 8) & 0xFF);  /* 脉冲数(bit8  - bit15) */
    cmd[10] = (uint8_t)((pulses >> 0) & 0xFF);  /* 脉冲数(bit0  - bit7) */
    cmd[11] = smd_checksum(cmd, 11);        /* 校验和 */
    cmd[12] = FRAME_TAIL;                   /* 帧尾 */

    /* 发送命令 */
    smd_send_data(cmd, 13);
}

/**
 * @brief    脉冲模式
 * @param    addr     :  电机地址
 * @retval   从机应答  :  帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_pulse_mode(uint8_t addr)
{
    uint8_t cmd[16] = {0};

    /* 装载命令 */
    cmd[0] =  FRAME_HEAD;                 /* 帧头 */
    cmd[1] =  addr;                       /* 地址 */
    cmd[2] =  FCT_PULSES_MODE;            /* 功能码 */
    cmd[3] =  smd_checksum(cmd, 3);       /* 校验和 */
    cmd[4] =  FRAME_TAIL;                 /* 帧尾 */

    /* 发送命令 */
    smd_send_data(cmd, 5);
}

/**
 * @brief    脉宽位置模式
 * @param    addr     :  电机地址
 * @param    topw_max :  高电平脉宽最长长度(0 < topw_max < 50000us)
 * @param    topw_min :  高电平脉宽最小长度(0 < topw_min < 50000us)
 * @param    top_pos :   脉宽最长长度对应的位置(51200为一圈)
 * @param    down_pos :  脉宽最小长度对应的位置
 * @retval   从机应答  :  帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_pulse_width_pos_mode(uint8_t addr, uint16_t topw_max, uint16_t topw_min, int32_t top_pos, int32_t down_pos)
{
    uint8_t cmd[32] = {0};

    /* 装载命令 */
    cmd[0] =  FRAME_HEAD;                 /* 帧头 */
    cmd[1] =  addr;                       /* 地址 */
    cmd[2] =  FCT_PULSE_WIDTH_POS_MODE;   /* 功能码 */
    cmd[3] = (uint8_t)((topw_max >> 8) & 0xFF); /* 高电平脉宽最长长度高8位字节 us */
    cmd[4] = (uint8_t)((topw_max >> 0) & 0xFF); /* 高电平脉宽最长长度低8位字节 */ 
    cmd[5] = (uint8_t)((topw_min >> 8) & 0xFF); /* 高电平脉宽最小长度高8位字节 us */
    cmd[6] = (uint8_t)((topw_min >> 0) & 0xFF); /* 高电平脉宽最小长度低8位字节 */ 
    /* 高电平脉宽最大长度对应的位置 */
    cmd[7] =  (uint8_t)((top_pos >> 24) & 0xFF);/* (bit24 - bit31 ) */
    cmd[8] =  (uint8_t)((top_pos >> 16) & 0xFF);/* (bit16 - bit23) */
    cmd[9] =  (uint8_t)((top_pos >> 8) & 0xFF); /* (bit8  - bit15) */
    cmd[10] = (uint8_t)((top_pos >> 0) & 0xFF); /* (bit0  - bit7) */
    /* 高电平脉宽最小长度对应的位置 */
    cmd[11] =  (uint8_t)((down_pos >> 24) & 0xFF);  /* (bit24 - bit31 ) */
    cmd[12] =  (uint8_t)((down_pos >> 16) & 0xFF);  /* (bit16 - bit23) */
    cmd[13] =  (uint8_t)((down_pos >> 8) & 0xFF);   /* (bit8  - bit15) */
    cmd[14] = (uint8_t)((down_pos >> 0) & 0xFF);    /* (bit0  - bit7) */
    cmd[15] =  smd_checksum(cmd, 15);       /* 校验和 */
    cmd[16] =  FRAME_TAIL;                 /* 帧尾 */

    /* 发送命令 */
    smd_send_data(cmd, 17);
}

/**
 * @brief    脉宽电流模式
 * @param    addr     :  电机地址
 * @param    topw_max :  高电平脉宽最长长度(0 < topw_max < 50000us)
 * @param    topw_min :  高电平脉宽最小长度(0 < topw_min < 50000us)
 * @param    top_pos :   脉宽最长长度对应的电流（单位mA）
 * @param    down_pos :  脉宽最小长度对应的电流
 * @retval   从机应答  :  帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_pulse_width_ma_mode(uint8_t addr, uint16_t topw_max, uint16_t topw_min, int32_t top_ma, int32_t down_ma)
{
    uint8_t cmd[32] = {0};
    
    /* 装载命令 */
    cmd[0] =  FRAME_HEAD;                 /* 帧头 */
    cmd[1] =  addr;                       /* 地址 */
    cmd[2] =  FCT_PULSE_WIDTH_MA_MODE;    /* 功能码 */
    cmd[3] = (uint8_t)((topw_max >> 8) & 0xFF); /* 高电平脉宽最长长度高8位字节 us */
    cmd[4] = (uint8_t)((topw_max >> 0) & 0xFF); /* 高电平脉宽最长长度低8位字节 */ 
    cmd[5] = (uint8_t)((topw_min >> 8) & 0xFF); /* 高电平脉宽最小长度高8位字节 us */
    cmd[6] = (uint8_t)((topw_min >> 0) & 0xFF); /* 高电平脉宽最小长度低8位字节 */ 
    /* 高电平脉宽最大长度对应的电流 */
    cmd[7] =  (uint8_t)((top_ma >> 24) & 0xFF);/* (bit24 - bit31 ) */
    cmd[8] =  (uint8_t)((top_ma >> 16) & 0xFF);/* (bit16 - bit23) */
    cmd[9] =  (uint8_t)((top_ma >> 8) & 0xFF); /* (bit8  - bit15) */
    cmd[10] = (uint8_t)((top_ma >> 0) & 0xFF); /* (bit0  - bit7) */
    /* 高电平脉宽最小长度对应的电流 */
    cmd[11] =  (uint8_t)((down_ma >> 24) & 0xFF);  /* (bit24 - bit31 ) */
    cmd[12] =  (uint8_t)((down_ma >> 16) & 0xFF);  /* (bit16 - bit23) */
    cmd[13] =  (uint8_t)((down_ma >> 8) & 0xFF);   /* (bit8  - bit15) */
    cmd[14] = (uint8_t)((down_ma >> 0) & 0xFF);    /* (bit0  - bit7) */
    cmd[15] =  smd_checksum(cmd, 15);       /* 校验和 */
    cmd[16] =  FRAME_TAIL;                  /* 帧尾 */

    /* 发送命令 */
    smd_send_data(cmd, 17);
}

/**
 * @brief    脉宽速度模式
 * @param    addr     :  电机地址
 * @retval   从机应答  :  帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_pulse_width_speed_mode(uint8_t addr, uint16_t topw_max, uint16_t topw_min, int32_t top_speed, int32_t down_speed)
{
    uint8_t cmd[32] = {0};
    
    /* 装载命令 */
    cmd[0] =  FRAME_HEAD;                 /* 帧头 */
    cmd[1] =  addr;                       /* 地址 */
    cmd[2] =  FCT_PULSE_WIDTH_SPEED_MODE; /* 功能码 */
    cmd[3] = (uint8_t)((topw_max >> 8) & 0xFF); /* 高电平脉宽最长长度高8位字节 us */
    cmd[4] = (uint8_t)((topw_max >> 0) & 0xFF); /* 高电平脉宽最长长度低8位字节 */ 
    cmd[5] = (uint8_t)((topw_min >> 8) & 0xFF); /* 高电平脉宽最小长度高8位字节 us */
    cmd[6] = (uint8_t)((topw_min >> 0) & 0xFF); /* 高电平脉宽最小长度低8位字节 */ 
    /* 高电平脉宽最大长度对应的速度 */
    cmd[7] =  (uint8_t)((top_speed >> 24) & 0xFF);/* (bit24 - bit31 ) */
    cmd[8] =  (uint8_t)((top_speed >> 16) & 0xFF);/* (bit16 - bit23) */
    cmd[9] =  (uint8_t)((top_speed >> 8) & 0xFF); /* (bit8  - bit15) */
    cmd[10] = (uint8_t)((top_speed >> 0) & 0xFF); /* (bit0  - bit7) */
    /* 高电平脉宽最小长度对应的速度 */
    cmd[11] =  (uint8_t)((down_speed >> 24) & 0xFF);  /* (bit24 - bit31 ) */
    cmd[12] =  (uint8_t)((down_speed >> 16) & 0xFF);  /* (bit16 - bit23) */
    cmd[13] =  (uint8_t)((down_speed >> 8) & 0xFF);   /* (bit8  - bit15) */
    cmd[14] = (uint8_t)((down_speed >> 0) & 0xFF);    /* (bit0  - bit7) */
    cmd[15] =  smd_checksum(cmd, 15);      /* 校验和 */
    cmd[16] =  FRAME_TAIL;                 /* 帧尾 */

    /* 发送命令 */
    smd_send_data(cmd, 17);
}


/**
 * @brief    开环速度模式
 * @param    addr      :  电机地址
 * @param    dir       :  方向       ，0正转，1反转
 * @param    acc       :  加速度     ，范围0 - 200，单位RPM/SS 注意：0直接启动
 * @param    speed     :  速度       ，范围0.1 - 3000RPM
 * @retval   从机应答   :  帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_ol_speed_mode(uint8_t addr, uint8_t dir, uint8_t acc, float speed)
{
    uint8_t cmd[16] = {0};

    /* 装载命令 */
    cmd[0] =  FRAME_HEAD;                 /* 帧头 */
    cmd[1] =  addr;                       /* 地址 */
    cmd[2] =  FCT_OL_SPEED_MODE;          /* 功能码 */
    cmd[3] =  dir;                        /* 方向 */
    cmd[4] =  acc;                        /* 加速度，注意：0是直接启动 */
    data_u.f = speed;                     /* 速度(RPM) */
    cmd[5] =  data_u.b[3];                
    cmd[6] =  data_u.b[2];   
    cmd[7] =  data_u.b[1];    
    cmd[8] =  data_u.b[0];     
    cmd[9] =  smd_checksum(cmd, 9);       /* 校验和 */
    cmd[10] =  FRAME_TAIL;                /* 帧尾 */

    /* 发送命令 */
    smd_send_data(cmd, 11);
}

/**
 * @brief    开环绝对位置模式控制
 * @param    addr     :  电机地址
 * @param    dir      :  方向        ，0为CW(顺时针)，其余值为CCW(逆时针)
 * @param    acc      :  加速度      ，范围0 - 200，单位RPM/SS 注意：0直接启动
 * @param    speed    :  最大速度    ，范围0 - 3000RPM
 * @param    pulses   :  脉冲数      ，范围0- (2^32 - 1)个
 * @retval   从机应答  :  帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_ol_pos_mode(uint8_t addr, uint8_t dir, uint8_t acc, uint16_t speed, uint32_t pulses)
{
    uint8_t cmd[16] = {0};

    /* 装载命令 */
    cmd[0] =  FRAME_HEAD;                   /* 帧头 */
    cmd[1] =  addr;                         /* 地址 */
    cmd[2] =  FCT_OL_POS_MODE;              /* 功能码 */
    cmd[3] =  dir;                          /* 方向 */
    cmd[4] =  acc;                          /* 加速度，注意：0是直接启动 */
    cmd[5] =  (uint8_t)((speed >> 8) & 0xFF);   /* 速度(RPM)高8位字节 */
    cmd[6] =  (uint8_t)((speed >> 0) & 0xFF);   /* 速度(RPM)低8位字节 */ 
    cmd[7] =  (uint8_t)((pulses >> 24) & 0xFF); /* 脉冲数(bit24 - bit31 ) */
    cmd[8] =  (uint8_t)((pulses >> 16) & 0xFF); /* 脉冲数(bit16 - bit23) */
    cmd[9] =  (uint8_t)((pulses >> 8) & 0xFF);  /* 脉冲数(bit8  - bit15) */
    cmd[10] = (uint8_t)((pulses >> 0) & 0xFF);  /* 脉冲数(bit0  - bit7) */
    cmd[11] = smd_checksum(cmd, 11);        /* 校验和 */
    cmd[12] = FRAME_TAIL;                   /* 帧尾 */

    /* 发送命令 */
    smd_send_data(cmd, 13);
}

/**
 * @brief    开环相对位置模式控制
 * @param    addr     :  电机地址
 * @param    dir      :  方向        ，0为CW(顺时针)，其余值为CCW(逆时针)
 * @param    acc      :  加速度      ，范围0 - 200，单位RPM/SS 注意：0直接启动
 * @param    speed    :  最大速度    ，范围0 - 3000RPM
 * @param    pulses   :  脉冲数      ，范围0- (2^32 - 1)个
 * @retval   从机应答  :  帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_ol_pos_rel_mode(uint8_t addr, uint8_t dir, uint8_t acc, uint16_t speed, uint32_t pulses)
{
    uint8_t cmd[16] = {0};

    /* 装载命令 */
    cmd[0] =  FRAME_HEAD;                   /* 帧头 */
    cmd[1] =  addr;                         /* 地址 */
    cmd[2] =  FCT_OL_POS_REL_MODE;          /* 功能码 */
    cmd[3] =  dir;                          /* 方向 */
    cmd[4] =  acc;                          /* 加速度，注意：0是直接启动 */
    cmd[5] =  (uint8_t)((speed >> 8) & 0xFF);   /* 速度(RPM)高8位字节 */
    cmd[6] =  (uint8_t)((speed >> 0) & 0xFF);   /* 速度(RPM)低8位字节 */ 
    cmd[7] =  (uint8_t)((pulses >> 24) & 0xFF); /* 脉冲数(bit24 - bit31 ) */
    cmd[8] =  (uint8_t)((pulses >> 16) & 0xFF); /* 脉冲数(bit16 - bit23) */
    cmd[9] =  (uint8_t)((pulses >> 8) & 0xFF);  /* 脉冲数(bit8  - bit15) */
    cmd[10] = (uint8_t)((pulses >> 0) & 0xFF);  /* 脉冲数(bit0  - bit7) */
    cmd[11] = smd_checksum(cmd, 11);        /* 校验和 */
    cmd[12] = FRAME_TAIL;                   /* 帧尾 */

    /* 发送命令 */
    smd_send_data(cmd, 13);
}

/**
 * @brief    开环脉冲模式
 * @param    addr     :  电机地址
 * @retval   从机应答  :  帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_ol_pulse_mode(uint8_t addr)
{
    uint8_t cmd[16] = {0};

    /* 装载命令 */
    cmd[0] =  FRAME_HEAD;                 /* 帧头 */
    cmd[1] =  addr;                       /* 地址 */
    cmd[2] =  FCT_OL_PULSES_MODE;         /* 功能码 */
    cmd[3] =  smd_checksum(cmd, 3);       /* 校验和 */
    cmd[4] =  FRAME_TAIL;                 /* 帧尾 */

    /* 发送命令 */
    smd_send_data(cmd, 5);
}

/**
 * @brief    IO启停模式
 * @param    addr      :  电机地址
 * @param    dir       :  方向       ，0正转，1反转
 * @param    acc       :  加速度     ，范围0 - 200，单位RPM/SS 注意：0直接启动
 * @param    speed     :  速度       ，范围0.1 - 3000RPM
 * @retval   从机应答   :  帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_io_run_ctrl(uint8_t addr, uint8_t dir, uint8_t acc, float speed)
{
    uint8_t cmd[16] = {0};

    /* 装载命令 */
    cmd[0] =  FRAME_HEAD;                 /* 帧头 */
    cmd[1] =  addr;                       /* 地址 */
    cmd[2] =  FCT_IO_RUN_MODE;            /* 功能码 */
    cmd[3] =  dir;                        /* 方向 */
    cmd[4] =  acc;                        /* 加速度，注意：0是直接启动 */
    data_u.f = speed;                     /* 速度(RPM) */
    cmd[5] =  data_u.b[3];                
    cmd[6] =  data_u.b[2];   
    cmd[7] =  data_u.b[1];    
    cmd[8] =  data_u.b[0];     
    cmd[9] =  smd_checksum(cmd, 9);       /* 校验和 */
    cmd[10] =  FRAME_TAIL;                /* 帧尾 */

    /* 发送命令 */
    smd_send_data(cmd, 11);
}


/** 
 * @brief    将当前位置角度清零
 * @param    addr    :  电机地址
 * @retval   从机应答  : 帧头 + 地址 + 功能码 + 命令状态 + 校验字节 + 帧尾
 */
void smd_angle_to_zero(uint8_t addr)
{
    uint8_t cmd[16] = {0};

    /* 装载命令 */
    cmd[0] =  FRAME_HEAD;                 /* 帧头 */
    cmd[1] =  addr;                       /* 地址 */
    cmd[2] =  FCT_ANGLE_ZERO;             /* 功能码 */
    cmd[3] =  smd_checksum(cmd, 3);       /* 校验和 */
    cmd[4] =  FRAME_TAIL;                 /* 帧尾 */

    /* 发送命令 */
    smd_send_data(cmd, 5);
}

/**
 * @brief    解除堵转保护
 * @param    addr     : 电机地址
 * @retval   从机应答  : 帧头 + 地址 + 功能码 + 命令状态 + 校验字节 + 帧尾
 */
void smd_remove_clog_protect(uint8_t addr)
{
    uint8_t cmd[16] = {0};

    /* 装载命令 */
    cmd[0] =  FRAME_HEAD;                 /* 帧头 */
    cmd[1] =  addr;                       /* 地址 */
    cmd[2] =  FCT_CLEAR_CLOG_PRO;         /* 功能码 */
    cmd[3] =  smd_checksum(cmd, 3);       /* 校验和 */
    cmd[4] =  FRAME_TAIL;                 /* 帧尾 */

    /* 发送命令 */
    smd_send_data(cmd, 5);
}


/**
 * @brief    电机使能控制
 * @param    addr     :  电机地址
 * @param    en       :  0 使能电机， 1 失能
 * @retval   从机应答 :  帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_motor_enable(uint8_t addr, uint8_t en)
{
    uint8_t cmd[16] = {0};

    /* 装载命令 */
    cmd[0] =  FRAME_HEAD;                 /* 帧头 */
    cmd[1] =  addr;                       /* 地址 */
    cmd[2] =  FCT_MOTOR_ENABLE;           /* 功能码 */
    cmd[3] =  en;                         /* 使能模式 */
    cmd[4] =  smd_checksum(cmd, 4);       /* 校验和 */
    cmd[5] =  FRAME_TAIL;                 /* 帧尾 */

    /* 发送命令 */
    smd_send_data(cmd, 6);
}

/**
 * @brief    清除电机状态（堵转、失能、刹车）
 * @param    addr     :  电机地址
 * @retval   从机应答  :  帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_clear_sta(uint8_t addr)
{
    uint8_t cmd[16] = {0};

    /* 装载命令 */
    cmd[0] =  FRAME_HEAD;                 /* 帧头 */
    cmd[1] =  addr;                       /* 地址 */
    cmd[2] =  FCT_CLEAR_STATE;            /* 功能码 */
    cmd[3] =  smd_checksum(cmd, 3);       /* 校验和 */
    cmd[4] =  FRAME_TAIL;                 /* 帧尾 */

    /* 发送命令 */
    smd_send_data(cmd, 5);
}

/**
 * @brief    立即停止
 * @param    addr     :  电机地址
 * @retval   从机应答  :  帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_stop_now(uint8_t addr)
{
    uint8_t cmd[16] = {0};

    /* 装载命令 */
    cmd[0] =  FRAME_HEAD;                 /* 帧头 */
    cmd[1] =  addr;                       /* 地址 */
    cmd[2] =  FCT_STOP_NOW;               /* 功能码 */
    cmd[3] =  smd_checksum(cmd, 3);       /* 校验和 */
    cmd[4] =  FRAME_TAIL;                 /* 帧尾 */

    /* 发送命令 */
    smd_send_data(cmd, 5);
}

/**
 * @} smd_send
 */

