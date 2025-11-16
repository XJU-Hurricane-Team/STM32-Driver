/**
 * @file    dji_bldc_motor.c
 * @author  Deadline039
 * @brief   M3508, M2006 直流无刷电机驱动
 * @version 1.5
 * @date    2024-03-02
 */

#include "dji_bldc_motor.h"

#include "can_list/can_list.h"

/**
 * @brief CAN 收到消息中断回调
 *
 * @param node_obj 节点数据
 * @param can_rx_header CAN 消息头
 * @param can_msg CAN 消息
 */
static void can_callback(void *node_obj, can_rx_header_t *can_rx_header,
                         uint8_t *can_msg) {
    if (node_obj == NULL) {
        return;
    }

    dji_motor_handle_t *motor_point = (dji_motor_handle_t *)node_obj;

    if (can_rx_header->id_type != CAN_ID_STD &&
        can_rx_header->id != motor_point->motor_id) {
        return;
    }

    motor_point->last_angle = motor_point->angle;
    motor_point->angle = (uint16_t)((can_msg[0] << 8) | can_msg[1]);

    if (!(motor_point->got_offset)) {
        /* 获取上电电机初始角度 */
        motor_point->offset_angle = motor_point->angle;
        motor_point->last_angle = motor_point->angle;
        motor_point->got_offset = true;
        motor_point->round_cnt = 0;
    }

    switch (motor_point->motor_model) {
#if (DJI_MOTOR_USE_M3508_2006 == 1)
        case DJI_M3508: {
            motor_point->real_current = (float)(can_msg[2] << 8 | can_msg[3]);
            motor_point->speed_rpm = (int16_t)(motor_point->real_current);
            motor_point->given_current =
                (int16_t)((float)(can_msg[4] << 8 | can_msg[5]) / -5.0f);
        } break;

        case DJI_M2006: {
            motor_point->speed_rpm = (int16_t)(can_msg[2] << 8 | can_msg[3]);
            motor_point->real_current =
                (float)(can_msg[4] << 8 | can_msg[5]) * 5.0f / 16384.0f;
        } break;
#endif /* DJI_MOTOR_USE_M3508_2006 == 1 */

#if (DJI_MOTOR_USE_GM6020 == 1)
        case DJI_GM6020: {
            motor_point->speed_rpm = (int16_t)(can_msg[2] << 8 | can_msg[3]);
            motor_point->torque_current =
                (int16_t)(can_msg[4] << 8 | can_msg[5]);
            motor_point->temperature = can_msg[6];
        } break;
#endif /* DJI_MOTOR_USE_GM6020 == 1 */

        default: {
        } break;
    }

    motor_point->hall = can_msg[6];

    if (motor_point->angle - motor_point->last_angle > 4096) {
        --(motor_point->round_cnt);
    } else if (motor_point->angle - motor_point->last_angle < -4096) {
        ++(motor_point->round_cnt);
    }

    motor_point->total_angle = motor_point->round_cnt * 4096 * 2 +
                               motor_point->angle - motor_point->offset_angle;

    /**
     * 对于 3508 与 2006, 是轴的相对位置 (上电后为 0
     * 度，角度会累加，已经除过减速比); rotor_degree = 总角度 (total_angle) /
     * (减速比 * 8192) * 360 对于 6020, 是绝对位置 (0 ~ 360) 6020 的减速比为 1,
     * 8192 就是 360 度，所以 rotor_degree = 当前角度 (angle) / 8192 / 360 =
     * angle / 22.75
     */
    switch (motor_point->motor_model) {
#if (DJI_MOTOR_USE_M3508_2006 == 1)
        case DJI_M3508: {
            /* 3508 减速比 1:19 */
            motor_point->rotor_degree =
                (float)(motor_point->total_angle) / (19.0f * 8192.0f) * 360.0f;
        } break;

        case DJI_M2006: {
            /* 2006 减速比 1:36 */
            motor_point->rotor_degree =
                (float)(motor_point->total_angle) / (36.0f * 8192.0f) * 360.0f;
        } break;
#endif /* DJI_MOTOR_USE_M3508_2006 == 1 */

#if (DJI_MOTOR_USE_GM6020 == 1)
        case DJI_GM6020: {
            motor_point->rotor_degree = (float)(motor_point->angle) / 22.75f;
        } break;
#endif /* DJI_MOTOR_USE_GM6020 == 1 */

        default: {
        } break;
    }
}

/**
 * @brief 初始化电机
 *
 * @param motor 电机结构体指针
 * @param motor_model 电机型号 `DJI_M3508 或 DJI_M2006`, 关系到减速比与角度
 * @param can_id CAN ID
 * @param can_select 选择哪一个 CAN 来通信
 * @return 初始化状态:
 * @retval - 0: 成功
 * @retval - 1: `motor`指针为空
 * @retval - 2: 添加 CAN 接收表错误
 */
uint8_t dji_motor_init(dji_motor_handle_t *motor, dji_motor_model_t motor_model,
                       dji_can_id_t can_id, can_selected_t can_select) {
    if (motor == NULL) {
        return 1;
    }

    motor->motor_model = motor_model;
    motor->got_offset = false;
    motor->can_select = can_select;
    if (can_list_add_new_node(can_select, (void *)motor, can_id, 0x7FF,
                              CAN_ID_STD, can_callback) != 0) {
        return 2;
    }

    return 0;
}

/**
 * @brief 反初始化电机
 *
 * @param motor 电机结构体指针
 * @return 反初始化状态:
 * @retval - 0: 成功
 * @return - 1: `motor`为空
 * @retval - 2: 移除出错
 */
uint8_t dji_motor_deinit(dji_motor_handle_t *motor) {
    if (motor == NULL) {
        return 1;
    }

    if (can_list_del_node_by_id(motor->can_select, CAN_ID_STD,
                                motor->motor_id) != 0) {
        return 2;
    }

    return 0;
}

#if (DJI_MOTOR_USE_M3508_2006 == 1)

/**
 * @brief 设置 M3508/2006 电机电流
 *
 * @param can_select 选择那个 CAN 发送
 *  @arg `can1_selected` 或者 `can2_selected`
 * @param can_identify CAN 标识符
 *  @arg `DJI_MOTOR_GROUP1` 或 `DJI_MOTOR_GROUP2`
 * @param iq1 电机 1 电流
 * @param iq2 电机 2 电流
 * @param iq3 电机 3 电流
 * @param iq4 电机 4 电流
 */
void dji_motor_set_current(can_selected_t can_select, uint16_t can_identify,
                           int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4) {
    if (can_identify != DJI_MOTOR_GROUP1 && can_identify != DJI_MOTOR_GROUP2) {
        /* 标识符不合法 */
        return;
    }

    uint8_t send_msg[8];
    send_msg[0] = (iq1 >> 8) & 0xFF;
    send_msg[1] = iq1 & 0xFF;
    send_msg[2] = (iq2 >> 8) & 0xFF;
    send_msg[3] = iq2 & 0xFF;
    send_msg[4] = (iq3 >> 8) & 0xFF;
    send_msg[5] = iq3 & 0xFF;
    send_msg[6] = (iq4 >> 8) & 0xFF;
    send_msg[7] = iq4 & 0xFF;
    can_send_message(can_select, CAN_ID_STD, can_identify, 8, send_msg);
}

#endif /* DJI_MOTOR_USE_M3508_2006 == 1 */

#if (DJI_MOTOR_USE_GM6020 == 1)

/**
 * @brief GM6020 电压控制
 *
 * @param can_select 选择那个 CAN 发送
 *  @arg `can1_selected` 或者 `can2_selected`
 * @param can_identify CAN 标识符
 *  @arg `DJI_GM6020_VOLTAGE_GROUP1` 或 `DJI_GM6020_VOLTAGE_GROUP2`
 * @param voltage1 电机 1 电压
 * @param voltage2 电机 2 电压
 * @param voltage3 电机 3 电压
 * @param voltage4 电机 4 电压
 */
void dji_gm6020_voltage_control(can_selected_t can_select,
                                uint16_t can_identify, int16_t voltage1,
                                int16_t voltage2, int16_t voltage3,
                                int16_t voltage4) {
    if (can_identify != DJI_GM6020_VOLTAGE_GROUP1 &&
        can_identify != DJI_GM6020_VOLTAGE_GROUP2) {
        /* 标识符不合法 */
        return;
    }

    uint8_t send_msg[8];
    send_msg[0] = (voltage1 >> 8) & 0xFF;
    send_msg[1] = voltage1 & 0xFF;
    send_msg[2] = (voltage2 >> 8) & 0xFF;
    send_msg[3] = voltage2 & 0xFF;
    send_msg[4] = (voltage3 >> 8) & 0xFF;
    send_msg[5] = voltage3 & 0xFF;
    send_msg[6] = (voltage4 >> 8) & 0xFF;
    send_msg[7] = voltage4 & 0xFF;
    can_send_message(can_select, CAN_ID_STD, can_identify, 8, send_msg);
}

/**
 * @brief GM6020 电流控制
 *
 * @param can_select 选择那个 CAN 发送
 *  @arg `can1_selected` 或者 `can2_selected`
 * @param can_identify CAN 标识符
 *  @arg `DJI_GM6020_CURRENT_GROUP1` 或 `DJI_GM6020_CURRENT_GROUP2`
 * @param current1 电机 1 电流
 * @param current2 电机 2 电流
 * @param current3 电机 3 电流
 * @param current4 电机 4 电流
 */
void dji_gm6020_current_control(can_selected_t can_select,
                                uint16_t can_identify, int16_t current1,
                                int16_t current2, int16_t current3,
                                int16_t current4) {
    if (can_identify != DJI_GM6020_CURRENT_GROUP1 &&
        can_identify != DJI_GM6020_CURRENT_GROUP2) {
        /* 标识符不合法 */
        return;
    }

    uint8_t send_msg[8];
    send_msg[0] = (current1 >> 8) & 0xFF;
    send_msg[1] = current1 & 0xFF;
    send_msg[2] = (current2 >> 8) & 0xFF;
    send_msg[3] = current2 & 0xFF;
    send_msg[4] = (current3 >> 8) & 0xFF;
    send_msg[5] = current3 & 0xFF;
    send_msg[6] = (current4 >> 8) & 0xFF;
    send_msg[7] = current4 & 0xFF;

    can_send_message(can_select, CAN_ID_STD, can_identify, 8, send_msg);
}

#endif /* DJI_MOTOR_USE_GM6020 == 1 */
