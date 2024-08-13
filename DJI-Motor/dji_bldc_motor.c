/**
 * @file    dji_bldc_motor.c
 * @author  Deadline039
 * @brief   M3508, M2006直流无刷电机驱动
 * @version 1.3
 * @date    2024-03-02
 * @note    支持两个CAN通信, 两个CAN可以设置ID一致的电机, 完全独立不影响
 */

#include "dji_bldc_motor.h"

/**
 * @brief 电机列表, 用于CAN中断回调设置电机参数, CAN1, CAN2各一个独立的列表
 */
static struct {
    dji_motor_handle_t *motor_point; /*!< 电机参数 */
} dji_motor_list[2][11];

/**
 * @brief 初始化电机
 *
 * @param motor 电机结构体指针
 * @param motor_model 电机型号`DJI_M3508或DJI_M2006`, 关系到减速比与角度
 * @param can_id CAN ID
 * @param can_selected 选择哪一个CAN来通信
 */
void dji_motor_init(dji_motor_handle_t *motor, dji_motor_model_t motor_model,
                    dji_can_id_t can_id, can_select_t can_selected) {
    dji_motor_list[can_selected][can_id - 0x201].motor_point = motor;

    motor->motor_model = motor_model;
    motor->got_offset = false;
}

/**
 * @brief 反初始化电机
 *
 * @param motor 电机结构体指针
 * @param can_selected CAN1还是CAN2
 */
void dji_motor_deinit(dji_motor_handle_t *motor, can_select_t can_selected) {
    dji_motor_list[can_selected][motor->motor_id - 0x201].motor_point = NULL;
}

/**
 * @brief CAN收到消息中断回调
 *
 * @param can_selected 哪一个CAN的消息
 * @param can_id CAN ID
 * @param recv_msg CAN消息
 */
void dji_motor_can_recv_callback(can_select_t can_selected, uint32_t can_id,
                                 uint8_t *recv_msg) {

    dji_motor_handle_t *motor_point;

    motor_point = dji_motor_list[can_selected][can_id - 0x201].motor_point;

    if (motor_point == NULL) {
        return;
    }

    motor_point->last_angle = motor_point->angle;
    motor_point->angle = (uint16_t)((recv_msg[0] << 8) | recv_msg[1]);

    if (!(motor_point->got_offset)) {
        /* 获取上电电机初始角度 */
        motor_point->offset_angle = motor_point->angle;
        motor_point->last_angle = motor_point->angle;
        motor_point->got_offset = true;
        motor_point->round_cnt = 0;
    }

    switch (motor_point->motor_model) {
        case DJI_M3508: {
            motor_point->real_current = (float)(recv_msg[2] << 8 | recv_msg[3]);
            motor_point->speed_rpm = (int16_t)(motor_point->real_current);
            motor_point->given_current =
                (int16_t)((float)(recv_msg[4] << 8 | recv_msg[5]) / -5.0f);
        } break;

        case DJI_M2006: {
            motor_point->speed_rpm = (int16_t)(recv_msg[2] << 8 | recv_msg[3]);
            motor_point->real_current =
                (float)(recv_msg[4] << 8 | recv_msg[5]) * 5.0f / 16384.0f;
        } break;

        case DJI_GM6020: {
            motor_point->speed_rpm = (int16_t)(recv_msg[2] << 8 | recv_msg[3]);
            motor_point->torque_current =
                (int16_t)(recv_msg[4] << 8 | recv_msg[5]);
            motor_point->temperature = recv_msg[6];
        } break;

        default: {
        } break;
    }

    motor_point->hall = recv_msg[6];

    if (motor_point->angle - motor_point->last_angle > 4096) {
        --(motor_point->round_cnt);
    } else if (motor_point->angle - motor_point->last_angle < -4096) {
        ++(motor_point->round_cnt);
    }

    motor_point->total_angle = motor_point->round_cnt * 4096 * 2 +
                               motor_point->angle - motor_point->offset_angle;

    /**
     * 对于3508与2006, 是轴的相对位置(上电后为0度, 角度会累加, 已经除过减速比);
     *      rotor_degree = 总角度(total_angle) / (减速比 * 8192) * 360
     * 对于6020, 是绝对位置(0 ~ 360)
     * 6020的减速比为1, 8192就是360度, 所以
     *      rotor_degree = 当前角度(angle) / 8192 / 360 = angle / 22.75
     */
    switch (motor_point->motor_model) {
        case DJI_M3508: {
            /* 3508减速比1:19 */
            motor_point->rotor_degree =
                (float)(motor_point->total_angle) / (19.0f * 8192.0f) * 360.0f;
        } break;

        case DJI_M2006: {
            /* 2006减速比1:36 */
            motor_point->rotor_degree =
                (float)(motor_point->total_angle) / (36.0f * 8192.0f) * 360.0f;
        } break;

        case DJI_GM6020: {
            motor_point->rotor_degree = (float)(motor_point->angle) / 22.75f;
        } break;

        default: {
        } break;
    }
}

/**
 * @brief 设置M3508/2006电机电流
 *
 * @param can_selected 选择那个CAN发送
 *  @arg `can1_selected`或者`can2_selected`
 * @param can_identify CAN标识符
 *  @arg `DJI_MOTOR_GROUP1`或`DJI_MOTOR_GROUP2`
 * @param iq1 电机1电流
 * @param iq2 电机2电流
 * @param iq3 电机3电流
 * @param iq4 电机4电流

 */
void dji_motor_set_current(can_select_t can_select, uint16_t can_identify,
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

/**
 * @brief GM6020电压控制
 *
 * @param can_selected 选择那个CAN发送
 *  @arg `can1_selected`或者`can2_selected`
 * @param can_identify CAN标识符
 *  @arg `DJI_GM6020_VOLTAGE_GROUP1`或`DJI_GM6020_VOLTAGE_GROUP2`
 * @param voltage1 电机1电压
 * @param voltage2 电机2电压
 * @param voltage3 电机3电压
 * @param voltage4 电机4电压
 */
void dji_gm6020_set_voltage(can_select_t can_select, uint16_t can_identify,
                            int16_t voltage1, int16_t voltage2,
                            int16_t voltage3, int16_t voltage4) {
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
 * @brief GM6020电流控制
 *
 * @param can_selected 选择那个CAN发送
 *  @arg `can1_selected`或者`can2_selected`
 * @param can_identify CAN标识符
 *  @arg `DJI_GM6020_CURRENT_GROUP1`或`DJI_GM6020_CURRENT_GROUP2`
 * @param current1 电机1电流
 * @param current2 电机2电流
 * @param current3 电机3电流
 * @param current4 电机4电流
 */
void dji_gm6020_set_current(can_select_t can_select, uint16_t can_identify,
                            int16_t current1, int16_t current2,
                            int16_t current3, int16_t current4) {
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
