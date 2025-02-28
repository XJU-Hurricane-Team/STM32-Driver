/**
 * @file    mlds_driver.c
 * @author  Deadline039
 * @brief   MLDS3605-C驱动器
 * @version 0.2
 * @date    2024-05-03
 */

#include "mlds_driver.h"
#include "CSP_Config.h"
#include "string.h"

/* 给驱动器发送的CAN数据 */
static uint8_t can_data[8];

void mlds_motor_init(mlds_motor_handle_t motor, uint8_t id,
                     can_selected_t can_selected) {
    motor.id = id;
    motor.can_selected = can_selected;
}

/**
 * @brief 设置驱动器工作模式
 *
 * @param mode 工作模式, 设置信号源与设置工作模式按位或.
 *             例如设置数字指令速度控制: `MLDS_SIGNAL_DIGITAL | MLDS_MODE_SPEED`
 */
void mlds_set_mode(mlds_motor_handle_t motor, uint32_t mode) {
    can_data[0] = 8;
    can_data[1] = motor.id;
    can_data[2] = 0x2a;
    can_data[3] = 0x00;
    memcpy(&can_data[4], &mode, 4);
    can_send_message(motor.can_selected, CAN_ID_STD, motor.id, 8, can_data);
}

/**
 * @brief 设置伺服, 以一定速度旋转
 *
 * @param speed 速度
 */
void mlds_run_speed(mlds_motor_handle_t motor,int32_t speed) {
    can_data[0] = 8;
  can_data[1] = motor.id;
    can_data[2] = 0x90;
    can_data[3] = 0x00;
    memcpy(&can_data[4], &speed, 4);
    can_send_message(motor.can_selected, CAN_ID_STD, motor.id, 8,can_data);
}

/**
 * @brief 设置电机以当前位置位绝对零点
 *
 */
void mlds_set_absolute_origin(mlds_motor_handle_t motor) {
    can_data[0] = 4;
  can_data[1] = motor.id;
    can_data[2] = 0x98;
    can_data[3] = 0x00;
    memset(&can_data[4], 0, 4);
        can_send_message(motor.can_selected, CAN_ID_STD, motor.id, 8,can_data);
}

/**
 * @brief 设置电机以绝对零点为零点, 转到设定的角度
 *
 * @param angle 要转的角度, -360~360°
 */
void mlds_run_absolute_angle(mlds_motor_handle_t motor,float angle) {
    can_data[0] = 8;
  can_data[1] = motor.id;
    int32_t angle_int = (int32_t)(angle * 117.76f);
    can_data[2] = 0x99;
    can_data[3] = 0x00;
    memcpy(&can_data[4], &angle_int, 4);
        can_send_message(motor.can_selected, CAN_ID_STD, motor.id, 8,can_data);
}

/**
 * @brief 设置电机以当前位置为零点, 转到设定的角度
 *
 * @param angle 要转的角度, -360~360°
 */
void mlds_run_relative_angle(mlds_motor_handle_t motor,float angle) {
    can_data[0] = 8;
  can_data[1] = motor.id;
    int32_t angle_int = (int32_t)(angle * 117.76f);
    can_data[2] = 0x9a;
    can_data[3] = 0x00;
    memcpy(&can_data[4], &angle_int, 4);
        can_send_message(motor.can_selected, CAN_ID_STD, motor.id, 8,can_data);
}

/**
 * @brief 设置速度pid
 *
 * @param set_item 设置p, i, d
 * @param set_value 设置的值
 */
void mlds_set_speed_pid(mlds_motor_handle_t motor,mlds_set_pid_t set_item, int16_t set_value) {
    can_data[0] = 8;
  can_data[1] = motor.id;
    switch (set_item) {
        case MLDS_SET_KP: {
            can_data[2] = 0x60;
        } break;

        case MLDS_SET_KI: {
            can_data[2] = 0x62;
        } break;

        case MLDS_SET_KD: {
            can_data[2] = 0x64;
        } break;

        default: {
        } break;
    }
    can_data[3] = 0x00;
    memcpy(&can_data[4], &set_value, 2);
    can_data[6] = 0x00;
    can_data[7] = 0x00;
        can_send_message(motor.can_selected, CAN_ID_STD, motor.id, 8,can_data);
}

/**
 * @brief 设置角度pid
 *
 * @param set_item 设置p, i, d
 * @param set_value 设置的值
 */
void mlds_set_angle_pid(mlds_motor_handle_t motor,mlds_set_pid_t set_item, int16_t set_value) {
    can_data[0] = 8;
  can_data[1] = motor.id;
    switch (set_item) {
        case MLDS_SET_KP: {
            can_data[2] = 0x66;
        } break;

        case MLDS_SET_KI: {
            can_data[2] = 0x6A;
        } break;

        case MLDS_SET_KD: {
            can_data[2] = 0x6C;
        } break;

        default: {
        } break;
    }
    can_data[3] = 0x00;
    memcpy(&can_data[4], &set_value, 2);
    can_data[6] = 0x00;
    can_data[7] = 0x00;
       can_send_message(motor.can_selected, CAN_ID_STD, motor.id, 8,can_data);
}