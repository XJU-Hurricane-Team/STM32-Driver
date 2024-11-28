/**
 * @file    vesc_motor.c
 * @author  Deadline039
 * @brief   VESC电机控制
 * @version 0.1
 * @date    2024-03-16
 * @see
 * https://vesc-project.com/sites/default/files/imce/u15301/VESC6_CAN_CommandsTelemetry.pdf
 */

#include "vesc_motor.h"

#include "buffer_append.h"
#include "can.h"
#include "can_list.h"

#include <stdlib.h>

/**
 * @brief 发送数据ID包装
 * @note These command numbers are put in the second byte of the 29 bit ID
 *       for the extended CAN frame. You need an extended frame (29 bits)
 *       vs. standard frame (11 bits) since bits 0-7 are reserved for
 *       numbering the individual speed controllers (0-255). With only
 *       3 bits left, only 8 commands would be available if you used a
 *       standard frame.
 */
typedef enum {
    CAN_PACKET_SET_DUTY = 0,
    CAN_PACKET_SET_CURRENT,
    CAN_PACKET_SET_CURRENT_BRAKE,
    CAN_PACKET_SET_RPM,
    CAN_PACKET_SET_POS,
    CAN_PACKET_FILL_RX_BUFFER,
    CAN_PACKET_FILL_RX_BUFFER_LONG,
    CAN_PACKET_PROCESS_RX_BUFFER,
    CAN_PACKET_PROCESS_SHORT_BUFFER,
    CAN_PACKET_STATUS,
    CAN_PACKET_SET_CURRENT_REL,
    CAN_PACKET_SET_CURRENT_BRAKE_REL,
    CAN_PACKET_SET_CURRENT_HANDBRAKE,
    CAN_PACKET_SET_CURRENT_HANDBRAKE_REL,
    CAN_PACKET_STATUS_2,
    CAN_PACKET_STATUS_3,
    CAN_PACKET_STATUS_4,
    CAN_PACKET_PING,
    CAN_PACKET_PONG,
    CAN_PACKET_DETECT_APPLY_ALL_FOC,
    CAN_PACKET_DETECT_APPLY_ALL_FOC_RES,
    CAN_PACKET_CONF_CURRENT_LIMITS,
    CAN_PACKET_CONF_STORE_CURRENT_LIMITS,
    CAN_PACKET_CONF_CURRENT_LIMITS_IN,
    CAN_PACKET_CONF_STORE_CURRENT_LIMITS_IN,
    CAN_PACKET_CONF_FOC_ERPMS,
    CAN_PACKET_CONF_STORE_FOC_ERPMS,
    CAN_PACKET_STATUS_5
} can_packet_id_t;

/**
 * @brief CAN接收回调函数
 *
 * @param can_ptr CAN列表中的指针, 在这里就是电机对象
 * @param can_rx_header CAN消息头
 * @param recv_msg 接收到的数据
 */
void vesc_can_callback(void *can_ptr, CAN_RxHeaderTypeDef *can_rx_header,
                       uint8_t *recv_msg) {
    uint32_t can_id = can_rx_header->ExtId;

    int32_t message_status = (can_id >> 8) & 0xFF;
    int32_t buffer_index = 0;
    vesc_motor_handle_t *vesc_motor = (vesc_motor_handle_t *)can_ptr;

    switch (message_status) {
        case CAN_PACKET_STATUS: {
            vesc_motor->erpm =
                buffer_get_float32(recv_msg, 1.0f, &buffer_index);
            vesc_motor->total_current =
                buffer_get_float16(recv_msg, 10.0f, &buffer_index);
            vesc_motor->duty =
                buffer_get_float16(recv_msg, 1000.0f, &buffer_index);
        } break;

        case CAN_PACKET_STATUS_2: {
            vesc_motor->amp_hours =
                buffer_get_float32(recv_msg, 10000.0f, &buffer_index);
            vesc_motor->amp_hours_charged =
                buffer_get_float32(recv_msg, 10000.0f, &buffer_index);
        } break;

        case CAN_PACKET_STATUS_3: {
            vesc_motor->watt_hours =
                buffer_get_float32(recv_msg, 10000.0f, &buffer_index);
            vesc_motor->watt_hours_charged =
                buffer_get_float32(recv_msg, 10000.0f, &buffer_index);
        } break;

        case CAN_PACKET_STATUS_4: {
            vesc_motor->mosfet_temperature =
                buffer_get_float32(recv_msg, 10.0f, &buffer_index);
            vesc_motor->motor_current =
                buffer_get_float32(recv_msg, 10.0f, &buffer_index);
            vesc_motor->total_current =
                buffer_get_float32(recv_msg, 10.0f, &buffer_index);
            vesc_motor->pid_pos =
                buffer_get_float32(recv_msg, 50.0f, &buffer_index);
        } break;

        case CAN_PACKET_STATUS_5: {
            vesc_motor->tachometer_value =
                buffer_get_int32(recv_msg, &buffer_index);
            vesc_motor->input_voltage =
                buffer_get_float16(recv_msg, 10.0f, &buffer_index);
        } break;

        default: {
        } break;
    }
}

/**
 * @brief 初始化VESC电机
 *
 * @param motor 电机结构体
 * @param id 电机ID
 * @param can_select 选择CAN1或者CAN2
 */
void vesc_motor_init(vesc_motor_handle_t *motor, uint8_t id,
                     can_select_t can_select) {
    if (motor == NULL) {
        return;
    }

    motor->vesc_id = id;
    motor->can_select = can_select;

    can_list_add_new_node(can_select, (void *)motor, id, 0xFF, CAN_ID_EXT,
                          vesc_can_callback);
}

/**
 * @brief VESC电机销毁
 *
 */
void vesc_motor_deinit(vesc_motor_handle_t *motor) {
    can_list_del_node_by_pointer(motor->can_select, (void *)motor);
}

/**
 * @brief 设置VESC电机占空比, 直接修改MOSFET的PWM输出
 *
 * @param motor 要控制的电机
 * @param duty 占空比值(-1.0 ~ 1.0)
 */
void vesc_motor_set_duty(vesc_motor_handle_t *motor, float duty) {
    int32_t index = 0;
    uint8_t buffer[4];
    buffer_append_float32(buffer, duty, 100000.0f, &index);
    can_send_message(motor->can_select, CAN_ID_EXT,
                     (motor->vesc_id | (CAN_PACKET_SET_DUTY << 8)), 4, buffer);
}

/**
 * @brief 设置VESC电机电流(毫安)
 *
 * @param motor 要控制的电机
 * @param current 电流值(-2e6 ~ 2e6)
 */
void vesc_motor_set_current(vesc_motor_handle_t *motor, float current) {
    int32_t index = 0;
    uint8_t buffer[4];
    buffer_append_float32(buffer, current, 1000.0f, &index);
    can_send_message(motor->can_select, CAN_ID_EXT,
                     (motor->vesc_id | (CAN_PACKET_SET_CURRENT << 8)), 4,
                     buffer);
}

/**
 * @brief 设置VESC电机刹车电流(毫安)
 *
 * @param motor 要控制的电机
 * @param current 电流值(-2e6 ~ 2e6)
 */
void vesc_motor_set_break_current(vesc_motor_handle_t *motor, float current) {
    int32_t index = 0;
    uint8_t buffer[4];
    buffer_append_float32(buffer, current, 1000.0f, &index);
    can_send_message(motor->can_select, CAN_ID_EXT,
                     (motor->vesc_id | (CAN_PACKET_SET_CURRENT_BRAKE << 8)), 4,
                     buffer);
}

/**
 * @brief 设置VESC电机转速
 *
 * @param motor 要控制的电机
 * @param erpm 转速值
 */
void vesc_motor_set_erpm(vesc_motor_handle_t *motor, float erpm) {
    int32_t index = 0;
    uint8_t buffer[4];
    buffer_append_float32(buffer, erpm, 1.0f, &index);
    can_send_message(motor->can_select, CAN_ID_EXT,
                     (motor->vesc_id | (CAN_PACKET_SET_RPM << 8)), 4, buffer);
}

/**
 * @brief 设置VESC电机位置(角度)
 *
 * @param motor 要控制的电机
 * @param pos 角度值
 */
void vesc_motor_set_pos(vesc_motor_handle_t *motor, float pos) {
    int32_t index = 0;
    uint8_t buffer[4];
    buffer_append_float32(buffer, pos, 1.0f, &index);
    can_send_message(motor->can_select, CAN_ID_EXT,
                     (motor->vesc_id | (CAN_PACKET_SET_POS << 8)), 4, buffer);
}

/**
 * @brief 设置VESC电机相对电流
 *
 * @param motor 要控制的电机
 * @param current 电流值(-1 ~ 1)
 */
void vesc_motor_set_relative_current(vesc_motor_handle_t *motor,
                                     float current) {
    int32_t index = 0;
    uint8_t buffer[4];
    buffer_append_float32(buffer, current, 100000.0f, &index);
    can_send_message(motor->can_select, CAN_ID_EXT,
                     (motor->vesc_id | (CAN_PACKET_SET_CURRENT_REL << 8)), 4,
                     buffer);
}

/**
 * @brief 设置VESC电机相对刹车电流
 *
 * @param motor 要控制的电机
 * @param current 电流值(-1 ~ 1)
 */
void vesc_motor_set_relative_break_current(vesc_motor_handle_t *motor,
                                           float current) {
    int32_t index = 0;
    uint8_t buffer[4];
    buffer_append_float32(buffer, current, 100000.0f, &index);
    can_send_message(motor->can_select, CAN_ID_EXT,
                     (motor->vesc_id | (CAN_PACKET_SET_CURRENT_BRAKE_REL << 8)),
                     4, buffer);
}

/**
 * @brief 设置VESC电机输入电流限制
 *
 * @param motor 要控制的电机
 * @param min_current 最小电流值(-2e6 ~ 2e6)
 * @param max_current 最大电流值(-2e6 ~ 2e6)
 * @param can_select 选择CAN1或CAN2
 * @param store_to_rom 是否存储到ROM, `true`为存储到ROM
 */
void vesc_motor_set_current_limit(vesc_motor_handle_t *motor, float min_current,
                                  float max_current, bool store_to_rom) {
    int32_t index = 0;
    uint8_t buffer[8];
    buffer_append_float32(buffer, min_current, 1000.0f, &index);
    buffer_append_float32(buffer, max_current, 1000.0f, &index);
    if (store_to_rom) {
        can_send_message(
            motor->can_select, CAN_ID_EXT,
            (motor->vesc_id | (CAN_PACKET_CONF_STORE_CURRENT_LIMITS_IN << 8)),
            8, buffer);
    } else {
        can_send_message(
            motor->can_select, CAN_ID_EXT,
            (motor->vesc_id | (CAN_PACKET_CONF_CURRENT_LIMITS_IN << 8)), 8,
            buffer);
    }
}
