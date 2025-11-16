/**
 * @file    damiao.c
 * @author  Deadline039
 * @brief   达妙电机驱动
 * @version 1.0
 * @date    2024-11-27
 */

#include "damiao.h"

#include "buffer_append/buffer_append.h"
#include "can_list/can_list.h"

#include <string.h>

#define MIT_MODE       0x000
#define POS_SPEED_MODE 0x100
#define SPEED_MODE     0x200

/**
 * @brief CAN 回调函数
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

    dm_handle_t *motor = (dm_handle_t *)node_obj;

    if (can_rx_header->id != motor->master_id) {
        return;
    }

    motor->device_id = can_msg[0] & 0x0F;
    motor->error = (can_msg[0] >> 4) & 0xF;

    int temp;

    temp = (can_msg[1] << 8) | can_msg[2];
    motor->position =
        uint_to_float(temp, -motor->pos_limit, motor->pos_limit, 16);
    temp = (can_msg[3] << 4) | (can_msg[4] >> 4);
    motor->speed = uint_to_float(temp, -motor->spd_limit, motor->spd_limit, 12);
    temp = ((can_msg[4] & 0x0F) << 8) | can_msg[5];
    motor->torque =
        uint_to_float(temp, -motor->torq_limit, motor->torq_limit, 12);
    motor->mos_temperature = (float)can_msg[6];
    motor->motor_temperature = (float)can_msg[7];
}

/**
 * @brief 达妙电机初始化
 *
 * @param motor 初始化电机结构体
 * @param master_id 主机 ID (电机反馈时使用)
 * @param device_id 电机 ID (控制时使用)
 * @param mode 模式
 * @param model 型号
 * @param pos_limit 位置绝对值范围限制
 * @param spd_limit 速度绝对值范围限制
 * @param torq_limit 扭矩绝对值范围限制
 * @param can_select 选择那一个 CAN 来通信
 * @return 初始化状态:
 * @retval - 0: 成功
 * @retval - 1: `motor`指针为空
 * @retval - 2: 添加 CAN 接收表错误
 */
uint8_t dm_motor_init(dm_handle_t *motor, uint32_t master_id,
                      uint32_t device_id, dm_mode_t mode, dm_model_t model,
                      float pos_limit, float spd_limit, float torq_limit,
                      can_selected_t can_select) {
    if (motor == NULL) {
        return 1;
    }

    motor->master_id = master_id;
    motor->device_id = device_id;
    motor->model = model;
    motor->mode = mode;
    motor->pos_limit = pos_limit;
    motor->spd_limit = spd_limit;
    motor->torq_limit = torq_limit;
    motor->can_select = can_select;

    if (can_list_add_new_node(can_select, (void *)motor, master_id, 0x7FF,
                              CAN_ID_STD, can_callback) != 0) {
        return 2;
    }

    return 0;
}

/**
 * @brief 电机反初始化
 *
 * @param motor 电机结构体
 * @return 反初始化状态:
 * @retval - 0: 成功
 * @return - 1: `motor`为空
 * @retval - 2: 移除出错
 */
uint8_t dm_motor_deinit(dm_handle_t *motor) {
    if (motor == NULL) {
        return 1;
    }

    if (can_list_del_node_by_id(motor->can_select, CAN_ID_STD,
                                motor->master_id) != 0) {
        return 2;
    }

    return 0;
}

/**
 * @brief 电机始能
 *
 * @param motor 电机指针
 */
void dm_motor_enable(dm_handle_t *motor) {
    if (motor == NULL) {
        return;
    }

    uint8_t send_msg[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    uint32_t id = motor->device_id;

    switch (motor->mode) {
        case DM_MODE_MIT: {
            id += MIT_MODE;
        } break;

        case DM_MODE_POS_SPEED: {
            id += POS_SPEED_MODE;
        } break;

        case DM_MODE_SPEED: {
            id += SPEED_MODE;
        } break;

        default:
            return;
    }

    can_send_message(motor->can_select, CAN_ID_STD, id, 8, send_msg);
}

/**
 * @brief 电机使能
 *
 * @param motor 电机指针
 */
void dm_motor_disable(dm_handle_t *motor) {
    if (motor == NULL) {
        return;
    }

    uint8_t send_msg[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
    uint32_t id = motor->device_id;

    switch (motor->mode) {
        case DM_MODE_MIT: {
            id += MIT_MODE;
        } break;

        case DM_MODE_POS_SPEED: {
            id += POS_SPEED_MODE;
        } break;

        case DM_MODE_SPEED: {
            id += SPEED_MODE;
        } break;

        default:
            return;
    }

    can_send_message(motor->can_select, CAN_ID_STD, id, 8, send_msg);
}

/**
 * @brief 保存当前位置为零点
 *
 * @param motor 电机结构体
 */
void dm_save_zero(dm_handle_t *motor) {
    if (motor == NULL) {
        return;
    }

    uint8_t send_msg[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};
    uint32_t id = motor->device_id;

    switch (motor->mode) {
        case DM_MODE_MIT: {
            id += MIT_MODE;
        } break;

        case DM_MODE_POS_SPEED: {
            id += POS_SPEED_MODE;
        } break;

        case DM_MODE_SPEED: {
            id += SPEED_MODE;
        } break;

        default:
            return;
    }
    can_send_message(motor->can_select, CAN_ID_STD, id, 8, send_msg);
}

/**
 * @brief 清除错误状态
 *
 * @param motor 电机指针
 */
void dm_clear_error(dm_handle_t *motor) {
    if (motor == NULL) {
        return;
    }

    uint8_t send_msg[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB};
    uint32_t id = motor->device_id;

    switch (motor->mode) {
        case DM_MODE_MIT: {
            id += MIT_MODE;
        } break;

        case DM_MODE_POS_SPEED: {
            id += POS_SPEED_MODE;
        } break;

        case DM_MODE_SPEED: {
            id += SPEED_MODE;
        } break;

        default:
            return;
    }
    can_send_message(motor->can_select, CAN_ID_STD, id, 8, send_msg);
}

/**
 * @brief MIT 模式控制电机
 *
 * @param motor 电机指针
 * @param position 位置
 * @param speed 速度
 * @param kp 位置比例系数
 * @param kd 位置微分系数
 * @param torque 扭矩
 */
void dm_mit_ctrl(dm_handle_t *motor, float position, float speed, float kp,
                 float kd, float torque) {
    uint8_t send_msg[8];

    uint16_t pos_tmp, spd_tmp, kp_tmp, kd_tmp, torq_tmp;

    pos_tmp = float_to_uint(position, -motor->pos_limit, motor->pos_limit, 16);
    spd_tmp = float_to_uint(speed, -motor->spd_limit, motor->spd_limit, 12);
    kp_tmp = float_to_uint(kp, DM_KP_MIN, DM_KP_MAX, 12);
    kd_tmp = float_to_uint(kd, DM_KD_MIN, DM_KD_MAX, 12);
    torq_tmp = float_to_uint(torque, -motor->torq_limit, motor->torq_limit, 12);

    send_msg[0] = (pos_tmp >> 8);
    send_msg[1] = pos_tmp;
    send_msg[2] = (spd_tmp >> 4);
    send_msg[3] = ((spd_tmp & 0xF) << 4) | (kp_tmp >> 8);
    send_msg[4] = kp_tmp;
    send_msg[5] = (kd_tmp >> 4);
    send_msg[6] = ((kd_tmp & 0xF) << 4) | (torq_tmp >> 8);
    send_msg[7] = torq_tmp;

    can_send_message(motor->can_select, CAN_ID_STD, motor->device_id + MIT_MODE,
                    8, send_msg);
}

/**
 * @brief 位置速度控制
 *
 * @param motor 电机指针
 * @param position 位置
 * @param speed 速度
 */
void dm_pos_speed_ctrl(dm_handle_t *motor, float position, float speed) {
    if (motor == NULL) {
        return;
    }

    uint8_t send_msg[8];
    memcpy(&send_msg[0], &position, sizeof(float));
    memcpy(&send_msg[4], &speed, sizeof(float));

    can_send_message(motor->can_select, CAN_ID_STD,
                    motor->device_id + POS_SPEED_MODE, 8, send_msg);
}

/**
 * @brief 电机速度控制
 *
 * @param motor 电机指针
 * @param speed 速度
 */
void dm_speed_ctrl(dm_handle_t *motor, float speed) {
    if (motor == NULL) {
        return;
    }

    uint8_t send_msg[4];
    memcpy(send_msg, &speed, sizeof(float));

    can_send_message(motor->can_select, CAN_ID_STD,
                    motor->device_id + SPEED_MODE, 4, send_msg);
}
