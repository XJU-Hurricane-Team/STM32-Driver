/**
 * @file    damiao.h
 * @author  Deadline039
 * @brief   达妙电机驱动
 * @version 1.0
 * @date    2024-11-27
 ******************************************************************************
 *    Date    | Version |   Author    | Version Info
 * -----------+---------+-------------+----------------------------------------
 * 2024-11-27 |   1.0   | Deadline039 | 初版
 */

#ifndef __DAMIAO_H
#define __DAMIAO_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include <CSP_Config.h>

/**
 * @brief 电机型号
 */
typedef enum {
    DM_J3507 = 0x00U,
    DM_J4310,
    DM_J4340,
    DM_J6006,
    DM_J8006,
    DM_J8009,
    DM_J10010,
    DM_S3519,
    DM_H6215,
    DM_G6220
} dm_model_t;

#define DM_KP_MIN 0.0f
#define DM_KP_MAX 500.0f
#define DM_KD_MIN 0.0f
#define DM_KD_MAX 5.0f

/**
 * @brief 故障信息
 */
typedef enum {
    DM_OK_DISABLED = 0x00U,      /*!< 无故障, 失能状态*/
    DM_OK_ENABLED,               /*!< 无故障, 始能状态 */
    DM_ERR_OVER_VOLTAGE = 0x08U, /*!< 过压错误 */
    DM_ERR_UNDER_VOLTAGE,        /*!< 欠压错误 */
    DM_ERR_OVER_CURRENT,         /*!< 过流错误 */
    DM_ERR_MOS_TEMPERATURE,      /*!< MOS 过温 */
    DM_ERR_MOTOR_TEMPERATURE,    /*!< 电机线圈过温 */
    DM_ERR_LOST_COMMUNICATION,   /*!< 通信丢失 */
    DM_ERR_OVER_LOAD             /*!< 过载 */
} dm_error_t;

/**
 * @brief 当前模式
 */
typedef enum {
    DM_MODE_MIT = 0x00U, /*!< MIT 控制模式 */
    DM_MODE_POS_SPEED,   /*!< 位置速度控制模式 */
    DM_MODE_SPEED        /*!< 速度控制模式 */
} dm_mode_t;

/**
 * @brief 电机控制结构体
 */
typedef struct {
    uint32_t master_id;        /*!< 反馈主机 ID */
    uint32_t device_id;        /*!< 控制设备 ID */
    can_selected_t can_select; /*!< 选择 CAN 通信 */
    dm_model_t model;          /*!< 型号 */
    dm_mode_t mode;            /*!< 当前模式 */

    float position;          /*!< 位置 */
    float speed;             /*!< 速度 */
    float torque;            /*!< 扭矩 */
    float mos_temperature;   /*!< MOS 温度 */
    float motor_temperature; /*!< 电机线圈温度 */
    dm_error_t error;        /*!< 错误信息 */

    /* 以下参数需要与上位机设定值一致, 否则会导致回传与控制的值发送错误 */

    float pos_limit;  /*!< 位置绝对值范围 */
    float spd_limit;  /*!< 速度绝对值范围 */
    float torq_limit; /*!< 扭矩绝对值范围 */
} dm_handle_t;

uint8_t dm_motor_init(dm_handle_t *motor, uint32_t master_id,
                      uint32_t device_id, dm_mode_t mode, dm_model_t model,
                      float pos_limit, float spd_limit, float torq_limit,
                      can_selected_t can_select);
uint8_t dm_motor_deinit(dm_handle_t *motor);
void dm_motor_enable(dm_handle_t *motor);
void dm_motor_disable(dm_handle_t *motor);
void dm_save_zero(dm_handle_t *motor);
void dm_clear_error(dm_handle_t *motor);

void dm_mit_ctrl(dm_handle_t *motor, float position, float speed, float kp,
                 float kd, float torque);
void dm_pos_speed_ctrl(dm_handle_t *motor, float position, float speed);
void dm_speed_ctrl(dm_handle_t *motor, float speed);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __DAMIAO_H */
