/**
 * @file    mlds_driver.h
 * @author  Deadline039
 * @brief   MLDS3605-C驱动器
 * @version 0.1
 * @date    2024-05-03
 */

#ifndef __MLDS_DRIVER_H
#define __MLDS_DRIVER_H

#include "../Config/CSP_Config.h"

/******************************************************************************
 * @defgroup 设置信号源
 * @{
 */
#define MLDS_SIGNAL_DIGITAL            0x00 /* 数字指令 */
#define MLDS_SIGNAL_DIFFERENT          0x01 /* 差分模拟电压 */
#define MLDS_SIGNAL_PWM                0x02 /* PWM */
#define MLDS_SIGNAL_PULSE              0x03 /* 步进脉冲 */
#define MLDS_SIGNAL_ANALOG             0x05 /* 单边模拟电压(0 ~ 5V) */
/******************************************************************************
 * @}
 */

/******************************************************************************
 * @defgroup 设置工作模式
 * @{
 */
#define MLDS_MODE_SPEED                (0x0 << 8U) /* 速度控制模式 */
#define MLDS_MODE_POSITION             (0x1 << 8U) /* 位置控制模式 */
#define MLDS_MODE_CURRENT              (0x2 << 8U) /* 电流控制模式 */
#define MLDS_MODE_AMPLIFIER            (0x3 << 8U) /* 放大器模式 */
/******************************************************************************
 * @}
 */

/******************************************************************************
 * @defgroup 反馈故障信息
 * @{
 */
#define MLDS_ERROR_TEMP_WARNING        (1U << 0) /* 温度报警 */
#define MLDS_ERROR_TEMP_PROTECT        (1U << 1) /* 温度保护 */
#define MLDS_ERROR_OVER_CURRENT        (1U << 2) /* 过流保护 */
#define MLDS_ERROR_UNDER_VOLTAGE       (1U << 3) /* 欠压保护 */
#define MLDS_ERROR_OVER_VOLTAGE        (1U << 4) /* 过压保护 */
#define MLDS_ERROR_ROM_PROTECT         (1U << 5) /* EEPROM出错保护 */
#define MLDS_ERROR_SPEED_PROTECT       (1U << 6) /* 速度失控保护 */
#define MLDS_ERROR_OVER_RUN            (1U << 8) /* 过载保护 */
#define MLDS_ERROR_OVER_POSITION_ERROR (1U << 9) /* 位置跟踪误差超限保护 */
/******************************************************************************
 * @}
 */

/**
 * @brief PID设置参数
 */
typedef enum {
    MLDS_SET_KP = 0U, /*!< 设置PID比例系数 */
    MLDS_SET_KI,      /*!< 设置PID积分系数 */
    MLDS_SET_KD,      /*!< 设置PID微分系数 */
} mlds_set_pid_t;

/**
 * @brief 伺服电机句柄
 * 
 */
typedef struct {
    uint8_t id;
    can_selected_t can_selected;
} mlds_motor_handle_t;

void mlds_set_mode(mlds_motor_handle_t motor, uint32_t mode);

void mlds_run_speed(mlds_motor_handle_t motor, int32_t speed);
void mlds_get_average_speed(mlds_motor_handle_t motor);

void mlds_set_absolute_origin(mlds_motor_handle_t motor);
void mlds_run_absolute_angle(mlds_motor_handle_t motor, float angle);
void mlds_run_relative_angle(mlds_motor_handle_t motor, float angle);
void mlds_set_speed_pid(mlds_motor_handle_t motor, mlds_set_pid_t set_item,
                        int16_t set_value);
void mlds_set_angle_pid(mlds_motor_handle_t motor, mlds_set_pid_t set_item,
                        int16_t set_value);

#endif /* __MLDS_DRIVER_H */