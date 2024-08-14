/**
 * @file    vesc_motor.h
 * @author  Deadline039
 * @brief   VESC电调程序
 * @version 1.0
 * @date    2024-03-16
 * @see     https://github.com/craigg96/vesc_can_bus_arduino
 *          https://vesc-project.com/sites/default/files/imce/u15301/VESC6_CAN_CommandsTelemetry.pdf
 */

#ifndef __VESC_H
#define __VESC_H

#include "can.h"

#include <stdbool.h>

/**
 * @brief VESC电机错误码
 */
typedef enum {
    VESC_FAULT_NONE = 0,         /*!< 无错误 */
    VESC_FAULT_OVER_VOLTAGE,     /*!< 过压 */
    VESC_FAULT_UNDER_VOLTAGE,    /*!< 欠压 */
    VESC_FAULT_DRV,              /*!< 驱动器错误 */
    VESC_FAULT_ABS_OVER_CURRENT, /*!< 过流 */
    VESC_FAULT_OVER_TEMP_FET,    /*!< MOS温度高 */
    VESC_FAULT_OVER_TEMP_MOTOR   /*!< 电机温度高 */
} vesc_fault_code_t;

/**
 * @brief VESC电机参数
 */
typedef struct {
    uint8_t vesc_id;         /*!< 电机 ID */
    can_select_t can_select; /*!< 选择CAN1还是CAN2 */

    float input_voltage; /*!< 电机电压 */
    float duty;          /*!< MOSFET占空比 */
    float erpm;          /*!< 转速 */
    float rpm;           /*!< 转子实际转速 */

    float amp_hours;         /*!< 电流时间 */
    float amp_hours_charged; /*!< 电流充电时间 */

    float watt_hours;         /*!< 功率时间 */
    float watt_hours_charged; /*!< 功率充电时间 */

    float motor_current; /*!< 电机电流 */
    float total_current; /*!< 总电流 */

    float mosfet_temperature; /*!< MOSFET温度 */
    float motor_temperature;  /*!< 电机温度 */

    float pid_pos; /*!< 转子位置 */

    int32_t tachometer_value;     /*!< 转速表 */
    vesc_fault_code_t error_code; /*!< 错误码 */
} vesc_motor_handle_t;

void vesc_motor_init(vesc_motor_handle_t *motor, uint8_t id,
                     can_select_t can_select);
void vesc_motor_deinit(vesc_motor_handle_t *motor);

void vesc_motor_set_duty(vesc_motor_handle_t *motor, float duty);
void vesc_motor_set_current(vesc_motor_handle_t *motor, float current);
void vesc_motor_set_break_current(vesc_motor_handle_t *motor, float current);
void vesc_motor_set_erpm(vesc_motor_handle_t *motor, float erpm);
void vesc_motor_set_pos(vesc_motor_handle_t *motor, float pos);
void vesc_motor_set_relative_current(vesc_motor_handle_t *motor, float current);
void vesc_motor_set_relative_break_current(vesc_motor_handle_t *motor,
                                           float current);
void vesc_motor_set_current_limit(vesc_motor_handle_t *motor, float min_current,
                                  float max_current, bool store_to_rom);

#endif /* __VESC_H */
