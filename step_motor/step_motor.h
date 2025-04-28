/**
 * @file    step_motor.h
 * @author  Deadline039
 * @brief   步进电机驱动
 * @version 1.0
 * @date    2025-02-03
 */

#ifndef __STEP_MOTOR_H
#define __STEP_MOTOR_H

#include <CSP_Config.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 * @brief 步进电机 GPIO 定义
 */
typedef struct {
    GPIO_TypeDef *port;
    uint32_t pin;
} step_motor_gpio_t;

/**
 * @brief 步进电机状态定义
 */
typedef enum {
    STEP_MOTOR_STATE_RESET, /*!< 初始状态, 未初始化 */
    STEP_MOTOR_STATE_STOP,  /*!< 停止状态 */
    STEP_MOTOR_STATE_RUN    /*!< 运动状态 */
} step_motor_state_t;

/**
 * @brief 步进电机方向定义
 */
typedef enum {
    /* 第一个定义为正, 第二个定义为负, 调整枚举顺序即可 */
    STEP_MOTOR_AWAY,    /*!< 远离电机方向 */
    STEP_MOTOR_TOWARDS, /*!< 朝着电机方向 */
} step_motor_dir_t;

/**
 * @brief 步进电机句柄
 */
typedef struct {
    step_motor_state_t state;   /*!< 当前状态 */
    step_motor_gpio_t en_pin;   /*!< EN 引脚 */
    step_motor_gpio_t dir_pin;  /*!< DIR 引脚 */
    step_motor_gpio_t step_pin; /*!< STEP 引脚 */

    TIM_HandleTypeDef *htim; /*!< 定时器句柄 */
    uint32_t channel;        /*!< 定时器通道 */
    uint32_t pulse_remain;   /*!< 剩余的脉冲数 */

    step_motor_dir_t dir; /*!< 电机当前方向 */
} step_motor_handle_t;

/* 步进电机运动一圈需要的脉冲数, 根据电机修改 */
#define STEP_MOTOR_CIRCLE_PULSE 400
/* 步进电机初始的方波周期, 越大频率越低, 速度越慢 */
#define STEP_MOTOR_INIT_PERIOD  1800

void step_motor_init(step_motor_handle_t *handle);
void step_motor_deinit(step_motor_handle_t *handle);

void step_motor_enable(step_motor_handle_t *handle);
void step_motor_disable(step_motor_handle_t *handle);
void step_motor_set_dir(step_motor_handle_t *handle, step_motor_dir_t dir);
void step_mtoor_set_speed(step_motor_handle_t *handle, uint16_t period);

void step_motor_run(step_motor_handle_t *handle, int32_t pulse);
void step_motor_interrupt_callback(step_motor_handle_t *handle);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __STEP_MOTOR_H */
