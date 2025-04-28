/**
 * @file    step_motor.c
 * @author  Deadline039
 * @brief   步进电机驱动
 * @version 1.0
 * @date    2025-02-05
 */

#include "step_motor.h"

#include <string.h>
#include <stdlib.h>

/**
 * @brief 步进电机初始化
 *
 * @param handle 句柄
 */
void step_motor_init(step_motor_handle_t *handle) {
    if (handle == NULL) {
        return;
    }

    GPIO_InitTypeDef gpio_init_struct = {.Mode = GPIO_MODE_OUTPUT_PP,
                                         .Pull = GPIO_PULLUP,
                                         .Speed = GPIO_SPEED_FREQ_HIGH};
    TIM_OC_InitTypeDef tim_pwm_config = {0};

    gpio_init_struct.Pin = handle->en_pin.pin;
    HAL_GPIO_Init(handle->en_pin.port, &gpio_init_struct);

    gpio_init_struct.Pin = handle->dir_pin.pin;
    HAL_GPIO_Init(handle->dir_pin.port, &gpio_init_struct);

    gpio_init_struct.Mode = GPIO_MODE_AF_PP;
    gpio_init_struct.Pin = handle->step_pin.pin;
    HAL_GPIO_Init(handle->step_pin.port, &gpio_init_struct);

    handle->htim->Init.Prescaler = 72 - 1;
    handle->htim->Init.Period = STEP_MOTOR_INIT_PERIOD - 1;
    handle->htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    handle->htim->Init.CounterMode = TIM_COUNTERMODE_UP;

    HAL_TIM_PWM_Init(handle->htim);

    tim_pwm_config.OCMode = TIM_OCMODE_PWM1;
    tim_pwm_config.OCPolarity = TIM_OCPOLARITY_HIGH;
    tim_pwm_config.Pulse = (STEP_MOTOR_INIT_PERIOD / 2) - 1;
    tim_pwm_config.OCFastMode = TIM_OCFAST_DISABLE;
    tim_pwm_config.OCIdleState = TIM_OCIDLESTATE_RESET;

    HAL_TIM_PWM_ConfigChannel(handle->htim, &tim_pwm_config, handle->channel);

    handle->pulse_remain = 0;

    handle->state = STEP_MOTOR_STATE_STOP;

    step_motor_disable(handle);
}

/**
 * @brief 设置步进电机运行速度
 * 
 * @param handle 句柄
 * @param period 脉冲周期 (单位为微秒), 越大速度越慢
 */
void step_mtoor_set_speed(step_motor_handle_t *handle, uint16_t period) {
    if (handle == NULL) {
        return;
    }

    if (handle->state == STEP_MOTOR_STATE_RUN) {
        return;
    }

    handle->htim->Instance->ARR = period - 1;
    __HAL_TIM_SET_COMPARE(handle->htim, handle->channel, period / 2);
}

/**
 * @brief 步进电机反初始化
 *
 * @param handle 句柄
 */
void step_motor_deinit(step_motor_handle_t *handle) {
    if (handle == NULL) {
        return;
    }

    HAL_TIM_PWM_Stop_IT(handle->htim, handle->channel);
}

/**
 * @brief 使能电机运动
 *
 * @param handle 句柄
 */
void step_motor_enable(step_motor_handle_t *handle) {
    if (handle == NULL) {
        return;
    }

    HAL_GPIO_WritePin(handle->en_pin.port, handle->en_pin.pin, GPIO_PIN_RESET);
}

/**
 * @brief 失能电机运动
 *
 * @param handle 句柄
 */
void step_motor_disable(step_motor_handle_t *handle) {
    if (handle == NULL) {
        return;
    }

    HAL_GPIO_WritePin(handle->en_pin.port, handle->en_pin.pin, GPIO_PIN_SET);
}

/**
 * @brief 设置电机运动方向
 *
 * @param handle 句柄
 * @param dir 方向
 */
void step_motor_set_dir(step_motor_handle_t *handle, step_motor_dir_t dir) {
    if (handle == NULL) {
        return;
    }

    switch (dir) {
        case STEP_MOTOR_TOWARDS:
            HAL_GPIO_WritePin(handle->dir_pin.port, handle->dir_pin.pin,
                              (GPIO_PinState)dir);
            break;

        case STEP_MOTOR_AWAY:
            HAL_GPIO_WritePin(handle->dir_pin.port, handle->dir_pin.pin,
                              (GPIO_PinState)dir);
            break;

        default:
            return;
    }

    handle->dir = dir;
}

/**
 * @brief 步进电机运动一段距离
 *
 * @param handle 句柄
 * @param pulse_num 脉冲个数
 */
void step_motor_run(step_motor_handle_t *handle, int32_t pulse_num) {
    if (handle == NULL) {
        return;
    }

    step_motor_enable(handle);

    if (pulse_num > 0) {
        step_motor_set_dir(handle, STEP_MOTOR_TOWARDS);
    } else if (pulse_num < 0) {
        step_motor_set_dir(handle, STEP_MOTOR_AWAY);
    } else {
        return;
    }

    handle->pulse_remain = abs(pulse_num);
    handle->state = STEP_MOTOR_STATE_RUN;

    HAL_TIM_PWM_Start_IT(handle->htim, handle->channel);
}

/**
 * @brief PWM 运行完毕中断回调
 *
 * @param handle 句柄
 */
void step_motor_interrupt_callback(step_motor_handle_t *handle) {
    if (handle == NULL) {
        return;
    }

    if (handle->pulse_remain == 0) {
        HAL_TIM_PWM_Stop_IT(handle->htim, handle->channel);
        handle->state = STEP_MOTOR_STATE_STOP;
        step_motor_disable(handle);
        return;
    }

    --handle->pulse_remain;
}
