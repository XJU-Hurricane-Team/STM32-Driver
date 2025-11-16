/**
 * @file    action_position.h
 * @author  Deadline039
 * @brief   东大全场定位解析代码
 * @version 0.1
 * @date    2023-11-11
 */

#ifndef __ACTION_POSITION_H
#define __ACTION_POSITION_H

#include "bsp.h"

typedef struct {
    float x;
    float y;
    float roll;
    float pitch;
    float yaw;
    float yaw_speed;
    float v;
} act_pos_data_t;

extern act_pos_data_t g_position_data;

void act_position_register_uart(UART_HandleTypeDef *huart);
void act_position_update_x(float new_x);
void act_position_update_y(float new_y);
void act_position_update_yaw(float new_yaw);
void act_position_reset_data(void);

#endif /* __ACTION_POSITION_H */
