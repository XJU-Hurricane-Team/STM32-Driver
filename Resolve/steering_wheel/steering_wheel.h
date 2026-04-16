/**
 * @file steering_wheel.h
 * @authors Ethan Lee,Deadline039,CV_Engineer_CHEN
 * @brief 舵轮底盘控制
 * @version 2.0
 * @data 2024-12-4
 */

#ifndef __STEERING_WHEEL_H
#define __STEERING_WHEEL_H

#define WHEEL_NUM 3 /*!< 和解算文件中保持一致 */
/* 航向轮与舵向轮控制函数指针 */
typedef void (*steering_direction_ctrl_t)(float * /* angle */);
typedef void (*steering_speed_ctrl_t)(float * /* speed */);

#ifdef USE_ARM_MATH

#include "arm_math.h"
#define SIN_F32(x) arm_sin_f32(x)
#define COS_F32(x) arm_cos_f32(x)
#define ACOS_F32(x) acosf(x)

#else /* USE_ARM_MATH */

#include <math.h>
#define SIN_F32(x) sinf(x)
#define COS_F32(x) cosf(x)
#define ACOS_F32(x) acosf(x)

#endif /* USE_ARM_MATH */

void steering_set_halt(bool halt);
bool steering_get_halt(void);

void steering_wheel_init(float chassis_radius, float *world_angle,
                         const steering_direction_ctrl_t direction_ctrl,
                         const steering_speed_ctrl_t speed_ctrl,
                         float *steering_motor_real_angle[]);

void steering_wheel_ctrl(float speedx, float speedy, float speedw);

#endif /* __STEERING_WHEEL_H */