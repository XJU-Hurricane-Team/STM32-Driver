/**
 * @file    omni_wheels.c
 * @authors CV_Engineer_CHEN
 * @brief   舵轮底盘控制
 * @version 1.0
 * @date    2025-4-18
 * @note    底盘局部坐标系
 */

#ifndef __OMNI_WHEELS_H
#define __OMNI_WHEELS_H

#ifdef USE_ARM_MATH
#include "arm_math.h"
#define SIN_F32(x)  arm_sin_f32(x)
#define COS_F32(x)  arm_cos_f32(x)
#define ACOS_F32(x) acosf(x)
#else /* USE_ARM_MATH */
#include <math.h>
#define SIN_F32(x)  sinf(x)
#define COS_F32(x)  cosf(x)
#define ACOS_F32(x) acosf(x)
#endif /* USE_ARM_MATH */

#define OMNI_WHEEL_NUM 4
#define OMNI_RADIU     1.0f

#include <stdint.h>
#include <stdbool.h>

/* 全向轮底盘电机控制函数指针 */
typedef void (*omni_single_wheel_ctrl_t)(float /* speed */[]);

void omni_wheel_init(omni_single_wheel_ctrl_t wheel_ctrl, float *yaw_angle,
                     bool coordinate);
void omni_wheel_ctrl(float target_speedx, float target_speedy,
                     float target_speedw);
void omni_set_self_coordinate(void);
void omni_set_world_coordinate(void);
bool omni_is_world_coordinate(void);

#endif /* __OMNI_WHEELS_H */