/**
 * @file    omni_wheels.c
 * @authors CV_Engineer_CHEN
 * @brief   舵轮底盘控制
 * @version 1.0
 * @date    2025-4-18
 * @note    底盘局部坐标系
 *                 y
 *                 |
 *                 |           z轴逆时针正
 *                 |
 *                 |
 *                (0)—-——-——-——-——>x
 *          四轮底盘由前轮为一号, 以横向定义序号 (Z 字型), 如下图:
 *                (1)------------------(2)
 *                 |                    |
 *                 |                    |
 *                 |                    |
 *                 |                    |
 *                 |                    |
 *                 |                    |
 *                (3)------------------(4)
 *          角度相关都为逆时针为正 (角速度, theta 分解)
 *
 *          三轮底盘由前轮为一号, 逆时针定义序号
 *                          (1)
 *                        /     \
 *                      /         \
 *                    /             \
 *                  /                 \
 *                /                     \
 *              (2)---------------------(3)
 *          角度相关都为逆时针为正 (角速度, theta 分解)
 ******************************************************************************
 */

#include "omni_wheels.h"
#include "my_math/my_math.h"

#ifndef PI
#define PI 3.1415926
#endif /* PI */

/* 根号2/2 */
#ifndef SQRT2_2
#define SQRT2_2 0.7071067812f
#endif /* SQRT2_2 */

/* 根号3/2 */
#ifndef SQRT3_2
#define SQRT3_2 0.8660254038f
#endif /* SQRT3_2 */

/* 二分之一 */
#ifndef ONE_2
#define ONE_2 0.5f
#endif /* ONE_2 */

omni_single_wheel_ctrl_t single_wheel_ctrl;
float *world_angle;
bool world_coordinate;

/**
 * @brief 设置世界坐标系
 */
void omni_set_world_coordinate(void) {
    world_coordinate = true;
}

/**
 * @brief 设置自身坐标系
 */
void omni_set_self_coordinate(void) {
    world_coordinate = false;
}

/**
 * @brief 获取当前是否是世界坐标
 * 
 */
bool omni_is_world_coordinate(void) {
    return world_coordinate;
}

/**
 * @brief 全向轮底盘初始化
 * 
 * @param wheel_ctrl 电机控制函数指针
 * @param yaw_angle 世界坐标系下的地址，输入[-PI ~ +PI]
 * @param coordinate 坐标系选择：世界坐标[1],局部坐标[0]
 */
void omni_wheel_init(omni_single_wheel_ctrl_t wheel_ctrl, float *yaw_angle,
                     bool coordinate) {
    single_wheel_ctrl = wheel_ctrl;
    world_angle = yaw_angle;
    world_coordinate = coordinate;
}

/**
 * @brief 全向轮底盘初始化速度 
 * 
 * @param target_speedx 目标速度x
 * @param target_speedy 目标速度y
 * @param target_speedw 目标自转速度w
 */
void omni_wheel_ctrl(float target_speedx, float target_speedy,
                     float target_speedw) {
    float speedx, speedy, speedw;      /* 车身自身坐标系下的目标速度 */
    float speed_wheel[OMNI_WHEEL_NUM]; /* 每个轮子的转速 */
    float world_angle_rad = DEG2RAD(*world_angle);

    /* 判定世界坐标开启or关闭 */
    if (world_coordinate) {
        /* 如果开启世界坐标 */
        speedx = COS_F32(world_angle_rad) * target_speedx +
                 SIN_F32(world_angle_rad) * target_speedy;
        speedy = -SIN_F32(world_angle_rad) * target_speedx +
                 COS_F32(world_angle_rad) * target_speedy;
        speedw = target_speedw * OMNI_RADIU;
    } else {
        /* 如果关闭世界坐标 */
        speedx = target_speedx;
        speedy = target_speedy;
        speedw = target_speedw;
    }

#if (3 == OMNI_WHEEL_NUM)
    speed_wheel[0] = -speedx + speedw;
    speed_wheel[1] = speedx * ONE_2 - speedy * SQRT3_2 + speedw;
    speed_wheel[2] = speedx * ONE_2 + speedy * SQRT3_2 + speedw;
#elif (4 == OMNI_WHEEL_NUM)
    speed_wheel[0] = -speedx * SQRT2_2 - speedy * SQRT2_2 + speedw;
    speed_wheel[1] = -speedx * SQRT2_2 + speedy * SQRT2_2 + speedw;
    speed_wheel[2] = speedx * SQRT2_2 - speedy * SQRT2_2 + speedw;
    speed_wheel[3] = speedx * SQRT2_2 + speedy * SQRT2_2 + speedw;
#endif /* OMNI_WHEEL_NUM */

    /* 调用轮子控制函数 */
    single_wheel_ctrl(speed_wheel);
}
