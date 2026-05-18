/**
 * @file    omni_wheels.c
 * @authors CV_Engineer_CHEN
 * @brief   舵轮底盘控制
 * @version 1.0
 * @date    2025-4-18
 * @note    底盘局部坐标系
 *                 y
 *                 ^
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

/**
 * @brief 全向轮底盘初始化
 * 
 * @param wheel_ctrl 电机控制函数指针
 * @param yaw_angle 世界坐标系下的地址，输入[-PI ~ +PI]
 */
void omni_wheel_init(omni_single_wheel_ctrl_t wheel_ctrl, float *yaw_angle) {
    single_wheel_ctrl = wheel_ctrl;
    world_angle = yaw_angle;
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

    speedx = COS_F32(world_angle_rad) * target_speedx +
             SIN_F32(world_angle_rad) * target_speedy;
    speedy = -SIN_F32(world_angle_rad) * target_speedx +
             COS_F32(world_angle_rad) * target_speedy;
    speedw = target_speedw * OMNI_RADIU;

    /* 统一引入 Vx, Vy, Vw 硬件坐标系映射，方便与正解算对称。 */
    float Vx = speedx;
    float Vy = speedy;
    float Vw = speedw;

#if (3 == OMNI_WHEEL_NUM)
    speed_wheel[0] = -Vx + Vw;
    speed_wheel[1] = Vx * ONE_2 - Vy * SQRT3_2 + Vw;
    speed_wheel[2] = Vx * ONE_2 + Vy * SQRT3_2 + Vw;
#elif (4 == OMNI_WHEEL_NUM)
    speed_wheel[0] = Vx * SQRT2_2 + Vy * SQRT2_2 + Vw;
    speed_wheel[1] = Vx * SQRT2_2 - Vy * SQRT2_2 + Vw;
    speed_wheel[2] = -Vx * SQRT2_2 + Vy * SQRT2_2 + Vw;
    speed_wheel[3] = -Vx * SQRT2_2 - Vy * SQRT2_2 + Vw;
#endif /* OMNI_WHEEL_NUM */

    /* 调用轮子控制函数 */
    single_wheel_ctrl(speed_wheel);
}

/**
 * @brief 全向轮底盘正向运动学解算
 *
 * @param wheel_speed 每个轮子的实际转速反馈
 * @param speedx 输出的底盘x方向速度（正前方）
 * @param speedy 输出的底盘y方向速度（正左方）
 * @param speedw 输出的底盘自转角速度（逆时针）
 */
void omni_wheel_forward_calc(float wheel_speed[OMNI_WHEEL_NUM], float *speedx,
                             float *speedy, float *speedw) {
    float Vx = 0.0f;
    float Vy = 0.0f;
    float Vw = 0.0f;
    float body_x, body_y, body_w;

#if (3 == OMNI_WHEEL_NUM)
    Vx = (-2.0f * wheel_speed[0] + wheel_speed[1] + wheel_speed[2]) / 3.0f;
    Vy = (-wheel_speed[1] + wheel_speed[2]) / (2.0f * SQRT3_2);
    Vw = (wheel_speed[0] + wheel_speed[1] + wheel_speed[2]) / 3.0f;
#elif (4 == OMNI_WHEEL_NUM)
    Vx = (wheel_speed[0] + wheel_speed[1] - wheel_speed[2] - wheel_speed[3]) /
         (4.0f * SQRT2_2);
    Vy = (wheel_speed[0] - wheel_speed[1] + wheel_speed[2] - wheel_speed[3]) /
         (4.0f * SQRT2_2);
    Vw = (wheel_speed[0] + wheel_speed[1] + wheel_speed[2] + wheel_speed[3]) /
         4.0f;
#endif /* OMNI_WHEEL_NUM */

    /* 根据逆解算中定义的硬件映射反推标准车体坐标系速度 */
    body_x = Vx;
    body_y = Vy;
    body_w = Vw;

    /* 逆解算中速度分解是从世界系到车体系，正解算需从车体系恢复到世界系 */
    float world_angle_rad = DEG2RAD(*world_angle);
    *speedx = COS_F32(world_angle_rad) * body_x - SIN_F32(world_angle_rad) * body_y;
    *speedy = SIN_F32(world_angle_rad) * body_x + COS_F32(world_angle_rad) * body_y;
    *speedw = body_w / OMNI_RADIU;
}
