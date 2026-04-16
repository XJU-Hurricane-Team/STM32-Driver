/**
 * @file    steering_wheel.c
 * @authors Ethan Lee,Deadline039,CV_Engineer_CHEN,PickingChip
 * @brief   舵轮底盘解算
 * @version 3.0
 * @date    2025-4-12
 * @note    四轮底盘由前轮为一号, 以横向定义序号 (Z 字型), 如下图:
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
 * 本代码有两个主要思路: (By Ethan Lee)
 *   1. 主控计算的只有目标角度和速度
 *     (所以世界坐标系的计算也在主控板完成, 并发送每个轮子的目标角度和速度),
 *     具体的角度选择和 PID 交由每个轮子自己的芯片来计算;
 *   2. 速度均视为向量, 有方向和大小,
 *     并服从矢量叠加原理. 矢量叠加思想是本代码重中之重
 *   3.如果使用DJI3508电机作为航向轮设置零点时3508的屁股统一朝左（俯视图）。
 * 建议: 把复杂问题分解, 先解决平动, 再搞定转动, 最后合成. 用分解的思路来做,
 *      更快更清晰
 ******************************************************************************
 */
#include <stdbool.h>
#include <stdint.h>

#include "steering_wheel.h"
#include "my_math/my_math.h"

/* 底盘解算句柄 */
static struct
{
    float radius;                                      /*!< 轮子到中心的距离 */
    float *world_angle;                                /*!< 世界坐标的角度 */
    bool halt;                                         /*!< 是否驻停 */
    float set_wheel_angle[WHEEL_NUM];                  /*!< 设置的轮子角度 */
    float set_wheel_speed[WHEEL_NUM];                  /*!< 设置的轮子速度 */
    float *steering_wheel_angle[WHEEL_NUM];            /*!< 每个航向电机当前的角度 */
    steering_direction_ctrl_t steering_direction_ctrl; /*!< 航向电机控制函数 */
    steering_speed_ctrl_t steering_speed_ctrl;         /*!< 舵向电机控制函数 */
} steering_ctrl_handle;

/**
 * @brief 设置自锁
 *
 * @param coordinate 是否开启自锁
 */
void steering_set_halt(bool halt)
{
    steering_ctrl_handle.halt = halt;
}

/**
 * @brief 查看自锁状态
 *
 * @return 是否开启自锁
 */
bool steering_get_halt(void)
{
    return steering_ctrl_handle.halt;
}

/**
 * @brief 舵轮解算初始化函数
 *
 * @param chassis_radius 轮子到中心的距离, 单位 m
 * @param world_angle 车身相对于世界坐标的角度指针 (车身 yaw), 范围 [-180, +180] DEG
 * @param direction_ctrl 舵向电机控制函数指针
 * @param speed_ctrl 航向电机控制函数指针
 * @param steering_motor_real_angle 舵向电机角度指针, 范围 [0 ~ 2*PI] RAD
 */

void steering_wheel_init(float chassis_radius, float *world_angle,
                         const steering_direction_ctrl_t direction_ctrl,
                         const steering_speed_ctrl_t speed_ctrl,
                         float *steering_motor_real_angle[])
{
    steering_ctrl_handle.radius = chassis_radius;
    steering_ctrl_handle.world_angle = world_angle;
    steering_ctrl_handle.halt = true;
    steering_ctrl_handle.steering_direction_ctrl = direction_ctrl;
    steering_ctrl_handle.steering_speed_ctrl = speed_ctrl;

    for (uint32_t i = 0; i < WHEEL_NUM; i++)
    {
        steering_ctrl_handle.steering_wheel_angle[i] =
            steering_motor_real_angle[i];
    }
}

/**
 * @brief 角度重映射函数，将角度映射到定义域 [-pi, pi]
 * @param input 需要映射角度
 * @return 映射后的角度
 */
static float remap(float input)
{
    input = fmod(input, 2 * PI);
    if (my_fabs(input) > PI)
    {
        input = input > 0 ? (input - 2 * PI) : (input + 2 * PI);
    }
    return input;
}

/**
 * @brief 单个舵轮最小转角解算, 求解航向轮旋转角度,以及航向轮的实际速度
 *
 * @param V 航向轮的转速
 * @param num 轮子编号
 * @param theta 舵向轮在世界坐标系下需要偏转的角度
 */
void steering_wheel_single_ctrl(uint8_t num, float V, float theta)
{
    /* 轮子的角度, 作为中间量 */
    static float wheel_dir[WHEEL_NUM] = {0};

    /** 最小角选择算法
     * @var re_theta        与theta相反方向对应的角度
     * @var diff_theta      舵向轮转到theta还需要转动的角度
     * @var diff_re_theta   舵向轮转到re_theta需要转动的角度
     */

    float re_theta = 0, diff_theta = 0, diff_re_theta = 0;
    wheel_dir[num] = *(steering_ctrl_handle.steering_wheel_angle[num]);

    /* 获得相反方向对应的角度 */
    re_theta = theta > 0 ? theta - PI : theta + PI;

    diff_theta = theta - wheel_dir[num];
    /* 求得差值, 即舵向电机需要转的角度 */
    diff_re_theta = re_theta - wheel_dir[num];
    /* 超过定义域重新映射 */
    diff_theta = remap(diff_theta);
    diff_re_theta = remap(diff_re_theta);

    /* 选择较小角度 */
    if (fabsf(diff_theta) > fabsf(diff_re_theta))
    {
        /*
         * 在当前位置基础上, 转所需角度,
         * 实际转动角度是 diff_re_theta 而不是 theta,
         * theta 只是一个绝对角度, 用于控制电机.
         */
        theta = wheel_dir[num] + diff_re_theta;
        V = -V;
    }
    else
    {
        /* 在当前位置基础上, 转所需角度.
         * 实际转动角度是 diff_theta 而不是 theta (相当于重新投射到局部坐标系) */
        theta = wheel_dir[num] + diff_theta;
    }
    steering_ctrl_handle.set_wheel_speed[num] = V;
    steering_ctrl_handle.set_wheel_angle[num] = remap(theta);
}

/**
 * @brief 舵轮底盘运动解算
 *
 * @param speedx x 方向速度
 * @param speedy y 方向速度
 * @param speedw 自转的角速度
 */
void steering_wheel_ctrl(float speedx, float speedy, float speedw)
{

    static float mix_x[WHEEL_NUM] = {0};    /*!< 自身坐标系下x方向上的合速度 */
    static float mix_y[WHEEL_NUM] = {0};    /*!< 自身坐标系下y方向上的合速度 */
    static float speed_wx[WHEEL_NUM] = {0}; /*!< 自转速度分解到自身坐标系x轴上*/
    static float speed_wy[WHEEL_NUM] = {0}; /*!< 自转速度分解到自身坐标系y轴上*/

    static float wheel_angle[WHEEL_NUM] = {0}; /*!< 轮子的角度 */
    static float wheel_speed[WHEEL_NUM] = {0}; /*!< 轮子的速度 */

    static float Vx = 0.0f; /*!< 底盘世界坐标系x方向合速度 */
    static float Vy = 0.0f; /*!< 底盘世界坐标系y方向合速度 */

    float steering_select_yaw_angle = DEG2RAD(
        -(*steering_ctrl_handle.world_angle)); /*!< 偏转角，需要角度转弧度 */

    /* 需要注意航向轮是反着装的, 根据车辆装载, 可能需要修改此处 */
    Vx = speedx;
    Vy = -speedy;

#if WHEEL_NUM == 4

    /*
     * 将自转速度分解到每个舵轮的X,Y方向上，
     * 后面与平动矢量叠加，逆时针方向为自旋的正方向。
     */

    speed_wx[0] = speedw * COS_F32(PI / 4);
    speed_wx[1] = speedw * COS_F32(PI / 4);
    speed_wx[2] = speedw * -COS_F32(PI / 4);
    speed_wx[3] = speedw * -COS_F32(PI / 4);

    speed_wy[0] = speedw * -SIN_F32(PI / 4);
    speed_wy[1] = speedw * SIN_F32(PI / 4);
    speed_wy[2] = speedw * -SIN_F32(PI / 4);
    speed_wy[3] = speedw * SIN_F32(PI / 4);

    /* 锁死车辆 */
    if (steering_ctrl_handle.halt)
    {
        wheel_angle[0] = -PI / 4;
        wheel_angle[1] = PI / 4;
        wheel_angle[2] = PI / 4;
        wheel_angle[3] = -PI / 4;

        wheel_speed[0] = 0.0f;
        wheel_speed[1] = 0.0f;
        wheel_speed[2] = 0.0f;
        wheel_speed[3] = 0.0f;
    }
    else if ((math_compare_float(speedx, 0.0f) != MATH_FP_EQUATION) ||
             (math_compare_float(speedy, 0.0f) != MATH_FP_EQUATION) ||
             (math_compare_float(speedw, 0.0f) != MATH_FP_EQUATION))
    {
        /* 合成并解算 */
        for (uint32_t i = 0; i < WHEEL_NUM; i++)
        {
            /* 速度分解并合成 */
            mix_x[i] = Vx * COS_F32(steering_select_yaw_angle) +
                       Vy * SIN_F32(steering_select_yaw_angle) + speed_wx[i];
            mix_y[i] = Vy * COS_F32(steering_select_yaw_angle) -
                       Vx * SIN_F32(steering_select_yaw_angle) + speed_wy[i];
            /* 航向电机转速解算 */
            wheel_speed[i] = sqrt(pow(mix_x[i], 2) + pow(mix_y[i], 2));

            /**
             * 这里的所映射到的角度空间是一个和全场定位一样的角度空间,
             * 逆时针从 0 开始一直增大到 PI, 经过一个跳变点到 - PI, 随后逐渐增大到 0
             * 这样处理后一圈内所有角度都会有一个值与之一一对应
             * 各轮子的角度空间也是这样的
             */

            /* 舵向电机角度解算 (极坐标映射) */
            wheel_angle[i] = mix_x[i] < 0
                                 ? ACOS_F32(mix_y[i] / wheel_speed[i])
                                 : -ACOS_F32(mix_y[i] / wheel_speed[i]);
        }
    }
    else
    {
        /* 仅停止, 保持角度 */
        wheel_speed[0] = 0.0f;
        wheel_speed[1] = 0.0f;
        wheel_speed[2] = 0.0f;
        wheel_speed[3] = 0.0f;
    }
#elif WHEEL_NUM == 3
    speed_wx[0] = speedw * COS_F32(0);
    speed_wx[1] = speedw * -COS_F32(PI / 3);
    speed_wx[2] = speedw * -COS_F32(-PI / 3);

    speed_wy[0] = speedw * -SIN_F32(0);
    speed_wy[1] = speedw * -SIN_F32(PI / 3);
    speed_wy[2] = speedw * -SIN_F32(-PI / 3);

    /* 锁死车辆 */
    if (steering_ctrl_handle.halt)
    {
        wheel_angle[0] = 0;
        wheel_angle[1] = PI / 3;
        wheel_angle[2] = -PI / 3;

        wheel_speed[0] = 0.0f;
        wheel_speed[1] = 0.0f;
        wheel_speed[2] = 0.0f;
    }
    else if ((math_compare_float(speedx, 0.0f) != MATH_FP_EQUATION) ||
             (math_compare_float(speedy, 0.0f) != MATH_FP_EQUATION) ||
             (math_compare_float(speedw, 0.0f) != MATH_FP_EQUATION))
    {

        /* 合成并解算 */
        for (uint32_t i = 0; i < WHEEL_NUM; i++)
        {
            /* 速度分解并合成 */
            mix_x[i] = Vx * COS_F32(steering_select_yaw_angle) +
                       Vy * SIN_F32(steering_select_yaw_angle) + speed_wx[i];
            mix_y[i] = Vy * COS_F32(steering_select_yaw_angle) -
                       Vx * SIN_F32(steering_select_yaw_angle) + speed_wy[i];
            /* 航向电机转速解算 */
            wheel_speed[i] = sqrt(pow(mix_x[i], 2) + pow(mix_y[i], 2));

            /**
             * 这里的所映射到的角度空间是一个和全场定位陀螺仪一样的角度空间,
             * 逆时针从 0 开始一直增大到 PI, 经过一个跳变点到 - PI, 随后逐渐增大到 0
             * 这样处理后一圈内所有角度都会有一个值与之一一对应
             * 各轮子的角度空间也是这样的
             */

            /* 舵向电机角度解算 (极坐标映射) */
            wheel_angle[i] = mix_x[i] < 0
                                 ? ACOS_F32(mix_y[i] / wheel_speed[i])
                                 : -ACOS_F32(mix_y[i] / wheel_speed[i]);
        }
    }
    else
    {
        /* 仅停止, 保持角度 */
        wheel_speed[0] = 0.0f;
        wheel_speed[1] = 0.0f;
        wheel_speed[2] = 0.0f;
    }
#endif /* WHEEL_NUM */
    /* 单个轮子的角度计算 */
    for (uint32_t i = 0; i < WHEEL_NUM; i++)
    {
        steering_wheel_single_ctrl(i, wheel_speed[i], wheel_angle[i]);
    }
    /* 电机控制 */
    steering_ctrl_handle.steering_direction_ctrl(
        steering_ctrl_handle.set_wheel_angle);
    steering_ctrl_handle.steering_speed_ctrl(
        steering_ctrl_handle.set_wheel_speed);
}
