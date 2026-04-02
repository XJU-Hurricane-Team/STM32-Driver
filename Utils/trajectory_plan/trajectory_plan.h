/**
 * @file trajectory_plan.h
 * @author whyyy
 * @brief 
 * @version 0.2
 * @date 2026-3-31
 * 
 * 
 */

#ifndef TRAJECTORY_PLAN_H
#define TRAJECTORY_PLAN_H

#include <stdio.h>
#include <math.h>
#include <stdint.h>

#define RAD_TO_RPM    9.549296f
#define RAD_TO_DEGREE 57.29578f
#define DEGREE_TO_RAD 0.0174533f
#define PI 3.1415926535f

/**
 * @brief 运动状态标志
 * 
 */
typedef enum {
    ACCELERATING, // 加速阶段
    UNIFORM_VELOCITY, // 匀速阶段
    DECELERATING, // 减速阶段
    FINISHED // 运动结束
} TrajectoryState;


/**
 * @brief 轨迹参数结构体
 * 
 */
typedef struct {
    float p_start;        // 起始位置 (rad)
    float p_goal;         // 目标位置 (rad)
    float v_max;          // 最大速度 (rad/s)
    float a_max;          // 最大加速度 (rad/s^2)
    float dt;             // 控制周期时间 (s)
    float total_time;     // 总运动时间 (s)
    float ta;             // 加速/减速时间 (s)
    float tv;             // 匀速时间 (s)
    float p_accel;        // 加速段距离 (rad)
    TrajectoryState state; // 当前运动状态
    float current_time;   // 运动已进行时间 (s)
    int8_t is_negative;       // 运动方向标记 (1或-1)
} Trajectory_Handler_t;

void t_trajectory_init(Trajectory_Handler_t *traj, float p_start, float p_goal, float v_max, float a_max, float dt);
int t_trajectory_update(Trajectory_Handler_t *traj, float *p_des, float *w_des);

#endif /* TRAJECTORY_PLAN_H */

