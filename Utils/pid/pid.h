/**
 * @file    pid.h
 * @author  Deadline039
 * @brief   pid 封装
 * @version 1.2
 * @date    2023-10-27
 *
 ******************************************************************************
 *    Date    | Version |   Author    | Version Info
 * -----------+---------+-------------+----------------------------------------
 * 2024-04-04 |   1.0   | Deadline039 | 初版
 * 2024-04-16 |   1.1   | Deadline039 | 添加 ARM 数学库, 用于加速计算
 * 2024-05-03 |   1.2   | Deadline039 | 移除 ARM 数学库, 感觉用处不大
 * 2025-02-26 |   1.3   | Deadline039 | 移除依赖, 添加宏选择使用增量 PID 以减小内存占用
 */

#ifndef __PID_H
#define __PID_H

/* 是否使用增量式 PID */
#define PID_USE_DELTA_PID 1

/**
 * @brief PID 类型, 位置 PID 或者增量 PID
 */
typedef enum {
    POSITION_PID = 0x00U,
    DELTA_PID
} pid_mode_t;

/**
 * @brief PID 控制句柄
 */
typedef struct {
    float kp, ki, kd; /*!< pid 三参数 */

#if PID_USE_DELTA_PID
    float err[3]; /*!< 差值, 包含本次, 上次, 上上次 */
#else             /* PID_USE_DELTA_PID */
    float err[2]; /*!< 差值, 包含本次, 上次 */
#endif            /* PID_USE_DELTA_PID */

    float iout; /*!< pid 积分结果, 在位置式使用 */

    float max_output;     /*!< 输出限幅 */
    float integral_limit; /*!< 积分限幅 */
    float deadband;       /*!< 死区 (绝对值) */
    float max_error;      /*!< 最大误差 */

    /* 位置模式 */
    float pos_out;     /*!< 本次输出 */

#if PID_USE_DELTA_PID
    /* 增量模式 */
    float delta_u;       /*!< 本次增量值 */
    float delta_out;     /*!< 本次增量输出 = delta_lastout + delta_u */
    float delta_lastout; /*!< 上次增量输出 */
    pid_mode_t pid_mode; /*!< PID 模式 */
#endif                   /* PID_USE_DELTA_PID */

} pid_t;

void pid_init(pid_t *pid, float maxout_p, float integral_limit_p,
              float deadband_p, float maxerr_p, pid_mode_t pid_mode_p,
              float kp_p, float ki_p, float kd_p);
void pid_reset(pid_t *pid, float kp_p, float ki_p, float kd_p);
float pid_calc(pid_t *pid, float target_p, float measure_p);

#endif /* __PID_H */
