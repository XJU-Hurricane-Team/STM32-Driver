/**
 * @file    pid.c
 * @author  Deadline039
 * @brief   pid 实现
 * @version 1.2
 * @date    2023-10-27
 */

#include "pid.h"

#include <math.h>

/**
 * @brief PID 状态记录
 */
enum {
    NOW = 0, /*!< 本次 */
    LAST,    /*!< 上次 */
    LLAST,   /*!< 上上次 */
};

/**
 * @brief 死区限制
 *
 * @param a 传入的值
 * @param abs_max 限制值
 */
static inline void abs_limit(float *a, float abs_max) {
    if (*a > abs_max) {
        *a = abs_max;
    }
    if (*a < -abs_max) {
        *a = -abs_max;
    }
}

/**
 * @brief PID 初始化
 *
 * @param pid PID 结构体指针
 * @param maxout_p 输出限幅
 * @param integral_limit_p 积分限幅
 * @param deadband_p 死区, PID 计算的最小误差
 * @param maxerr_p 最大误差
 * @param pid_mode_p PID 模式 (如果不使用增量式 PID, 此参数无效)
 *  @arg `POSITION_PID`, 位置式 PID;
 *  @arg `DELTA_PID`, 增量式 PID
 * @param kp_p P 参数
 * @param ki_p I 参数
 * @param kd_p D 参数
 */
void pid_init(pid_t *pid, float maxout_p, float integral_limit_p,
              float deadband_p, float maxerr_p, pid_mode_t pid_mode_p,
              float kp_p, float ki_p, float kd_p) {
    pid->max_output = maxout_p;
    pid->integral_limit = integral_limit_p;
    pid->deadband = deadband_p;
    pid->max_error = maxerr_p;

    pid->kp = kp_p;
    pid->ki = ki_p;
    pid->kd = kd_p;
    pid->pos_out = 0.0f;

#if PID_USE_DELTA_PID
    pid->pid_mode = pid_mode_p;
    pid->delta_out = 0.0f;
#else  /* PID_USE_DELTA_PID */
    (void)pid_mode_p;
#endif /* PID_USE_DELTA_PID */
}

/**
 * @brief PID 参数调整
 *
 * @param pid PID 结构体指针
 * @param kp_p P 参数
 * @param ki_p I 参数
 * @param kd_p D 参数
 */
void pid_reset(pid_t *pid, float kp_p, float ki_p, float kd_p) {
    pid->kp = kp_p;
    pid->ki = ki_p;
    pid->kd = kd_p;
}

/**
 * @brief PID 计算
 *
 * @param pid PID 结构体指针
 * @param target_p 目标值
 * @param measure_p 电机测量值
 * @return PID 计算的结果
 */
float pid_calc(pid_t *pid, float target_p, float measure_p) {
    float pout, dout;

    pid->err[NOW] = target_p - measure_p;

    if (fabsf(pid->err[NOW]) > pid->max_error) {
        return 0.0f;
    }

    if (fabsf(pid->err[NOW]) < pid->deadband) {
        return 0.0f;
    }
#if PID_USE_DELTA_PID
    if (pid->pid_mode == POSITION_PID) {
#endif /* PID_USE_DELTA_PID */

        /* 位置式 PID */
        pout = pid->kp * pid->err[NOW];
        pid->iout += pid->ki * pid->err[NOW];
        dout = pid->kd * (pid->err[NOW] - pid->err[LAST]);
        abs_limit(&pid->iout, pid->integral_limit);
        pid->pos_out = pout + pid->iout + dout;
        abs_limit(&pid->pos_out, pid->max_output);
#if PID_USE_DELTA_PID
    } else if (pid->pid_mode == DELTA_PID) {
        /* 增量式 PID */
        pout = pid->kp * (pid->err[NOW] - pid->err[LAST]);
        float iout = pid->ki * pid->err[NOW];
        dout = pid->kd * (pid->err[NOW] - 2 * pid->err[LAST] + pid->err[LLAST]);
        abs_limit(&iout, pid->integral_limit);
        pid->delta_u = pout + iout + dout;
        pid->delta_out = pid->delta_lastout + pid->delta_u;
        abs_limit(&pid->delta_out, pid->max_output);
        pid->delta_lastout = pid->delta_out;
    }
#endif /* PID_USE_DELTA_PID */

#if PID_USE_DELTA_PID
    /* 状态转移 */
    pid->err[LLAST] = pid->err[LAST];
    pid->err[LAST] = pid->err[NOW];

    return pid->pid_mode == POSITION_PID ? pid->pos_out : pid->delta_out;
#else /* PID_USE_DELTA_PID */
    /* 状态转移 */
    pid->err[LAST] = pid->err[NOW];

    return pid->pos_out;

#endif /* PID_USE_DELTA_PID */
}