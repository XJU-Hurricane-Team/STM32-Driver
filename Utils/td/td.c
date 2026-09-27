/**
 * @file    td.c
 * @brief   跟踪微分器 (Tracking Differentiator) 库实现
 * @version 1.0
 * @date    2026-02-09
 */

#include "td.h"
#include <math.h>
#include <stddef.h>

/**
 * @brief 输出限幅辅助函数
 */
static inline void abs_limit(float *a, float abs_max) {
    if (*a > abs_max) {
        *a = abs_max;
    } else if (*a < -abs_max) {
        *a = -abs_max;
    }
}

/**
 * @brief TD初始化
 * @param td TD实例指针
 * @param dt 采样周期（秒）
 * @param cfg 配置参数
 */
void td_init(TD* td, float dt, const TD_Config* cfg) {
    if (td == NULL) {
        return;
    }

    td->x1 = 0.0f;
    td->x2 = 0.0f;
    td->r = 0.0f;
    td->h = 0.0f;
    td->h0 = 0.0f;
    td->max_x2 = 0.0f;
    if (cfg == NULL || !isfinite(dt) || dt <= 0.0f ||
        !isfinite(cfg->r) || cfg->r <= 0.0f ||
        !isfinite(cfg->n) || !isfinite(cfg->max_x2) || cfg->max_x2 < 0.0f) {
        return;
    }

    float n = cfg->n;
    if (n < 1.0f) {
        n = 1.0f;
    }
    if (!isfinite(n * dt)) {
        return;
    }

    td->r = cfg->r;
    td->h = dt;
    td->h0 = n * dt;
    td->max_x2 = cfg->max_x2;
}

/**
 * @brief fhan最速控制综合函数（梯形加速度曲线）
 *
 * 这是韩京清教授提出的最速控制综合函数，用于实现时间最优控制。
 * 未限制 x2 时采用有界加速度的最速控制；额外限速会改变该性质。
 *
 * @param x1 位置误差
 * @param x2 速度
 * @param r 快速跟踪因子（相当于最大加速度）
 * @param h 积分步长（采样周期）
 * @param h0 滤波因子
 * @return 加速度输出
 */
float td_fhan(float x1, float x2, float r, float h, float h0) {
    if (!isfinite(x1) || !isfinite(x2) || !isfinite(r) ||
        !isfinite(h) || !isfinite(h0) || r <= 0.0f || h <= 0.0f || h0 < h) {
        return 0.0f;
    }

    /* h 是积分周期；fhan 内部统一使用 h0 作为滤波预测周期。 */
    float d = r * h0;
    float d0 = h0 * d;
    if (!isfinite(d) || !isfinite(d0)) {
        return 0.0f;
    }

    float y = x1 + h0 * x2;

    /* 安全检查：限制 y 的范围防止 sqrtf 溢出 */
    const float MAX_Y = 1e15f;
    if (fabsf(y) > MAX_Y) {
        y = (y > 0.0f) ? MAX_Y : -MAX_Y;
    }

    float sqrt_arg = d * d + 8.0f * r * fabsf(y);

    if (!isfinite(sqrt_arg)) {
        return (y > 0.0f) ? -r : r;
    }
    if (sqrt_arg < 0.0f) {
        return 0.0f;
    }

    float a0 = sqrtf(sqrt_arg);                     /* 离散时间下的刹车轨迹方程的逆运算 */

    float a;
    if (fabsf(y) <= d0) {                           /* 线性区 */
        a = x2 + y / h0;
    } else {                                        /* 非线性区 */
        a = x2 + 0.5f * (a0 - d) * ((y > 0) ? 1.0f : -1.0f);
    }

    float fhan_out;                                 /* 输出 */
    if (fabsf(a) <= d) {                            /* 积分区 */
        fhan_out = -r * a / d;
    } else {                                        /* 非积分区 */
        fhan_out = -r * ((a > 0) ? 1.0f : -1.0f);
    }

    return fhan_out;
}

/**
 * @brief TD更新计算
 *
 * @param td TD实例指针
 * @param target 目标值
 * @return 跟踪输出x1（平滑后的目标值）
 *
 * @note 使用结构体中固化的采样周期td->h
 */
float td_update(TD* td, float target) {
    if (td == NULL) {
        return 0.0f;
    }
    if (!isfinite(target) || td->h <= 0.0f || td->r <= 0.0f) {
        return td->x1;
    }
    float x1_error = td->x1 - target;

    /* 使用固化的采样周期td->h进行计算 */
    float fh = td_fhan(x1_error, td->x2, td->r, td->h, td->h0);

    /* 使用固化的td->h进行积分 */
    float new_x2 = td->x2 + td->h * fh;

    /* 速度限制检查 */
    if (td->max_x2 > 0.0f) {
        abs_limit(&new_x2, td->max_x2);
    }

    /* 离散 TD 使用旧 x2 推进 x1，再更新 x2。 */
    td->x1 += td->h * td->x2;
    td->x2 = new_x2;

    return td->x1;
}

/**
 * @brief TD重置状态
 * @param td TD实例指针
 * @param init_val 初始值
 */
void td_reset(TD* td, float init_val) {
    if (td == NULL || !isfinite(init_val)) {
        return;
    }
    td->x1 = init_val;
    td->x2 = 0.0f;
}
