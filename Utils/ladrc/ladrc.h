/**
 * @file    ladrc.h
 * @author  Jackrainman
 * @brief   LADRC算法库（包含一阶和二阶LADRC）
 * @version 3.1
 * @date    2026-02-07
 ******************************************************************************
 *    Date    | Version |   Author    | Version Info
 * -----------+---------+-------------+----------------------------------------
 * 2024-04-04 |   1.0   | Jackrainman | 初版
 * 2026-02-06 |   2.2   | Jackrainman | 修复二阶LADRC的ESO公式(b*u位置)
 *           |         |             | 一阶LADRC: 二阶ESO; 二阶LADRC: 三阶ESO
 * 2026-02-07 |   3.0   | Jackrainman | 重构：TD与LADRC解耦，使用组合模式
 * 2026-02-07 |   3.1   | Jackrainman | 重构：参数固化模式，调参时h0改为滤波因子N
 */

#ifndef __LADRC_H
#define __LADRC_H

#include <stdbool.h>
#include "../td/td.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* ============================================================================
 *                          一阶 LADRC
 * ============================================================================ */

/**
 * @brief 一阶LADRC结构体
 */
typedef struct {
    /* ESO参数 */
    float beta1;    /**< ESO增益1 - 输出估计误差校正 */
    float beta2;    /**< ESO增益2 - 扰动估计误差校正 */

    /* 控制器参数 */
    float kp;       /**< 比例增益 */
    float b;        /**< 控制增益 */

    /* 状态估计 */
    float x1;       /**< 状态1: 系统输出估计 */
    float x2;       /**< 状态2: 总扰动估计 */

    /* 输出限制 */
    float max_output;   /**< 最大输出限制 */

    /* 采样周期 */
    float dt;       /**< 采样周期(秒) */

    /* 控制输出 */
    float out;      /**< 控制器输出 */

    /* TD（跟踪微分器）- 组合模式 */
    TD td;              /**< TD实例 */
    bool use_td;        /**< 是否启用TD */
} first_order_ladrc_t;

/**
 * @brief 一阶LADRC初始化
 * @param fladrc 一阶LADRC结构体指针
 * @param max_output 输出限幅值
 * @param beta1 ESO增益1
 * @param beta2 ESO增益2
 * @param kp 比例增益
 * @param b 控制增益
 * @param dt 采样周期(秒) - 固定采样周期，将固化到结构体中
 * @param td_r TD快速跟踪因子(0表示不使用TD)
 * @param td_n TD滤波因子(无量纲) - 建议值1~5，值越大滤波越强但延迟越大
 * @param td_max_x2 TD最大速度限制(0表示不限制)
 */
void first_order_ladrc_init(first_order_ladrc_t *fladrc, float max_output,
                            float beta1, float beta2, float kp, float b,
                            float dt,
                            float td_r, float td_n, float td_max_x2);

/**
 * @brief 一阶LADRC参数重置
 * @param fladrc 一阶LADRC结构体指针
 * @param beta1 ESO增益1
 * @param beta2 ESO增益2
 * @param kp 比例增益
 * @param b 控制增益
 * @param dt 采样周期(秒)
 * @param td_r TD快速跟踪因子(0表示不使用TD)
 * @param td_n TD滤波因子(无量纲) - 建议值1~5
 * @param td_max_x2 TD最大速度限制
 */
void first_order_ladrc_reset(first_order_ladrc_t *fladrc,
                             float beta1, float beta2, float kp, float b,
                             float dt,
                             float td_r, float td_n, float td_max_x2);

/** 以当前测量值预置 ESO，并以当前目标预置 TD；输出清零。 */
void first_order_ladrc_set_state(first_order_ladrc_t *fladrc,
                                 float measure, float target);

/**
 * @brief 一阶LADRC计算函数
 * @param fladrc 一阶LADRC结构体指针
 * @param target 目标值
 * @param measure 测量值
 * @return 控制输出
 */
float first_order_ladrc_calc(first_order_ladrc_t *fladrc,
                            float target, float measure);

/**
 * @brief 获取ESO估计的扰动值
 * @param fladrc 一阶LADRC结构体指针
 * @return 扰动估计值 x2
 */
static inline float first_order_ladrc_get_disturbance(const first_order_ladrc_t *fladrc) {
    return fladrc->x2;
}

/**
 * @brief 获取ESO估计的状态值
 * @param fladrc 一阶LADRC结构体指针
 * @return 状态估计值 x1
 */
static inline float first_order_ladrc_get_state(const first_order_ladrc_t *fladrc) {
    return fladrc->x1;
}

/**
 * @brief 在线调整一阶LADRC参数（带宽法）
 * @param fladrc 一阶LADRC结构体指针
 * @param w0 观测器带宽
 * @param wc 控制器带宽
 * @param b 控制增益
 */
static inline void first_order_ladrc_tune(first_order_ladrc_t *fladrc,
                                          float w0, float wc, float b) {
    fladrc->beta1 = 2.0f * w0;
    fladrc->beta2 = w0 * w0;
    fladrc->kp = wc;
    fladrc->b = b;
}

/* ============================================================================
 *                          二阶 LADRC
 * ============================================================================ */

/**
 * @brief 二阶LADRC控制句柄
 */
typedef struct {
    /* ESO参数 */
    float beta1;        /*!< ESO增益beta1 */
    float beta2;        /*!< ESO增益beta2 */
    float beta3;        /*!< ESO增益beta3 */

    /* 控制器参数 */
    float kp;           /*!< 控制器比例增益 */
    float kd;           /*!< 控制器微分增益 */
    float b;            /*!< 控制增益 */
#ifdef LADRC_USE_2DOF
    float wr1;          /*!< 2DOF位置参考权重 */
    float wr2;          /*!< 2DOF速度参考权重 */
#endif

    /* 状态估计值 */
    float x1;           /*!< 状态x1估计值(位置) */
    float x2;           /*!< 状态x2估计值(速度) */
    float x3;           /*!< 状态x3估计值(扰动) */

    /* 输出限制 */
    float max_output;   /*!< 输出限幅 */

    /* 采样周期 */
    float dt;           /*!< 采样周期(秒) */

    /* 输出 */
    float out;          /*!< 控制输出 */

    /* TD（跟踪微分器）- 组合模式 */
    TD td;              /*!< TD实例 */
    bool use_td;        /*!< 是否启用TD */
} ladrc_t;


void ladrc_init(ladrc_t *ladrc, float max_output,
                float beta1, float beta2, float beta3,
                float kp, float kd, float b,
                float dt,
                float td_r, float td_n, float td_max_x2);

void ladrc_reset(ladrc_t *ladrc, float beta1, float beta2, float beta3,
                 float kp, float kd, float b,
                 float dt,
                 float td_r, float td_n, float td_max_x2);

/** 以当前测量值预置 ESO，并以当前目标预置 TD；输出清零。 */
void ladrc_set_state(ladrc_t *ladrc, float measure, float target);

#ifdef LADRC_USE_2DOF
void ladrc_init_2dof(ladrc_t *ladrc, float max_output,
                     float beta1, float beta2, float beta3,
                     float kp, float kd, float b,
                     float dt,
                     float td_r, float td_n, float td_max_x2,
                     float wr1, float wr2);

void ladrc_reset_2dof(ladrc_t *ladrc, float beta1, float beta2, float beta3,
                      float kp, float kd, float b,
                      float dt,
                      float td_r, float td_n, float td_max_x2,
                      float wr1, float wr2);
#endif

float ladrc_calc(ladrc_t *ladrc, float target, float measure);

/**
 * @brief 获取ESO估计的扰动值
 * @param ladrc LADRC 结构体指针
 * @return 扰动估计值 x3
 */
static inline float ladrc_get_disturbance(const ladrc_t *ladrc) {
    return ladrc->x3;
}

/**
 * @brief 获取ESO估计的速度值
 * @param ladrc LADRC 结构体指针
 * @return 速度估计值 x2
 */
static inline float ladrc_get_velocity(const ladrc_t *ladrc) {
    return ladrc->x2;
}

/**
 * @brief 在线调整LADRC参数（带宽法）
 *
 * @param ladrc LADRC 结构体指针
 * @param w0 观测器带宽
 * @param wc 控制器带宽
 * @param b 控制增益
 */
static inline void ladrc_tune(ladrc_t *ladrc, float w0, float wc, float b) {
    ladrc->beta1 = 3.0f * w0;
    ladrc->beta2 = 3.0f * w0 * w0;
    ladrc->beta3 = w0 * w0 * w0;
    ladrc->kp = wc * wc;
    ladrc->kd = 2.0f * wc;
    ladrc->b = b;
}



#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __LADRC_H */
