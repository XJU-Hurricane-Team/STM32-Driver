/**
 * @file    ladrc.c
 * @author  Jackrainman
 * @brief   LADRC算法库实现（包含一阶和二阶LADRC）
 * @version 3.1
 * @date    2026-02-07
 */

#include "ladrc.h"
#include <math.h>
#include <string.h>

/* ============================================================================
 *                          通用工具函数
 * ============================================================================ */

/**
 * @brief 输出限幅
 *
 * @param a 传入的值
 * @param abs_max 限制值
 */
static inline void abs_limit(float *a, float abs_max) {
    if (*a > abs_max) {
        *a = abs_max;
    } else if (*a < -abs_max) {
        *a = -abs_max;
    }
}

/* ============================================================================
 *                          一阶 LADRC 实现
 * ============================================================================ */

/**
 * @brief 一阶LADRC初始化
 *
 * @note 一阶LADRC只使用比例反馈 kp，扰动补偿由二阶ESO估计值 x2 完成。
 */
void first_order_ladrc_init(first_order_ladrc_t *fladrc, float max_output,
                           float beta1, float beta2, float kp, float b,
                           float dt,
                           float td_r, float td_n, float td_max_x2) {

    if (fladrc == NULL) {
        return;
    }
    memset(fladrc, 0, sizeof(*fladrc));

    /* 初始化ESO参数 */
    fladrc->beta1 = beta1;
    fladrc->beta2 = beta2;

    /* 初始化控制器参数 */
    fladrc->kp = kp;
    fladrc->b = b;

    /* 初始化状态估计值 */
    fladrc->x1 = 0.0f;
    fladrc->x2 = 0.0f;

    /* 初始化输出限制 */
    fladrc->max_output = max_output;

    /* 初始化采样周期 - 固化到结构体中 */
    fladrc->dt = dt;

    /* 初始化输出 */
    fladrc->out = 0.0f;

    /* 初始化TD（组合模式）- 使用TD_Config */
    if (td_r > 0.0f) {
        TD_Config cfg = {
            .r = td_r,
            .n = td_n,
            .max_x2 = td_max_x2
        };
        td_init(&fladrc->td, dt, &cfg);
        fladrc->use_td = fladrc->td.h > 0.0f;
    } else {
        fladrc->use_td = false;
    }
}

/**
 * @brief 一阶LADRC参数重置
 */
void first_order_ladrc_reset(first_order_ladrc_t *fladrc,
                             float beta1, float beta2, float kp, float b,
                             float dt,
                             float td_r, float td_n, float td_max_x2) {
    if (fladrc == NULL) {
        return;
    }
    fladrc->beta1 = beta1;
    fladrc->beta2 = beta2;
    fladrc->kp = kp;
    fladrc->b = b;
    fladrc->dt = dt;

    /* 参数复位会清空 ESO；运行中应随后按测量值预置状态。 */
    fladrc->x1 = 0.0f;
    fladrc->x2 = 0.0f;
    fladrc->out = 0.0f;

    /* 重新配置TD参数 - 使用TD_Config */
    if (td_r > 0.0f) {
        TD_Config cfg = {
            .r = td_r,
            .n = td_n,
            .max_x2 = td_max_x2
        };
        td_init(&fladrc->td, dt, &cfg);
        fladrc->use_td = fladrc->td.h > 0.0f;
    } else {
        fladrc->use_td = false;
        td_reset(&fladrc->td, 0.0f);
    }
}

void first_order_ladrc_set_state(first_order_ladrc_t *fladrc,
                                 float measure, float target) {
    if (fladrc == NULL || !isfinite(measure) || !isfinite(target)) {
        return;
    }
    fladrc->x1 = measure;
    fladrc->x2 = 0.0f;
    fladrc->out = 0.0f;
    if (fladrc->use_td) {
        td_reset(&fladrc->td, target);
    }
}

/**
 * @brief 一阶LADRC计算函数
 */
float first_order_ladrc_calc(first_order_ladrc_t *fladrc,
                             float target, float measure) {
    if (fladrc == NULL || !isfinite(target) || !isfinite(measure) ||
        !isfinite(fladrc->dt) || fladrc->dt <= 0.0f ||
        !isfinite(fladrc->b) || fabsf(fladrc->b) < 0.0001f ||
        !isfinite(fladrc->max_output) || fladrc->max_output <= 0.0f) {
        if (fladrc != NULL) {
            fladrc->out = 0.0f;
        }
        return 0.0f;
    }
    /*
     * 一阶LADRC原理：
     * 被控对象：ẋ = f(x, w, t) + b*u  (一阶系统)
     * 其中f(x,w,t)为总扰动，包含模型不确定性和外部扰动
     */

    /* 步骤0: TD（跟踪微分器）处理目标值 - 使用组合模式调用TD模块（参数固化模式） */
    float td_target = target;
    if (fladrc->use_td) {
        /* 参数固化模式：不再传入dt，使用结构体中固化的td->h */
        td_target = td_update(&fladrc->td, target);
    }

    /* 步骤1: 执行二阶扩张状态观测器(ESO) */
    /*
     * 一阶系统ESO公式 (二阶观测器):
     * dx1 = x2 + b*u + beta1 * (measure - x1)
     * dx2 = beta2 * (measure - x1)
     *
     * 状态含义:
     * x1 - 系统输出估计
     * x2 - 总扰动估计 (包含内部动态和外部扰动)
     *
     * 注意：使用上一时刻的实际输出(限幅后的值)进行ESO更新，防止积分饱和
     */

    /* 计算ESO微分方程 - 优化：只计算一次误差 */
    float error = measure - fladrc->x1;
    float dx1 = fladrc->x2 + fladrc->b * fladrc->out + fladrc->beta1 * error;
    float dx2 = fladrc->beta2 * error;

    /* 更新状态估计值(欧拉积分，乘以dt) */
    fladrc->x1 += dx1 * fladrc->dt;
    fladrc->x2 += dx2 * fladrc->dt;

    /* 步骤2: 计算控制量 */
    /*
     * 一阶LADRC控制律:
     * u0 = kp * (target - x1)        // 比例控制
     * u = (u0 - x2) / b              // 扰动补偿
     */

    /* 计算名义控制量u0 */
    float u0 = fladrc->kp * (td_target - fladrc->x1);

    /* 控制律无积分状态；ESO 使用上周期限幅后的实际命令。 */
    float out_temp = (u0 - fladrc->x2) / fladrc->b;

    /* 输出限幅 */
    abs_limit(&out_temp, fladrc->max_output);

    /* 保存输出用于下一次ESO计算 */
    fladrc->out = out_temp;

    return fladrc->out;
}

/* ============================================================================
 *                          二阶 LADRC 实现
 * ============================================================================ */

/**
 * @brief LADRC 初始化 (参数固化模式)
 *
 * @note 此函数将物理参数 dt 固化到结构体中，后续 update 调用不再需要传 dt。
 *
 * @param ladrc      LADRC 结构体句柄
 * @param max_output 控制量输出限幅 (例如 PWM 最大值)
 * @param beta1      ESO 状态观测器增益 1 (位置观测带宽)
 * @param beta2      ESO 状态观测器增益 2 (速度观测带宽)
 * @param beta3      ESO 状态观测器增益 3 (扰动观测带宽)
 * @param kp         控制器比例增益 (刚度)，二阶LADRC带宽法通常取 kp = wc^2
 * @param kd         控制器微分/阻尼增益，二阶LADRC带宽法通常取 kd = 2 * wc；
 *                   若简化为纯位置反馈可设为0，但会减弱速度阻尼。
 * @param b          系统增益估计值 (b0) - 决定控制量的缩放比例
 * @param dt         RTOS 固定采样周期 (秒) - [关键] 必须与实际任务频率一致，将固化到结构体
 * @param td_r       TD 快速跟踪因子 (r) - 决定目标值响应速度。0 表示禁用 TD (直接透传)
 * @param td_n       TD 滤波因子 (无量纲，建议值 1~5) - 决定噪声过滤能力。
 *                   [物理意义]: N=1: 滤波最弱，响应最快；
 *                             N=3~5: 典型推荐值，平衡滤波效果与响应速度；
 *                             N>10: 强滤波但有明显滞后。
 * @param td_max_x2  TD 最大速度限制 (0 表示不限制) - 防止设定值跳变过大导致系统冲击
 */
void ladrc_init(ladrc_t *ladrc, float max_output,
                float beta1, float beta2, float beta3,
                float kp, float kd, float b,
                float dt,
                float td_r, float td_n, float td_max_x2) {
    if (ladrc == NULL) {
        return;
    }
    memset(ladrc, 0, sizeof(*ladrc));
#ifdef LADRC_USE_2DOF
    ladrc->wr1 = 1.0f;
    ladrc->wr2 = 1.0f;
#endif

    /* 1. 初始化 ESO (扩张状态观测器) 参数 */
    ladrc->beta1 = beta1;
    ladrc->beta2 = beta2;
    ladrc->beta3 = beta3;

    /* 2. 初始化 控制器 (State Error Feedback) 参数 */
    ladrc->kp = kp;
    ladrc->kd = kd;
    ladrc->b = b;

    /* 3. 清空 ESO 内部状态 */
    ladrc->x1 = 0.0f; // 估计位置
    ladrc->x2 = 0.0f; // 估计速度
    ladrc->x3 = 0.0f; // 估计总扰动

    /* 4. 输出与抗饱和设置 */
    ladrc->max_output = max_output;

    ladrc->out = 0.0f;

    /* 5. 固化采样周期 (核心修改) */
    // 这个 dt 将被 ESO 和 TD 共同使用，作为时间基准
    ladrc->dt = dt;

    /* 6. 初始化 TD (使用 TD_Config) */
    if (td_r > 0.0f) {
        TD_Config cfg = {
            .r = td_r,
            .n = td_n,
            .max_x2 = td_max_x2
        };
        td_init(&ladrc->td, dt, &cfg);
        ladrc->use_td = ladrc->td.h > 0.0f;
    } else {
        // 禁用 TD 模式
        ladrc->use_td = false;
        // 为了安全，将 TD 状态清零
        td_reset(&ladrc->td, 0.0f);
    }
}

#ifdef LADRC_USE_2DOF
void ladrc_init_2dof(ladrc_t *ladrc, float max_output,
                     float beta1, float beta2, float beta3,
                     float kp, float kd, float b,
                     float dt,
                     float td_r, float td_n, float td_max_x2,
                     float wr1, float wr2) {
    if (ladrc == NULL) {
        return;
    }
    ladrc_init(ladrc, max_output,
               beta1, beta2, beta3,
               kp, kd, b,
               dt,
               td_r, td_n, td_max_x2);
    ladrc->wr1 = wr1;
    ladrc->wr2 = wr2;
}
#endif

/**
 * @brief LADRC 参数调整
 *
 * @param ladrc LADRC 结构体指针
 * @param beta1 ESO增益beta1
 * @param beta2 ESO增益beta2
 * @param beta3 ESO增益beta3
 * @param kp 控制器比例增益，二阶LADRC带宽法通常取 kp = wc^2
 * @param kd 控制器微分/阻尼增益，二阶LADRC带宽法通常取 kd = 2 * wc
 * @param b 控制增益
 * @param dt 采样周期(秒)
 * @param td_r TD快速跟踪因子(0表示不使用TD)
 * @param td_n TD滤波因子(无量纲，建议值1~5)
 * @param td_max_x2 TD最大速度限制
 */
void ladrc_reset(ladrc_t *ladrc, float beta1, float beta2, float beta3,
                 float kp, float kd, float b,
                 float dt,
                 float td_r, float td_n, float td_max_x2) {
    if (ladrc == NULL) {
        return;
    }
    ladrc->beta1 = beta1;
    ladrc->beta2 = beta2;
    ladrc->beta3 = beta3;
    ladrc->kp = kp;
    ladrc->kd = kd;
    ladrc->b = b;
    ladrc->dt = dt;

    /* 参数复位会清空 ESO；运行中应随后按测量值预置状态。 */
    ladrc->x1 = 0.0f;
    ladrc->x2 = 0.0f;
    ladrc->x3 = 0.0f;
    ladrc->out = 0.0f;

    /* 重新配置TD参数 - 使用TD_Config */
    if (td_r > 0.0f) {
        TD_Config cfg = {
            .r = td_r,
            .n = td_n,
            .max_x2 = td_max_x2
        };
        td_init(&ladrc->td, dt, &cfg);
        ladrc->use_td = ladrc->td.h > 0.0f;
    } else {
        ladrc->use_td = false;
        td_reset(&ladrc->td, 0.0f);
    }
}

void ladrc_set_state(ladrc_t *ladrc, float measure, float target) {
    if (ladrc == NULL || !isfinite(measure) || !isfinite(target)) {
        return;
    }
    ladrc->x1 = measure;
    ladrc->x2 = 0.0f;
    ladrc->x3 = 0.0f;
    ladrc->out = 0.0f;
    if (ladrc->use_td) {
        td_reset(&ladrc->td, target);
    }
}

#ifdef LADRC_USE_2DOF
void ladrc_reset_2dof(ladrc_t *ladrc, float beta1, float beta2, float beta3,
                      float kp, float kd, float b,
                      float dt,
                      float td_r, float td_n, float td_max_x2,
                      float wr1, float wr2) {
    if (ladrc == NULL) {
        return;
    }
    ladrc_reset(ladrc,
                beta1, beta2, beta3,
                kp, kd, b,
                dt,
                td_r, td_n, td_max_x2);
    ladrc->wr1 = wr1;
    ladrc->wr2 = wr2;
}
#endif

/**
 * @brief LADRC 计算
 *
 * @param ladrc LADRC 结构体指针
 * @param target 目标值
 * @param measure 电机测量值
 * @return LADRC 计算的结果
 */
float ladrc_calc(ladrc_t *ladrc, float target, float measure) {
    if (ladrc == NULL || !isfinite(target) || !isfinite(measure) ||
        !isfinite(ladrc->dt) || ladrc->dt <= 0.0f ||
        !isfinite(ladrc->b) || fabsf(ladrc->b) < 0.0001f ||
        !isfinite(ladrc->max_output) || ladrc->max_output <= 0.0f) {
        if (ladrc != NULL) {
            ladrc->out = 0.0f;
        }
        return 0.0f;
    }
    /* 步骤0: TD处理目标值 */
    float r1 = target;
    float r2 = 0.0f;
    if (ladrc->use_td) {
        r1 = td_update(&ladrc->td, target);
        r2 = td_get_velocity(&ladrc->td);
    }

    /* 步骤1: 执行三阶扩张状态观测器(ESO) */
    /*
     * 二阶LADRC被控对象: ẍ = f + b*u
     * 三阶ESO公式:
     * dx1 = x2 + β1*(y - x1)         <- ẋ1 = x2 (速度)
     * dx2 = x3 + b*u + β2*(y - x1)   <- ẋ2 = x3 + b*u (加速度=扰动+控制)
     * dx3 = β3*(y - x1)              <- ẋ3 = df/dt (扰动变化率)
     */

    /* 计算ESO微分方程 - 优化：只计算一次误差 */
    float error = measure - ladrc->x1;
    float dx1 = ladrc->x2 + ladrc->beta1 * error;
    float dx2 = ladrc->x3 + ladrc->b * ladrc->out + ladrc->beta2 * error;
    float dx3 = ladrc->beta3 * error;

    /* 更新状态估计值(欧拉积分，乘以dt) */
    ladrc->x1 += dx1 * ladrc->dt;
    ladrc->x2 += dx2 * ladrc->dt;
    ladrc->x3 += dx3 * ladrc->dt;

    /* 步骤2: 计算控制量u0 */
#ifdef LADRC_USE_2DOF
    /*
     * 2DOF控制律:
     * u0 = kp * (wr1 * r1 - x1) + kd * (wr2 * r2 - x2)
     * u = (u0 - x3) / b
     */
    float u0 = ladrc->kp * (ladrc->wr1 * r1 - ladrc->x1)
             + ladrc->kd * (ladrc->wr2 * r2 - ladrc->x2);
#else
    /*
     * 标准控制律:
     * u0 = kp * (r - x1) - kd * x2
     * u = (u0 - x3) / b
     */
    float u0 = ladrc->kp * (r1 - ladrc->x1) + ladrc->kd * (r2 - ladrc->x2);
#endif

    /* 控制律无积分状态；ESO 使用上周期限幅后的实际命令。 */
    float out_temp = (u0 - ladrc->x3) / ladrc->b;

    /* 输出限幅 */
    abs_limit(&out_temp, ladrc->max_output);

    /* 保存输出用于下一次ESO计算 */
    ladrc->out = out_temp;

    return ladrc->out;
}
