/**
 * @file    td.h
 * @brief   跟踪微分器 (Tracking Differentiator) 库
 * @version 1.0
 * @date    2026-02-09
 *
 * TD用于安排过渡过程，实时跟踪目标信号并提取微分。
 * 相比轨迹规划，TD能自动适应目标变化，无需重新初始化。
 */

#ifndef TD_H
#define TD_H


#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* ========== 配置结构体 ========== */
typedef struct {
    float r;           /**< 快速跟踪因子 = 最大加速度
                        *   调参指导: 越大跟踪越快，但可能超调
                        *   建议: 100~5000，根据系统响应能力 */

    float n;           /**< 预测周期倍率：h0 = max(n, 1) * h
                        *   物理意义: 响应速度 vs 平滑度的权衡
                        *   n=1: 最快响应，最弱滤波
                        *   n=3: 平衡（推荐默认值）
                        *   n=5: 强滤波，适合噪声大的场景 */

    float max_x2;      /**< x1 的最大变化率（0=不限制）
                        *   物理意义: 系统最大允许速度
                        *   建议: 设为电机或机械结构的极限 */
} TD_Config;

/* ========== TD实例 ========== */
typedef struct {
    /* 参数（初始化后固化） */
    float r;           /**< 快速因子 */
    float h;           /**< 采样周期（固化） */
    float h0;          /**< 内部滤波参数 = n * h */

    /* 状态 */
    float x1;          /**< 跟踪输出（平滑后的值） */
    float x2;          /**< 微分输出（速度估计） */

    /* 限制 */
    float max_x2;      /**< 速度限制 */
} TD;

/* ========== API ========== */

/**
 * @brief 初始化TD
 * @param td TD实例指针
 * @param dt 采样周期（秒），必须与实际调用周期一致
 * @param cfg 配置参数；dt、r 必须有限且大于零，max_x2 必须非负
 *
 * @example
 *   TD td;
 *   TD_Config cfg = {.r=2000, .n=3, .max_x2=50};
 *   td_init(&td, 0.001f, &cfg);
 */
void td_init(TD* td, float dt, const TD_Config* cfg);

/**
 * @brief TD更新
 * @param td TD实例
 * @param target 目标值
 * @return 平滑后的跟踪值
 *
 * @note 每周期调用一次，dt在初始化时固化，不需要每次传入
 */
float td_update(TD* td, float target);

/**
 * @brief 获取速度估计
 * @param td TD实例指针
 * @return 微分输出x2
 */
static inline float td_get_velocity(const TD* td) {
    return td->x2;
}

/**
 * @brief 获取位置估计
 * @param td TD实例指针
 * @return 跟踪输出x1
 */
static inline float td_get_position(const TD* td) {
    return td->x1;
}

/**
 * @brief 重置TD状态
 * @param td TD实例指针
 * @param init_val 初始值
 */
void td_reset(TD* td, float init_val);

/**
 * @brief fhan 最速控制函数；h 为积分周期，h0 为内部预测周期且 h0>=h
 * @note 限制 x2 后不再具有原始最速控制的收敛保证。
 */
float td_fhan(float x1, float x2, float r, float h, float h0);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* TD_H */
