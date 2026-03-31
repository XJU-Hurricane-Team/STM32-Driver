/**
 * @file trajectory_plan.c
 * @author whyyy
 * @brief 
 * @version 0.2
 * @date 2026-3-31
 * 
 * 
 */
#include "trajectory_plan/trajectory_plan.h"


/**
 * @brief 初始化梯形轨迹，计算关键时间参数
 * * @param traj Trajectory 结构体的指针
 * @param p_start 起始位置 (rad)
 * @param p_goal 目标位置 (rad)
 * @param v_max 最大速度 (rad/s)
 * @param a_max 最大加速度 (rad/s^2)
 * @param dt 控制周期时间 (s)
 */
void t_trajectory_init(Trajectory_Handler_t *traj, float p_start, float p_goal, float v_max, float a_max, float dt) {
    
    traj->p_start = p_start;
    traj->p_goal = p_goal;
    traj->v_max = fabs(v_max); // 确保速度和加速度为正
    traj->a_max = fabs(a_max);
    traj->dt = dt;
    traj->current_time = 0.0;
    traj->state = ACCELERATING;
    
    float distance = p_goal - p_start;
    traj->is_negative = (distance < 0) ? -1 : 1; // 确定运动方向 (1或-1)
    float D = fabs(distance); // 绝对距离
    
    // 计算加/减速时间 Ta (假设达到最大速度)
    traj->ta = traj->v_max / traj->a_max;
    
    // 计算加速段距离 P_accel
    traj->p_accel = 0.5 * traj->a_max * traj->ta * traj->ta; // 0.5 * a * Ta^2
    
    // 3. 判断运动类型 (梯形 or 三角形)
    if (D < 2.0 * traj->p_accel) {
        // 距离太短，是三角形运动 (无法达到v_max) 
        
        // 计算新的最大速度 v_prime_max
        float v_prime_max = sqrt(traj->a_max * D); 
        traj->v_max = v_prime_max;
        
        // 计算新的 Ta (加速时间)
        traj->ta = traj->v_max / traj->a_max;
        traj->tv = 0.0; // 无匀速时间
        
    } else {
        // 梯形运动 (可以达到v_max)
        // 计算匀速时间 Tv
        float p_uniform = D - 2.0 * traj->p_accel;
        traj->tv = p_uniform / traj->v_max;
    }
    
  
    traj->total_time = 2.0 * traj->ta + traj->tv;
    
    // 三角形运动下调整 p_accel
    if (traj->tv == 0.0) {
        traj->p_accel = 0.5 * traj->a_max * traj->ta * traj->ta;
    }
}

/**
 * @brief 在每个控制周期计算期望位置和速度
 * * @param traj Trajectory 结构体的指针
 * @param p_des 输出：期望位置 (rad)
 * @param w_des 输出：期望速度 (rad/s)
 * @return int 1: 运动未完成, 0: 运动完成
 */
int t_trajectory_update(Trajectory_Handler_t *traj, float *p_des, float *w_des) {
    
    if (traj->state == FINISHED) {
        *p_des = traj->p_goal;
        *w_des = 0.0;
        return 0; // 运动完成
    }
    
    // 增加时间
    traj->current_time += traj->dt;
    
    // 时间到达认为运动结束
    if (traj->current_time >= traj->total_time) {
        traj->current_time = traj->total_time;
        traj->state = FINISHED; 
        *p_des = traj->p_goal;
        *w_des = 0.0;
        return 0;
    }

    float t = traj->current_time;
    float a = traj->a_max; 
    float v = traj->v_max;
    float Ta = traj->ta;
    float Tv = traj->tv;
    float P_accel = traj->p_accel;
    
    // --- 阶段判断与计算 ---
    
    if (t <= Ta) {
        // 加速阶段: 位置是 t^2，速度是 t
        traj->state = ACCELERATING;
        *p_des = traj->p_start + 0.5 * a * t * t * traj->is_negative;
        *w_des = a * t * traj->is_negative;

    } else if (t <= (Ta + Tv)) {
        // 匀速阶段: 位置是 t 的线性函数
        traj->state = UNIFORM_VELOCITY;
        *p_des = traj->p_start + (P_accel + v * (t - Ta)) * traj->is_negative;
        *w_des = v * traj->is_negative;

    } else {
        // 减速阶段: 计算剩余时间 T_rem
        traj->state = DECELERATING;
        float T_rem = traj->total_time - t; // 剩余时间
        
        // 减速阶段的位置公式
        float p_rem = 0.5 * a * T_rem * T_rem;
        *p_des = traj->p_goal - p_rem * traj->is_negative;
        
        // 减速阶段的速度公式
        *w_des = a * T_rem * traj->is_negative; 
    }
    
    return 1; // 运动未完成
}



