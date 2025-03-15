/**
 * @file    lpf.h
 * @author  Deadline039
 * @brief   低通滤波器
 * @version 1.0
 * @date    2025-03-15
 */

#ifndef __LPF_H
#define __LPF_H

/**
 * @brief 滤波器结构体
 */
typedef struct {
    float k;    /*!< 滤波器 k 参数 */
    float last; /*!< 上次滤波的结果 */
} lpf_t;

void lpf_init(lpf_t *lpf, float k);
float lpf_calc(lpf_t *lpf, float data);

#endif /* __LPF_H */
