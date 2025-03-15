/**
 * @file    lpf.c
 * @author  Deadline039
 * @brief   低通滤波器
 * @version 1.0
 * @date    2025-03-15
 */

#include "lpf.h"

#include <stdlib.h>

/**
 * @brief 低通滤波器初始化
 * 
 * @param lpf 滤波器结构体
 * @param k 滤波器参数
 */
void lpf_init(lpf_t *lpf, float k) {
    if (lpf == NULL) {
        return;
    }

    lpf->k = k;
}

/**
 * @brief 滤波器结果计算
 * 
 * @param lpf 滤波器结构体
 * @param data 当前数据
 * @return 滤波后数据
 */
float lpf_calc(lpf_t *lpf, float data) {
    if (lpf == NULL) {
        return 0.0f;
    }

    float filtered = lpf->last * lpf->k + data * (1 - lpf->k);
    lpf->last = filtered;

    return filtered;
}