/**
 * @file    my_math.h
 * @author  Deadline039
 * @brief   精简数学库, 封装一些常用的函数
 * @version 1.1
 * @date    2024-03-02
 ******************************************************************************
 *    Date    | Version |   Author    | Version Info
 * -----------+---------+-------------+----------------------------------------
 * 2024-03-02 |   1.0   | Deadline039 | 初版
 * 2024-05-04 |   1.1   | Deadline039 | 添加正余弦定理
 */

#ifndef __MY_MATH_H
#define __MY_MATH_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#define my_abs(X)    ((X) >= 0 ? (X) : (-(X)))
#define my_fabs(X)   ((X) >= 0.0f ? (X) : (-(X)))
#define my_max(X, Y) ((X) >= (Y) ? (X) : (Y))
#define my_min(X, Y) ((X) >= (Y) ? (Y) : (X))
#define my_limit(x, min, max)                                                  \
    (x) = (((x) <= (min)) ? (min) : (((x) >= (max)) ? (max) : (x)))

#ifndef PI
#define PI 3.1415926536L
#endif /* PI */

#define DEG2RAD(X) ((X) * PI / 180.0f) /* 角度转弧度 */
#define RAD2DEG(X) ((X) * 180.0f / PI) /* 弧度转角度 */

/**
 * @brief 数学库
 */
typedef enum {
    MATH_FP_MORETHAN = 0xFF, /*!< 第一个数比第二个数大 */
    MATH_FP_EQUATION = 0x00, /*!< 第一个数与第二个数相等 */
    MATH_FP_LESSTHAN = 0x01  /*!< 第一个数比第二个数小 */
} fp_compare_result_t;

fp_compare_result_t math_compare_float(float x, float y);
fp_compare_result_t math_compare_double(double x, double y);

float triangle_cosine_law(float a, float b, float c);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __MY_MATH_H */
