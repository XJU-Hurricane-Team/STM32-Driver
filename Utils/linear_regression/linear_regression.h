/**
 * @file    linear_regression.h
 * @author  Deadline039
 * @brief   线性回归算法
 * @version 1.0
 * @date    2025-02-14
 * @ref     https://www.codesansar.com/numerical-methods/linear-regression-method-using-c-programming.htm
 *
 *****************************************************************************
 * Change Logs:
 * Date         Version     Author      Notes
 * 2025-02-14   1.0         Deadline039 第一次发布
 */

#ifndef __LINEAR_REGRESSION_H
#define __LINEAR_REGRESSION_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

void linear_regression(const float *x, const float *y, const int n, float *k,
                       float *b);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __LINEAR_REGRESSION_H */