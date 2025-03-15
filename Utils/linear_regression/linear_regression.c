/**
 * @file    linear_regression.c
 * @author  Deadline039
 * @brief   线性回归算法
 * @version 1.0
 * @date    2025-02-14
 * @ref     https://www.codesansar.com/numerical-methods/linear-regression-method-using-c-programming.htm
 */

/**
 * @brief 线性回归算法
 * 
 * @param[in] x x 坐标数据
 * @param[in] y y 坐标数据
 * @param[in] n 数据长度
 * @param[out] k 结果的 k
 * @param[out] b 结果的 b
 */
void linear_regression(const float *x, const float *y, const int n, float *k,
                       float *b) {
    if (n <= 0) {
        return;
    }

    float sumX = 0, sumX2 = 0, sumY = 0, sumXY = 0;
    float temp_k, temp_b;

    /* Calculating Required Sum */
    for (int i = 0; i < 5; i++) {
        sumX = sumX + x[i];
        sumX2 = sumX2 + x[i] * x[i];
        sumY = sumY + y[i];
        sumXY = sumXY + x[i] * y[i];
    }

    /* Calculating k and b */
    temp_k = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);
    temp_b = (sumY - temp_k * sumX) / n;

    *k = temp_k;
    *b = temp_b;
}