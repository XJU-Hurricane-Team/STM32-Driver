/**
 * @file    bmp180.h
 * @author  Deadline039
 * @brief   BMP180气压传感器
 * @version 1.0
 * @date    2024-05-05
 */

#ifndef __BMP180_H
#define __BMP180_H

#include <CSP_Config.h>

#define I2C_HANDLE_1 i2c1_handle

/**
 * @brief BMP180获取到的数据
 */
typedef struct {
    float tempature;  /*!< 温度, 摄氏度 */
    int32_t pressure; /*!< 气压, Pa */
    int32_t altitude; /*!< 海拔, 单位: m */
} bmp_data_t;

/* BMP180获取到的数据 */
extern bmp_data_t g_bmp_data;

void bmp180_init(void);
void bmp180_get_data(void);

#endif /* __BMP180_H */
