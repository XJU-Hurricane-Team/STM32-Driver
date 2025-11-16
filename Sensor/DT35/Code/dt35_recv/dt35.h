/**
 * @file    dt35.h
 * @author  Deadline039, PickingChip
 * @brief   DT35解析
 * @version 0.2
 * @date    2024-05-02
 */

#ifndef __DT35_H
#define __DT35_H

#include "bsp.h"

/**
 * @brief DT35数据
 */
typedef struct {
    float dt35_raw; /*!< DT35原始数据，Q_2的电流值4mA~20mA */
    float distance; /*!< DT35计算后得到的距离值 */

    UART_HandleTypeDef *uart_handle; /*!< 串口接收句柄 */
    uint8_t recv_it_buf[14];         /*!< 串口中断接收buf */
    uint8_t recv_buf[12];            /*!< 串口数据缓冲区  */
    uint16_t recv_sta;               /*!< 标志位, bit7置1开始接收 */
    float Q2_near; /*!< Q2设置的近点距离，范围30~Q2_far，单位mm*/
    float Q2_far;  /*!< Q2设置的远点距离，范围Q2_near~10000，单位mm*/

} dt35_data_t;

extern dt35_data_t g_dt35_data[2];

void dt35_register_uart(UART_HandleTypeDef *huart_1,
                        UART_HandleTypeDef *huart_2);

#endif /* __DT35_H */
