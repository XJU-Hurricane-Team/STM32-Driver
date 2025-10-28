/**
 * @file    dt35.c
 * @author  Deadline039, PickingChip
 * @brief   DT35解析
 * @version 0.2
 * @date    2024-05-07
 */

#include "dt35.h"
#include "stdlib.h"

/* 两个DT35的数据 */
/* [0]为x方向 [1]为y方向 */
dt35_data_t g_dt35_data[2];

/**
 * @brief DT35串口接收回调
 *
 * @param huart 串口句柄
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {

    dt35_data_t *dt35_ptr;

    if (huart->Instance == g_dt35_data[0].uart_handle->Instance) {
        /* 第一个DT35数据 */
        dt35_ptr = &g_dt35_data[0];
    } else if (huart->Instance == g_dt35_data[1].uart_handle->Instance) {
        /* 第2个DT35数据 */
        dt35_ptr = &g_dt35_data[1];
    } else {
        return;
    }
    for (uint8_t i = 0; i < Size; i++) {
        switch (dt35_ptr->recv_it_buf[i]) {
            case 'e': {
                /* 数据结束标识符，将缓存区中的字符串转换成数字 */
                dt35_ptr->dt35_raw =
                    (float32_t)atof((const char *)dt35_ptr->recv_buf);
                dt35_ptr->distance = (dt35_ptr->Q2_far - dt35_ptr->Q2_near) *
                                         (dt35_ptr->dt35_raw - 4) / 16 +
                                     dt35_ptr->Q2_near;
                dt35_ptr->recv_sta = 0;
                memset(dt35_ptr->recv_buf, 0, Size);
            } break;

            case 's': {
                dt35_ptr->recv_sta |= 0x80;
            } break;

            default: {
                if (dt35_ptr->recv_sta != 0) {
                    dt35_ptr->recv_buf[dt35_ptr->recv_sta & 0x7f] =
                        dt35_ptr->recv_it_buf[i];
                    ++dt35_ptr->recv_sta;
                }
            } break;
        }
    }

    HAL_UARTEx_ReceiveToIdle_IT(huart, dt35_ptr->recv_it_buf, 14);
}

/**
 * @brief DT35注册串口
 *
 * @param huart_1 第一个DT35串口
 * @param huart_2 第二个DT35串口
 */
void dt35_register_uart(UART_HandleTypeDef *huart_1,
                        UART_HandleTypeDef *huart_2) {
    g_dt35_data[0].uart_handle = huart_1;
    g_dt35_data[0].Q2_near = 140; /* 需要自行标定 */
    g_dt35_data[0].Q2_far = 2050; /* 需要自行标定 */

    HAL_UARTEx_ReceiveToIdle_IT(g_dt35_data[0].uart_handle,
                                g_dt35_data[0].recv_it_buf, 14);

    g_dt35_data[1].uart_handle = huart_1;
    g_dt35_data[1].Q2_near = 140; /* 需要自行标定 */
    g_dt35_data[1].Q2_far = 2050; /* 需要自行标定 */

    HAL_UARTEx_ReceiveToIdle_IT(g_dt35_data[1].uart_handle,
                                g_dt35_data[1].recv_it_buf, 14);
}
