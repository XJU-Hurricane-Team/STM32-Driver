/**
 * @file smd_usart.h
 * @author PickingChip 
 * @brief 正点原子 步进电机驱动器 USART通信代码
 * @version 0.1
 * @date 2026-04-03
 * 
 * 
 */

#ifndef __SMD_USART_H
#define __SMD_USART_H

#include "stdio.h"
#include "stdbool.h"
#include <cubemx.h>

/******************************************************************************************/

#define RX_BUFFER_SIZE     128
#define SMD_UART           (&huart2)

/******************************************************************************************/
/* 控制RS485_RE脚, 控制RS485发送/接收状态
 * RS485_RE = 0, 进入接收模式
 * RS485_RE = 1, 进入发送模式
 */
#define RS485_RE(x)                                                            \
    do {                                                                       \
        x ? HAL_GPIO_WritePin(RS485_RE_GPIO_Port, RS485_RE_Pin,           \
                              GPIO_PIN_SET)                                    \
          : HAL_GPIO_WritePin(RS485_RE_GPIO_Port, RS485_RE_Pin,           \
                              GPIO_PIN_RESET);                                 \
    } while (0)

extern uint8_t g_rx_cmd[RX_BUFFER_SIZE]; /* 存放接收到的指令 */


void smd_usart_send_cmd(uint8_t *data,
                        uint8_t len); /* 串口发送发送多个字节数据 */
void atk_smd_usart_init(void);

#endif /* __SMD_USART_H */
