/**
 * @file    msg_protocol.h
 * @author  Deadline039
 * @brief   消息协议
 * @version 2.4
 * @date    2024-03-01
 *
 *****************************************************************************
 *                             ##### 如何使用 ####
 * (#) 在`msg_id_t`枚举中添加要使用的数据通信类型
 *
 * (#) 发送
 *      (##) 调用`message_register_send_uart`注册串口消息通讯句柄, 发送将会
 *           使用这个函数注册的句柄
 *      (##) 调用`message_send_data`来发送数据. 如果要更改串口, 重新调用
 *           `message_register_uart_handle`更改发送串口句柄
 *      (##) `message_send_data`函数需要指定消息 ID (`msg_id_t`), 消息数据
 *           类型 (`msg_type_t), `data`(数据指针, 也就是要发送的数据), 
 *           以及`data_len`, 数据长度
 * (#) 接收
 *      (##) 调用`message_register_polling_uart`添加消息 ID 对应的轮询串口
 *      (##) 调用`message_register_recv_callback`注册接收回调函数, 当收到消息
 *           以后会调用回调函数.
 *      (##) 需要持续调用`message_polling_data`来轮询消息, 可以放到 RTOS 的一个任
 *           务或者定时器里. 当收到消息后根据`msg_id_t`来调用相应的回调函数
 *      (##) 回调函数参数形式必须是void func(uint32_t, uint8_t, uint8_t*)
 *           第一个参数是消息长度, 第二个参数是消息标识 (高四位是 ID, 低四位是数据类型)
 *           第三个参数是数据区内容, 无返回值
 *      (##) `message_polling_data`仅支持 DMA 接收
 ******************************************************************************
 *    Date    | Version |   Author    | Version Info
 * -----------+---------+-------------+----------------------------------------
 * 2024-04-13 |   1.0   | Deadline039 | 初版
 * 2025-03-13 |   2.0   | Deadline039 | 添加 FIFO
 * 2025-04-20 |   2.1   | Deadline039 | 添加转义
 * 2025-04-26 |   2.2   | Deadline039 | 修复缩容扩容错误
 * 2025-05-10 |   2.3   | Deadline039 | 改用环形队列接收消息
 * 2025-05-30 |   2.4   | Deadline039 | 添加 CRC8 校验
 */

#ifndef __MSG_PROTOCOL_H
#define __MSG_PROTOCOL_H

#include "stm32f3xx_hal.h"
#include <stdint.h>
#include <stdlib.h>

/* 帧结束标志 (End Of Frame), 注意需要避开数据头标识和长度 */
#define MSG_EOF               0x7F
/* 转义标识 (Escape), 注意需要避开头标识和长度 */
#define MSG_ESC               0x8F
/* 启用 CRC8 */
#define MSG_ENABLE_CRC8       0

/* 线程安全处理, 启用后会使用互斥信号量来保护发送缓冲区, 仅支持 FreeRTOS. */
#define MSG_ENABLE_RTOS       0

/* 始能统计, 启用后统计接收成功错误计数, 队列最大深度等信息 */
#define MSG_ENABLE_STATISTICS 0

/* 内存分配相关 */
#define MSG_MALLOC(x)         malloc(x)
#define MSG_REALLOC(p, x)     realloc(p, x)
#define MSG_FREE(p)           free(p)

/**
 * @brief 数据含义
 */
typedef enum {
    MSG_ID_1, /*!< demo 1, TX2->RX3 */
    MSG_ID_2, /*!< demo 2, TX3->RX4 */
    MSG_ID_3, /*!< demo 3, TX4->RX5 */
    MSG_ID_4, /*!< demo 4, TX5->RX2 */

    MSG_ID_RESERVE_LEN /*!< 保留位, 用于定义数据长度 */
} msg_id_t;

/**
 * @brief 数据类型
 */
typedef enum {
    MSG_DATA_UINT8 = 0x00U,
    MSG_DATA_INT8,
    MSG_DATA_UINT16,
    MSG_DATA_INT16,
    MSG_DATA_INT32,
    MSG_DATA_UINT32,
    MSG_DATA_INT64,
    MSG_DATA_UINT64,
    MSG_DATA_FP32,
    MSG_DATA_FP64,
    MSG_DATA_STRING,
    MSG_DATA_CUSTOM, /*!< 自定义数据类型 */
    /*!< 可以在下面加自定义的数据类型 */

} msg_type_t;

/**
 * @brief 回调函数指针定义
 *
 * @param msg_length 消息帧长度
 * @param msg_id_type 消息 ID 和数据类型 (高四位为 ID, 低四位为数据类型)
 * @param[in] msg_data 消息数据接收区
 */
typedef void (*msg_recv_callback_t)(uint32_t /* msg_length */,
                                    uint8_t /* msg_id_type */,
                                    uint8_t * /* msg_data */);

void message_register_send_uart(msg_id_t msg_id, UART_HandleTypeDef *huart,
                                uint32_t buf_size);
void message_register_recv_callback(msg_id_t msg_id,
                                    msg_recv_callback_t msg_callback);
void message_register_polling_uart(msg_id_t msg_id, UART_HandleTypeDef *huart,
                                   uint32_t buf_size, uint32_t fifo_size);

void message_send_data(msg_id_t msg_id, msg_type_t data_type, uint8_t *data,
                       uint32_t data_len);

void message_polling_data(void);

#endif /* __MSG_PROTOCOL_H */
