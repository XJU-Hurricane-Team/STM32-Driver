/**
 * @file    can.h
 * @author  Deadline039
 * @brief   CAN通信相关
 * @version 1.0
 * @date    2023-10-26
 */

#ifndef __CAN_H
#define __CAN_H

#include "stm32f4xx_hal.h"

/**
 * @brief 选择CAN1还是CAN2, 关系到回调和控制
 */
typedef enum {
    can1_selected = 0x00U, /*!< 选择CAN1 */
    can2_selected          /*!< 选择CAN2 */
} can_select_t;

/*******************************************************************************
 * @defgroup CAN1
 * @{
 */
#define CAN1_RX_GPIO_PORT     GPIOA
#define CAN1_RX_GPIO_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()
#define CAN1_RX_GPIO_PIN      GPIO_PIN_11

#define CAN1_TX_GPIO_PORT     GPIOA
#define CAN1_TX_GPIO_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()
#define CAN1_TX_GPIO_PIN      GPIO_PIN_12

void can1_init(void);

/**
 * @}
 */

/*******************************************************************************
 * @defgroup CAN2
 * @{
 */

#define CAN2_RX_GPIO_PORT     GPIOB
#define CAN2_RX_GPIO_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()
#define CAN2_RX_GPIO_PIN      GPIO_PIN_5

#define CAN2_TX_GPIO_PORT     GPIOB
#define CAN2_TX_GPIO_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()
#define CAN2_TX_GPIO_PIN      GPIO_PIN_6

void can2_init(void);

/**
 * @}
 */

/*******************************************************************************
 * @defgroup 公共处理函数
 * @{
 */
uint8_t can_send_message(can_select_t can_select, uint32_t can_ide, uint32_t id,
                         uint8_t len, uint8_t *msg);
/**
 * @}
 */

#endif /* __CAN_H */
