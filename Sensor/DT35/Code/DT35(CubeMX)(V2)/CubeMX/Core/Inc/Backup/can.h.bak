/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan;

/* USER CODE BEGIN Private defines */
/**
 * @brief Select which CAN will be used.
 */
typedef enum {
    can1_selected = 0U, /*!< Select CAN1 */
    can2_selected,      /*!< Select CAN2 */
    can3_selected       /*!< Select CAN3 */
} can_selected_t;

/* Wait for can tx mailbox empty times. */
#define CAN_SEND_TIMEOUT        1000

/* 需要手动说明CAN的使能状态 */
#define CAN1_ENABLE 1
#define CAN2_ENABLE 0
#define CAN3_ENABLE 0

#define FDCAN1_ENABLE 0
#define FDCAN2_ENABLE 0
#define FDCAN3_ENABLE 0

/* F3 系列 bxCAN 实例名为 CAN，兼容 can_list 中的 CAN1_BASE. */
#ifndef CAN1_BASE
#define CAN1_BASE CAN_BASE
#endif
/* USER CODE END Private defines */

void MX_CAN_Init(void);

/* USER CODE BEGIN Prototypes */
uint8_t can_send_message(can_selected_t can_selected, uint32_t can_ide,
                         uint32_t id, uint8_t len, const uint8_t *msg);
uint8_t can_send_remote(can_selected_t can_selected, uint32_t can_ide,
                         uint32_t id, uint8_t len, const uint8_t *msg);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

