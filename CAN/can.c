/**
 * @file    can.c
 * @author  Deadline039
 * @brief   CAN通信相关
 * @version 1.0
 * @date    2023-10-26
 */

#include "can.h"
#include "can_list.h"

#include <assert.h>

static CAN_RxHeaderTypeDef can_rx_header;
static CAN_TxHeaderTypeDef can_tx_header;

/*******************************************************************************
 * @defgroup CAN1
 * @{
 */

static CAN_HandleTypeDef can1_handle;

/**
 * @brief CAN1初始化
 *
 * @note 默认波特率配置为1Mpbs, 如需其他波特率请自行计算
 */
void can1_init(void) {
    HAL_StatusTypeDef res = HAL_OK;

    /** 1. 配置CAN外设 **************************************************/
    can1_handle.Instance = CAN1;

    can1_handle.Init.TimeTriggeredMode = DISABLE;
    can1_handle.Init.AutoBusOff = DISABLE;
    can1_handle.Init.AutoWakeUp = DISABLE;
    can1_handle.Init.AutoRetransmission = ENABLE;
    can1_handle.Init.ReceiveFifoLocked = DISABLE;
    can1_handle.Init.TransmitFifoPriority = DISABLE;
    can1_handle.Init.Mode = CAN_MODE_NORMAL;

    /**
     * CAN 波特率 = RCC_APB1Periph_CAN1 / ((SJW + BS1 + BS2) * Prescale);
     * SJW = Synchronisation Jump Width
     * BS = Bit Segment
     * 例如:      SJW_1TQ, BS1_6TQ, BS2_8TQ, Prescale = 3
     * 则波特率为: 45M / ((1 + 6 + 8) * 3) = 1000 Kbps
     */
    can1_handle.Init.SyncJumpWidth = CAN_SJW_1TQ;
    can1_handle.Init.TimeSeg1 = CAN_BS1_6TQ;
    can1_handle.Init.TimeSeg2 = CAN_BS2_8TQ;
    can1_handle.Init.Prescaler = 3;

    res = HAL_CAN_Init(&can1_handle);

#ifdef DEBUG
    assert(res == HAL_OK);
#endif /* DEBUG */

    /** 2. 配置过滤器 ****************************************************/
    CAN_FilterTypeDef can_filter_config;

    can_filter_config.FilterBank = 0;
    can_filter_config.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_config.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_config.FilterIdHigh = 0x0000;
    can_filter_config.FilterIdLow = 0x0000;
    can_filter_config.FilterMaskIdHigh = 0x0000;
    can_filter_config.FilterMaskIdLow = 0x0000;
    can_filter_config.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    can_filter_config.FilterActivation = CAN_FILTER_ENABLE;
    can_filter_config.SlaveStartFilterBank = 0;

    res = HAL_CAN_ConfigFilter(&can1_handle, &can_filter_config);

#ifdef DEBUG
    assert(res == HAL_OK);
#endif /* DEBUG */

    /** 3. 启动CAN外设 **************************************************/
    res = HAL_CAN_Start(&can1_handle);

#ifdef DEBUG
    assert(res == HAL_OK);
#endif /* DEBUG */

    /** 4. 配置中断 *****************************************************/
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);

    res =
        HAL_CAN_ActivateNotification(&can1_handle, CAN_IT_RX_FIFO0_MSG_PENDING);
#ifdef DEBUG
    assert(res == HAL_OK);
#endif /* DEBUG */

    HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);

    __HAL_CAN_ENABLE_IT(&can1_handle, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/**
 * @brief CAN1 RX0中断服务函数
 *
 */
void CAN1_RX0_IRQHandler(void) {
    HAL_CAN_IRQHandler(&can1_handle);
}

/**
 * @}
 */

/*******************************************************************************
 * @defgroup CAN2
 * @{
 */

static CAN_HandleTypeDef can2_handle;

/**
 * @brief CAN2初始化
 *
 */
void can2_init(void) {
    HAL_StatusTypeDef res = HAL_OK;

    /** 1. 配置CAN外设 **************************************************/
    can2_handle.Instance = CAN2;

    can2_handle.Init.TimeTriggeredMode = DISABLE;
    can2_handle.Init.AutoBusOff = DISABLE;
    can2_handle.Init.AutoWakeUp = DISABLE;
    can2_handle.Init.AutoRetransmission = ENABLE;
    can2_handle.Init.ReceiveFifoLocked = DISABLE;
    can2_handle.Init.TransmitFifoPriority = DISABLE;
    can2_handle.Init.Mode = CAN_MODE_NORMAL;

    /**
     * CAN 波特率 = RCC_APB1Periph_can2 / ((SJW + BS1 + BS2) * Prescale);
     * SJW = Synchronisation Jump Width
     * BS = Bit Segment
     * 例如:      SJW_1TQ, BS1_6TQ, BS2_8TQ, Prescale = 3
     * 则波特率为: 45M / ((1 + 6 + 8) * 3) = 1000 Kbps
     */
    can2_handle.Init.SyncJumpWidth = CAN_SJW_1TQ;
    can2_handle.Init.TimeSeg1 = CAN_BS1_6TQ;
    can2_handle.Init.TimeSeg2 = CAN_BS2_8TQ;
    can2_handle.Init.Prescaler = 3;

    res = HAL_CAN_Init(&can2_handle);

#ifdef DEBUG
    assert(res == HAL_OK);
#endif /* DEBUG */

    /** 2. 配置过滤器 ****************************************************/
    CAN_FilterTypeDef can_filter_config;

    can_filter_config.FilterBank = 14;
    can_filter_config.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_config.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_config.FilterIdHigh = 0x0000;
    can_filter_config.FilterIdLow = 0x0000;
    can_filter_config.FilterMaskIdHigh = 0x0000;
    can_filter_config.FilterMaskIdLow = 0x0000;
    can_filter_config.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    can_filter_config.FilterActivation = CAN_FILTER_ENABLE;
    can_filter_config.SlaveStartFilterBank = 14;

    res = HAL_CAN_ConfigFilter(&can2_handle, &can_filter_config);

#ifdef DEBUG
    assert(res == HAL_OK);
#endif /* DEBUG */

    /** 3. 启动CAN外设 **************************************************/
    res = HAL_CAN_Start(&can2_handle);

#ifdef DEBUG
    assert(res == HAL_OK);
#endif /* DEBUG */

    /** 4. 配置中断 *****************************************************/
    HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);

    __HAL_CAN_ENABLE_IT(&can2_handle, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/**
 * @brief CAN2 RX0中断服务函数
 *
 */
void CAN2_RX0_IRQHandler(void) {
    HAL_CAN_IRQHandler(&can2_handle);
}

/**
 * @}
 */

/*******************************************************************************
 * @defgroup 公共处理函数
 * @{
 */

/**
 * @brief CAN底层驱动
 *
 * @param hcan CAN句柄
 */
void HAL_CAN_MspInit(CAN_HandleTypeDef *hcan) {
    GPIO_InitTypeDef gpio_initure = {.Mode = GPIO_MODE_AF_PP,
                                     .Pull = GPIO_PULLUP,
                                     .Speed = GPIO_SPEED_FREQ_HIGH};

    if (hcan->Instance == CAN1) {
        __HAL_RCC_CAN1_CLK_ENABLE();

        CAN1_TX_GPIO_ENABLE();

        gpio_initure.Alternate = GPIO_AF9_CAN1;

        gpio_initure.Pin = CAN1_TX_GPIO_PIN;
        HAL_GPIO_Init(CAN1_TX_GPIO_PORT, &gpio_initure);

        CAN1_RX_GPIO_ENABLE();
        gpio_initure.Pin = CAN1_RX_GPIO_PIN;
        HAL_GPIO_Init(CAN1_RX_GPIO_PORT, &gpio_initure);
    } else if (hcan->Instance == CAN2) {
        __HAL_RCC_CAN2_CLK_ENABLE();

        CAN2_TX_GPIO_ENABLE();

        gpio_initure.Alternate = GPIO_AF9_CAN2;

        gpio_initure.Pin = CAN2_TX_GPIO_PIN;
        HAL_GPIO_Init(CAN2_TX_GPIO_PORT, &gpio_initure);

        CAN2_RX_GPIO_ENABLE();
        gpio_initure.Pin = CAN2_RX_GPIO_PIN;
        HAL_GPIO_Init(CAN2_RX_GPIO_PORT, &gpio_initure);
    }
}

extern void can_list_callback(can_select_t can_select,
                              CAN_RxHeaderTypeDef *can_rx_header,
                              uint8_t *recv_msg);

/**
 * @brief CAN RX FIFO0挂起中断回调
 *
 * @param hcan CAN句柄
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    static uint8_t recv_msg[8];
    HAL_StatusTypeDef res;
    res = HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &can_rx_header, recv_msg);

#ifdef DEBUG
    assert(res == HAL_OK);
#endif /* DEBUG */

    if (hcan->Instance == CAN1) {
        can_list_callback(can1_selected, &can_rx_header, recv_msg);
    } else if (hcan->Instance == CAN2) {
        can_list_callback(can2_selected, &can_rx_header, recv_msg);
    }
}

/**
 * @brief CAN发送一组数据
 *
 * @param id 标准ID
 * @param msg 数据指针
 * @param len 数据长度
 * @param can_select 选择使用那个CAN发送
 *  @arg `can1_selected`或者`can2_selected`
 * @param can_ide 标准帧
 *  @arg `CAN_ID_STD`或者`CAN_ID_EXT`
 * @return 发送状态 0,成功; 1,失败
 * @note 发送格式固定为: 标准ID, 数据帧
 */
uint8_t can_send_message(can_select_t can_select, uint32_t can_ide,
                         uint32_t id, uint8_t len, uint8_t *msg) {

    CAN_HandleTypeDef *can_handle =
        (can_select == can1_selected) ? (&can1_handle) : (&can2_handle);

    uint16_t wait_time = 0;
    uint32_t TxMailbox = CAN_TX_MAILBOX0;

    can_tx_header.IDE = can_ide;

    if (can_tx_header.IDE == CAN_ID_STD) {
        can_tx_header.StdId = id;
    } else {
        can_tx_header.ExtId = id;
    }

    can_tx_header.RTR = CAN_RTR_DATA; /* 数据帧 */
    can_tx_header.DLC = len;

    if (HAL_CAN_AddTxMessage(can_handle, &can_tx_header, msg, &TxMailbox) !=
        HAL_OK) {
        /* 发送消息 */
        return 1;
    }

    while (HAL_CAN_GetTxMailboxesFreeLevel(can_handle) != 3) {
        /* 等待发送完成,所有邮箱为空 */
        wait_time++;
        if (wait_time > 0xFFF) {
            /* 超时，直接中止邮箱的发送请求 */
            HAL_CAN_AbortTxRequest(can_handle, TxMailbox);
            return 1;
        }
    }
    return 0;
}

/**
 * @}
 */
