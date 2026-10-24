#include "can_bsp.h"
#include <string.h>

CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
static uint8_t TxData[8];
uint8_t FreeTxNum;
uint32_t TxMailbox = 0;

void CAN_Init(void)
{
	CAN_FilterTypeDef  sFilterConfig;
	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14; // meaningless
	if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
	{
		/* Filter configuration Error */
		Error_Handler();
	}

	if (HAL_CAN_Start(&hcan) != HAL_OK)
	{
		/* Start Error */
		Error_Handler();
	}
	
	if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
	{
		/* Activation Error */
		Error_Handler();
	}
	
	
	TxHeader.StdId = 0x00;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = 8; 
	TxHeader.TransmitGlobalTime = DISABLE;
}

void CAN_Send_Message(uint32_t ID, uint8_t* data, uint8_t len)
{
	memcpy(TxData, data, len);
	
	TxHeader.StdId = ID;
	TxHeader.DLC = (uint32_t)len;
	
	FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(&hcan);
	
	while (FreeTxNum == 0) 
	{
		/* If no free Tx Mailboxes, clear this request */
		HAL_CAN_AbortTxRequest(&hcan,TxMailbox);
		FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(&hcan);
	}
	
	if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
	{
		/* Transmission request Error */
		Error_Handler();
	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	uint8_t RxData[8];
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
	{
		/* Reception Error */
		Error_Handler();
	}
}
