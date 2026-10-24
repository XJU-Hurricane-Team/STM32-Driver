/**
 ******************************************************************************
 * @file    L6360.c
 * @author  SkylerLiang  Skyler_Liang@163.com
 * @version V1.0.0
 * @date    2024-04-24
 * @brief   L6360 driver (IO-Link communication master transceiver IC)
 * @note    COPYRIGHT 2024 UESTC-LIMITI
 ******************************************************************************
 */

#include "L6360.h"

static uint8_t i2cTxBuffer[2];
static uint8_t i2cRxBuffer[2];

void L6360_Init(void)
{
	L6360_Reset();
	L6360_Set_EN_CQ(0);
	L6360_Set_EN_LPlus(0);
	HAL_Delay(1000);
	L6360_Set_EN_LPlus(1);
	HAL_Delay(1000);
}

uint8_t L6360_Set_Buadrate(IO_Link_Buadrate_t buadrate)
{
	HAL_StatusTypeDef status;
//	HAL_UART_DeInit(&L6360_UART);
	L6360_UART.Init.BaudRate = (uint32_t)buadrate;
	status = HAL_UART_Init(&L6360_UART);
	return (uint8_t)status;
}

uint8_t L6360_Set_EN_CQ(uint8_t enable)
{
	HAL_StatusTypeDef status;
	if (enable != 0U)
	{
		HAL_GPIO_WritePin(L6360_EN_CQ_GPIO_Port, L6360_EN_CQ_Pin, GPIO_PIN_SET);
//		status = HAL_UART_AbortReceive(&L6360_UART);
		status = HAL_OK;
	}
	else
	{
		HAL_GPIO_WritePin(L6360_EN_CQ_GPIO_Port, L6360_EN_CQ_Pin, GPIO_PIN_RESET);
//		status = HAL_UART_Receive_IT(&L6360_UART, &uartRxBuffer[0], 1U);
		status = HAL_OK;
	}
	return (uint8_t)status;
}

uint8_t L6360_Set_EN_LPlus(uint8_t enable)
{
	HAL_StatusTypeDef status;
	if (enable != 0U)
	{
		HAL_GPIO_WritePin(L6360_EN_LPlus_GPIO_Port, L6360_EN_LPlus_Pin, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(L6360_EN_LPlus_GPIO_Port, L6360_EN_LPlus_Pin, GPIO_PIN_RESET);
	}
	status = HAL_OK;
	return (uint8_t)status;
}

// 1 to reset the chip
uint8_t L6360_Set_RST(uint8_t enable)
{
	HAL_StatusTypeDef status;
	if (enable != 0U)
	{
		HAL_GPIO_WritePin(L6360_RST_GPIO_Port, L6360_RST_Pin, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(L6360_RST_GPIO_Port, L6360_RST_Pin, GPIO_PIN_SET);
	}
	status = HAL_OK;
	return (uint8_t)status;
}

void L6360_Reset(void)
{
	L6360_Set_RST(1);
	HAL_Delay(100);
	L6360_Set_RST(0);
	HAL_Delay(100);
}

HAL_StatusTypeDef L6360_Send_IN_CQ_Data(uint8_t* pdata, uint8_t len)
{
//	HAL_StatusTypeDef status;
//	uartTxBuffer[0] = data;
//	status = HAL_UART_Transmit_IT(&L6360_UART, &uartTxBuffer[0], 1U);
//	status = HAL_UART_Transmit(&L6360_UART, pdata, len, 10);
//	return status;
	return HAL_UART_Transmit(&L6360_UART, pdata, len, 10);
}

HAL_StatusTypeDef L6360_Recv_OUT_CQ_Data(uint8_t* pdata, uint8_t len)
{
	HAL_StatusTypeDef status;
	__HAL_UART_FLUSH_DRREGISTER(&L6360_UART);
	__HAL_UART_CLEAR_FLAG(&L6360_UART, UART_FLAG_RXNE);
	status = HAL_UART_Receive(&L6360_UART, pdata, len, 10);
	return status;
}

uint8_t L6360_Read_Register(uint8_t regAdd)
{
	HAL_StatusTypeDef status;
	uint8_t data = 0U;
	i2cTxBuffer[0] = regAdd;
	uint8_t i2cAdd = L6360_I2C_ADD | L6360_I2C_ADD_OFFSET;
	status = HAL_I2C_Master_Transmit(&L6360_I2C, ((uint16_t)i2cAdd << 1U), i2cTxBuffer, 1U, 100U);
	if (status == HAL_OK)
	{
		status = HAL_I2C_Master_Receive(&L6360_I2C, ((uint16_t)i2cAdd << 1U), i2cRxBuffer, 1U, 100U);
		if (status == HAL_OK)
		{
			data = i2cRxBuffer[0];
		}
	}
	return data;
}

uint16_t L6360_Current_Read_Register(void)
{
	HAL_StatusTypeDef status;
	uint16_t data = 0U;
	uint8_t i2cAdd = L6360_I2C_ADD | L6360_I2C_ADD_OFFSET;
	status = HAL_I2C_Master_Receive(&L6360_I2C, ((uint16_t)i2cAdd << 1U), i2cRxBuffer, 2U, 100U);
	
	if (status == HAL_OK)
	{
		data = (uint16_t)i2cRxBuffer[0] | (uint16_t)((uint16_t)i2cRxBuffer[1] << 8U);
	}
	
	return (data);
}

uint8_t L6360_Write_Register(uint8_t regAdd, uint8_t data)
{
	HAL_StatusTypeDef status;
	uint8_t i2cAdd = L6360_I2C_ADD | L6360_I2C_ADD_OFFSET;
	i2cTxBuffer[0] = data;
	i2cTxBuffer[1] = (regAdd & 0x1FU) + (PARITY(data) << 5U) + (ODD_PARITY(data) << 6U) + (EVEN_PARITY(data)  << 7U);
	status = HAL_I2C_Master_Transmit(&L6360_I2C, ((uint16_t) i2cAdd << 1U), i2cTxBuffer, 2U, 100U);
	
	return (uint8_t)status;
}

uint8_t L6360_Set_Led(uint8_t id, LED_Mode_t mode)
{
	uint8_t status;
	if (id == 1)
	{
		switch(mode)
		{
			case LED_ON:
			{
				status = (L6360_Write_Register((uint8_t)L6360_LED_1_MSB, 0xFF) || L6360_Write_Register((uint8_t)L6360_LED_1_LSB, 0xFF));
				break;
			}
			case LED_OFF:
			{
				status = (L6360_Write_Register((uint8_t)L6360_LED_1_MSB, 0x00) || L6360_Write_Register((uint8_t)L6360_LED_1_LSB, 0x00));
				break;
			}
			case LED_SLOW_BLINK:
			{
				status = (L6360_Write_Register((uint8_t)L6360_LED_1_MSB, 0xFF) || L6360_Write_Register((uint8_t)L6360_LED_1_LSB, 0x00));
				break;
			}
			case LED_FAST_BLINK:
			{
				status = (L6360_Write_Register((uint8_t)L6360_LED_1_MSB, 0xF0) || L6360_Write_Register((uint8_t)L6360_LED_1_LSB, 0xF0));
				break;
			}
		}
	}
	else if (id == 2)
	{
		switch(mode)
		{
			case LED_ON:
			{
				status = (L6360_Write_Register((uint8_t)L6360_LED_2_MSB, 0xFF) || L6360_Write_Register((uint8_t)L6360_LED_2_LSB, 0xFF));
				break;
			}
			case LED_OFF:
			{
				status = (L6360_Write_Register((uint8_t)L6360_LED_2_MSB, 0x00) || L6360_Write_Register((uint8_t)L6360_LED_2_LSB, 0x00));
				break;
			}
			case LED_SLOW_BLINK:
			{
				status = (L6360_Write_Register((uint8_t)L6360_LED_2_MSB, 0x00) || L6360_Write_Register((uint8_t)L6360_LED_2_LSB, 0xFF));
				break;
			}
			case LED_FAST_BLINK:
			{
				status = (L6360_Write_Register((uint8_t)L6360_LED_2_MSB, 0x0F) || L6360_Write_Register((uint8_t)L6360_LED_2_LSB, 0x0F));
				break;
			}
		}
	}
	return (uint8_t)status;
}

