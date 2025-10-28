/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ads8864.h"
#include "arm_math.h"
#include "can_bsp.h"
#include "oled.h"
#include "stdio.h"

#define LED1 PCout(13)   //LED1
#define LED2 PCout(14)   //LE2
#define LED3 PCout(15)   //LED3
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern uint8_t adc_data[BUFFER_SIZE];
extern int adc_rawdata[ADC_SAMPLE_SIZE];
extern int adc_FIRdata[ADC_SAMPLE_SIZE];
extern int adc_mean[2];

float Q_2 = 0;
int i=0;
static uint8_t TxData[12];

const int32_t v_code1 = 683;
const int32_t v_code2 = 63157;
uint32_t v_codex = 0;

float GF = 16.0f/62474.0f;

uint16_t RGB = 180;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
void getNum(int n,uint8_t laser_data[])
{
	int s,len;
	
	if(n>100 && n<1000) len=3;
	if(n>1000)len=4;
	
	for(int i=0;i<len;i++)
		{
			s=n%10;
			laser_data[i]=(uint8_t)s+'0';
			n=n/10;
		}	
}


uint8_t getFloatNum(float n, uint8_t tx_data[], int decimal_places)
{
    char buffer[10];
    
    snprintf(buffer, sizeof(buffer), "%.*f", decimal_places, n);
		tx_data[0] = 0x73; 					/* Ö¡Í·S */
    int len = strlen(buffer);
    for(int i = 0; i < len; i++) {
        tx_data[i+1] = (uint8_t)buffer[i];
    }
		tx_data[len+1] = 0x65;			/* Ö¡Î² */
    return (len+2);
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  MX_TIM15_Init();
  /* USER CODE BEGIN 2 */
	
	ads8864_Init();
	LL_mDelay(20);
	
  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	
/**************ÂË²¨Æ÷*******************/
		Mean_filter();

/**************Calc**********************/
		
		Q_2 = 4 + GF*(adc_mean[0] - v_code1);
		
		
		uint8_t len = getFloatNum(Q_2 , TxData, 4);
		
		HAL_UART_Transmit(&huart1, TxData, len, 1000);
		
/**************CAN**********************/		
	
/**************OLED**********************/

/**************UART**********************/		
		HAL_Delay(20);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_2)
  {
  }
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {

  }
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_9);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_SetSystemCoreClock(72000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
  LL_RCC_SetUSARTClockSource(LL_RCC_USART1_CLKSOURCE_PCLK1);
  LL_RCC_SetI2CClockSource(LL_RCC_I2C1_CLKSOURCE_HSI);
  LL_RCC_SetTIMClockSource(LL_RCC_TIM15_CLKSOURCE_PCLK2);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
