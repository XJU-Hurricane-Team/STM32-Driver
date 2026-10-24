/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "can_bsp.h"
#include "DT35.h"

#define DT35_ID 0x101
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
uint8_t reWakeUpFlag = 0;
uint8_t reWakeUpCounter = 0;
uint8_t wakeUpCounter = 0;
uint16_t errorCounter = 0;
volatile uint16_t errorflag = 0;
volatile uint16_t errorstate = 0;
volatile uint8_t l6360_irq_pending = 0;
volatile uint8_t l6360_runtime_monitor_enabled = 0;
uint16_t l6360_startup_state = 0;
uint16_t l6360_startup_irq_count = 0;
uint16_t processData = 0;
uint16_t processDataLast = 0;
extern M_Sequence_Type_0 m_sequence0;
extern M_Sequence_Type_2_2 m_sequence2;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
	* @brief 16bits距离值按大端拆分为两个8bits
	* @retval status
	*/
HAL_StatusTypeDef uart_send_message(UART_HandleTypeDef *huart, uint16_t data){
	uint8_t buffer[2];
	buffer[0] = (uint8_t)(data >> 8);
	buffer[1] = (uint8_t)(data & 0xFF);
	return HAL_UART_Transmit(huart, buffer, 2, 100);
}
/* USER CODE END 0 */

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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_CAN_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  CAN_Init();
  /* Ignore expected IRQ activity during L6360 power/reset sequencing. */
  l6360_runtime_monitor_enabled = 0;
  HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
  __HAL_GPIO_EXTI_CLEAR_IT(L6360_IRQ_Pin);

  L6360_Init();

  /* Keep startup status for diagnostics instead of dropping it. */
  l6360_startup_state = L6360_Current_Read_Register();
  l6360_startup_irq_count = 0;
  l6360_irq_pending = 0;

  /* Runtime error counters start from 0 after startup snapshot. */
  errorstate = 0;
  errorflag = 0;

  __HAL_GPIO_EXTI_CLEAR_IT(L6360_IRQ_Pin);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
  l6360_runtime_monitor_enabled = 1;


while (dl_wake_up() != HAL_OK && wakeUpCounter < 3)
  {
	  wakeUpCounter++;
	  HAL_Delay(300);
  }
  L6360_Set_Led(1, LED_ON);
  HAL_Delay(0);
  dl_write_type_0(&m_sequence0, 0x99, 0x00);
  HAL_Delay(1);
  DT35_Set_Resolution(1);   // 1:1mm/计数(量程可达65m,覆盖12m);0:0.1mm(仅6.5m)
//  dl_write_ISDU_uint8(0x69, 0x00);
  HAL_Delay(1);
  
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim3);
  L6360_Set_Led(2, LED_SLOW_BLINK);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (l6360_irq_pending != 0U)
    {
      l6360_irq_pending = 0;
      errorstate = L6360_Current_Read_Register();
    }

	  if (!reWakeUpFlag)
	  {
		  errorCounter += dl_read_type_2_2(&m_sequence2, &processData, NULL, Idle_1);
		  HAL_Delay(0);
	  }
	  else if (reWakeUpFlag)
	  {
		  HAL_Delay(10);
		  while (dl_reconnection_handler() != DL_OK)
		  {
			  HAL_Delay(100);
		  }
		  L6360_Set_Led(1, LED_ON);
		  L6360_Set_Led(2, LED_ON);
		  reWakeUpFlag = 0;
	  }
	  if (processData != 0 && processDataLast == 0)
	  {
		L6360_Set_Led(2, LED_ON);
	  }
	  else if (processData == 0 && processDataLast != 0)
	  {
		L6360_Set_Led(2, LED_SLOW_BLINK);
	  }
	  processDataLast = processData;
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == L6360_IRQ_Pin)
	{
    if (l6360_runtime_monitor_enabled != 0U)
    {
      l6360_irq_pending = 1;
      errorflag++;
    }
    else
    {
      l6360_startup_irq_count++;
    }
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == htim3.Instance)
	{
		if (reWakeUpFlag == 0)
		{
				uart_send_message(&huart2, processData);
		}
	}
	else if (htim->Instance == htim2.Instance)
	{
		if (errorCounter < 10)
		{
			errorCounter = 0;
		}
		else
		{
			if (reWakeUpFlag == 0)
			{
				reWakeUpFlag = 1;
				L6360_Set_Led(1, LED_FAST_BLINK);
				L6360_Set_Led(2, LED_FAST_BLINK);
				errorCounter = 0;
			}
		}
	}
}
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
	  HAL_NVIC_SystemReset();
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
