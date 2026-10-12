#include "ads8864.h"
#include "arm_math.h"
#include "gpio.h"
#include "tim.h"

int adc_rawdata[ADC_SAMPLE_SIZE] = {0};
uint8_t adc_data[BUFFER_SIZE] = {0};
int adc_mean[1] = {0};

uint16_t adc_counter;
uint8_t adc_conv_flag;

static void DIN_Set(void) 
{
	HAL_GPIO_WritePin(DIN_GPIO_Port, DIN_Pin, GPIO_PIN_SET);
}

static void Convst_Set(uint16_t pulse) 
{
	WRITE_REG(TIM15->CCR2, pulse);
}

static void Convst_Reset(void) 
{
	WRITE_REG(TIM15->CCR2, 0);
}


void Mean_filter(void)
{
	arm_mean_q31(adc_rawdata,ADC_SAMPLE_SIZE,&adc_mean[0]);
}

/**
  * @brief  获得ADS8864的采样结果
  * @note   将此函数放入stm32f3xx_it.c文件中的 （HAL库）
  *         void EXTI2_TSC_IRQHandler(void)
  * @retval none
  */
void ADS8864_ReadValue(void)
{
	if(__HAL_GPIO_EXTI_GET_IT(IRQ_Pin) != RESET)
  {
		CLEAR_BIT(TIM15->CR1, TIM_CR1_CEN);	/* 关闭定时器计数 */
    	__HAL_GPIO_EXTI_CLEAR_IT(IRQ_Pin);//清除中断标志
		CLEAR_BIT(EXTI->FTSR, IRQ_Pin);//关闭下降沿触发
		HAL_NVIC_DisableIRQ(IRQ_EXTI_IRQn);//关闭中断
		
		if( HAL_SPI_Receive(&ADS8864_SPI_Port, adc_data, 2, 0x0F) == HAL_OK)
		{

			adc_rawdata[adc_counter] = ((adc_data[0] << 8) | adc_data[1]) & 0xFFFFFFFF; //补符号位
			
			adc_counter++;
			
			if (adc_counter == (ADC_SAMPLE_SIZE) )
			{
				adc_counter = 0;
			}
			SET_BIT(EXTI->FTSR, IRQ_Pin);//开启下降沿触发
			HAL_NVIC_EnableIRQ(IRQ_EXTI_IRQn);//开启中断
			SET_BIT(TIM15->CR1, TIM_CR1_CEN);   /* 恢复定时器计数 */
		}
	}
}

/**
  * @brief  ADS8864初始化函数
  * @note   采用三线制SPI，片选信号脉冲宽度695ns
  * @retval none
  */
void ads8864_Init(void)
{
	DIN_Set();
	//STM32获取一次ADS8864的ADC码值频率理论为66khz
	Convst_Set(28);
	
	//延时20ms
	HAL_Delay(20);
}

