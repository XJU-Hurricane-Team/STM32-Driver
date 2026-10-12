/**
 * @file    my_tasks.c
 * @author  Register0017
 * @brief   DT35 data processing task.
 * @version 1.1
 * @brief   Improving interrupt handling.
 * @date    2026-10-12
 */
#include "includes.h"
/**
 * @brief   以下为须手动标定的参数
 */
#define TEACH_NEAR_MM    100.0f    /* DT35 near 点真实距离 (mm) */
#define TEACH_FAR_MM     900.0f    /* DT35 far 点真实距离 (mm) */
#define TEACH_NEAR_ADC   2416      /* DT35 near 点平均码值（AdcCalibSample(30000) 实测） */
#define TEACH_FAR_ADC    61789     /* DT35 far 点平均码值（AdcCalibSample(30000) 实测） */
#define USE_Q2  1

#if USE_Q2
#define I_NEAR_MA        3.99f     /* DT35 near 点实测电流 (mA) */
#define I_FAR_MA         19.99f    /* DT35 far  点实测电流 (mA) */
#endif

extern uint8_t adc_data[BUFFER_SIZE];
extern int adc_rawdata[ADC_SAMPLE_SIZE];
extern int adc_FIRdata[ADC_SAMPLE_SIZE];
extern int adc_mean[1];

#if USE_Q2
float GF_T = (I_FAR_MA - I_NEAR_MA) / (float)(TEACH_FAR_ADC - TEACH_NEAR_ADC);  /* 电流与码值的线性关系斜率（增益系数）*/
float Q_2 = 0;                                                                  /* 计算出的电流值（mA） */
#endif
float DIST = 0;                                                                 /* 计算出的距离值（mm） */
static uint8_t TxData[16];

/**
 * @brief   将浮点数转换为字符串并添加帧头和帧尾
 * @param   n: 浮点数
 * @param   tx_data: 输出的字符串数组
 * @param   decimal_places: 小数位数
 * @return  返回生成的字符串长度
 */
uint8_t getFloatNum(float n, uint8_t tx_data[], int decimal_places) {
    char buffer[15];

    snprintf(buffer, sizeof(buffer), "%.*f", decimal_places, n);
    tx_data[0] = 0x73; /* 帧头 'S' */
    int len = strlen(buffer);
    for (int i = 0; i < len; i++) {
        tx_data[i + 1] = (uint8_t)buffer[i];
    }
    tx_data[len + 1] = 0x65; /* 帧尾 'e' */
    return (len + 2);
}

/**
 * @brief   采样一段时间内的平均码值
 * @param   duration_ms: 采样持续时间（毫秒）
 * @return  返回平均码值
 */
int32_t AdcCalibSample(uint32_t duration_ms)
{
    uint32_t t0  = HAL_GetTick();
    int64_t  sum = 0;
    uint32_t cnt = 0;

    while ((HAL_GetTick() - t0) < duration_ms) {
        Mean_filter();
        sum += adc_mean[0];
        cnt++;
        HAL_Delay(1);
    }
    return (cnt > 0) ? (int32_t)(sum / cnt) : 0;
}

void my_task(void) {
    //int32_t sample = 0;

    ads8864_Init();
    HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);
    HAL_Delay(20);

    //sample = AdcCalibSample(30000);
    while (1) {
		Mean_filter();

#if USE_Q2
        Q_2  = I_NEAR_MA + GF_T * (adc_mean[0] - TEACH_NEAR_ADC);
        DIST = TEACH_NEAR_MM + (Q_2 - I_NEAR_MA) * (TEACH_FAR_MM - TEACH_NEAR_MM) / (I_FAR_MA - I_NEAR_MA);
#else
        DIST = TEACH_NEAR_MM + (adc_mean[0] - TEACH_NEAR_ADC) * (TEACH_FAR_MM - TEACH_NEAR_MM) / (TEACH_FAR_ADC - TEACH_NEAR_ADC);
#endif
        uint8_t len = getFloatNum(DIST, TxData, 4);
        /* 发送数据 */
        HAL_UART_Transmit(&huart1, TxData, len, 1000);

        HAL_Delay(2);
    }
}