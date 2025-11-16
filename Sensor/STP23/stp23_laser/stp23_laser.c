/**
 * @file    stp23_laser.c
 * @author  Deadline039
 * @brief   STP23 激光接收
 * @version 1.0
 * @date    2025-05-02
 */

#include "stp23_laser.h"

/* 激光数据, 单位: mm */
float g_stp23_laser_data[4];

/* 通信接口, SPI 或者 DMA */
static void *comm_if_handle;
static stp23_comm_interface_t comm_interface;

#define CS_ON()                                                                \
    HAL_GPIO_WritePin(CSP_GPIO_PORT(STP23_SPI_CS_PORT), STP23_SPI_CS_PIN,      \
                      GPIO_PIN_RESET)
#define CS_OFF()                                                               \
    HAL_GPIO_WritePin(CSP_GPIO_PORT(STP23_SPI_CS_PORT), STP23_SPI_CS_PIN,      \
                      GPIO_PIN_SET)

/**
 * @brief STP23 激光初始化
 * 
 * @param handle 句柄. I2C 或 SPI 句柄
 * @param interface 接口类型, I2C 或 SPI
 */
void stp23_init(void *handle, stp23_comm_interface_t interface) {
    if (interface > STP23_COM_SPI) {
        return;
    }

    comm_if_handle = handle;
    comm_interface = interface;
    if (comm_interface == STP23_COM_SPI) {
        GPIO_InitTypeDef gpio_init_struct = {.Mode = GPIO_PULLUP,
                                             .Pin = STP23_SPI_CS_PIN,
                                             .Speed = GPIO_SPEED_FREQ_HIGH,
                                             .Pull = GPIO_PULLUP};
        CSP_GPIO_CLK_ENABLE(STP23_SPI_CS_PORT);
        HAL_GPIO_Init(CSP_GPIO_PORT(STP23_SPI_CS_PORT), &gpio_init_struct);
        CS_OFF();
    }
}

/**
 * @brief STP23 获取数据
 * 
 * @return 获取到的数据指针
 * @note 获取成功后会更新`g_stp23_laser_data`全局变量, 返回的也是这个变量的地址
 */
float *stp23_get_data(void) {
    if (comm_if_handle == NULL) {
        return g_stp23_laser_data;
    }

    switch (comm_interface) {
        case STP23_COM_I2C: {
            HAL_I2C_Master_Receive(
                (I2C_HandleTypeDef *)comm_if_handle, STP23_I2C_ADDRESS,
                (uint8_t *)g_stp23_laser_data, sizeof(g_stp23_laser_data),
                STP23_OPERATE_TIMEOUT);
        } break;

        case STP23_COM_SPI: {
            CS_ON();
            HAL_SPI_Receive((SPI_HandleTypeDef *)comm_if_handle,
                            (uint8_t *)g_stp23_laser_data,
                            sizeof(g_stp23_laser_data), STP23_OPERATE_TIMEOUT);
            CS_OFF();
        } break;

        default:
            break;
    }

    return g_stp23_laser_data;
}