/**
 * @file    iic.h
 * @author  Deadline039
 * @brief   I2C初始化以及驱动代码
 * @version 0.2
 * @date    2023-11-05
 */
#ifndef __IIC_H
#define __IIC_H

#include <CSP_Config.h>

#define IIC_SCL_GPIO_PORT     GPIOB
#define IIC_SCL_GPIO_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()
#define IIC_SCL_GPIO_PIN      GPIO_PIN_10

#define IIC_SDA_GPIO_PORT     GPIOB
#define IIC_SDA_GPIO_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()
#define IIC_SDA_GPIO_PIN      GPIO_PIN_11

#define IIC_SCL(x)                                                             \
    x ? HAL_GPIO_WritePin(IIC_SCL_GPIO_PORT, IIC_SCL_GPIO_PIN, GPIO_PIN_SET)   \
      : HAL_GPIO_WritePin(IIC_SCL_GPIO_PORT, IIC_SCL_GPIO_PIN, GPIO_PIN_RESET)
#define IIC_SDA(x)                                                             \
    x ? HAL_GPIO_WritePin(IIC_SDA_GPIO_PORT, IIC_SDA_GPIO_PIN, GPIO_PIN_SET)   \
      : HAL_GPIO_WritePin(IIC_SDA_GPIO_PORT, IIC_SDA_GPIO_PIN, GPIO_PIN_RESET)

#define IIC_READ_SDA HAL_GPIO_ReadPin(IIC_SDA_GPIO_PORT, IIC_SDA_GPIO_PIN)

#define WAIT_TIME    0 /* IO操作间隔时间, 单位为us */

void iic_init(void);
void iic_start(void);
void iic_stop(void);
void iic_ack(void);
void iic_nack(void);
void iic_wait_ack(void);
void iic_send_byte(uint8_t data);
uint8_t iic_read_byte(uint8_t ack);
void iic_write_commandd(uint8_t cmd);
void iic_write_data(uint8_t data);

#endif /* __IIC_H */