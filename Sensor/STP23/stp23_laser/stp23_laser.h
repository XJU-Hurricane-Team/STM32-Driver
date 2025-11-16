/**
 * @file    stp23_laser.h
 * @author  Deadline039
 * @brief   STP23 激光接收
 * @version 1.0
 * @date    2025-05-02
 */

#ifndef __STP23_LASER_H
#define __STP23_LASER_H

#include <CSP_Config.h>

#define STP23_OPERATE_TIMEOUT 1000

#define STP23_I2C_ADDRESS     0x24

#define STP23_SPI_CS_PORT     B
#define STP23_SPI_CS_PIN      GPIO_PIN_12

extern float g_stp23_laser_data[4];

typedef enum {
    STP23_COM_I2C, /*!< 使用 I2C 通信 */
    STP23_COM_SPI  /*!< 使用 SPI 通信 */
} stp23_comm_interface_t;

void stp23_init(void *handle, stp23_comm_interface_t interface);
float *stp23_get_data(void);

#endif /* __STP23_LASER_H */
