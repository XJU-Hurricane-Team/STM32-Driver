/**
 * @file    w25qxx.h
 * @author  Deadline039
 * @brief   W25QXX 系列芯片驱动
 * @version 1.0
 * @date    2025-01-22
 * @ref     https://github.com/lbthomsen/stm32-w25qxx
 *****************************************************************************
 * Change Logs:
 * Date         Version     Author      Notes
 * 2025-01-22   1.0         Deadline039 第一次发布
 */

#ifndef __W25QXX_H
#define __W25QXX_H

#include <CSP_Config.h>

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* 是否需要使用 QSPI */
#define W25QXX_USE_QSPI             1
/* 是否使用 SPI */
#define W25QXX_USE_SPI              0

#define W25QXX_FAST_READ_QUAD_IO_4B 0xEC

/**
 * @brief 外设操作句柄
 */
typedef union {
#if W25QXX_USE_QSPI
    QSPI_HandleTypeDef *hqspi; /*!< QSPI */
#endif                         /* W25QXX_USE_QSPI */

#if W25QXX_USE_SPI
    SPI_HandleTypeDef *hspi; /*!< SPI */
#endif                       /* W25QXX_USE_SPI */
} w25qxx_spi_handle_t;

/**
 * @brief 芯片句柄
 */
typedef struct {
    w25qxx_spi_handle_t handle; /*!< 外设句柄 */
#if W25QXX_USE_SPI
    GPIO_TypeDef *cs_port; /*!< 片选端口 */
    uint16_t cs_pin;       /*!< 片选引脚 */
#endif                     /* W25QXX_USE_SPI */

#if W25QXX_USE_QSPI
    bool enable_qspi; /*!< 是否启用 QSPI 模式 */
#endif                /* W25QXX_USE_QSPI */

    bool use_qspi; /*!< 是否使用 QSPI */

    uint8_t manufacturer_id; /*!< 制造商 */
    uint16_t device_id;      /*!< 设备 ID */
    uint8_t *buf;            /*!< 缓冲区 */
    uint32_t block_count;    /*!< 芯片块大小 (芯片容量) */
} w25qxx_handle_t;

/**
 * @brief 芯片操作状态
 */
typedef enum {
    W25QXX_OK,     /*!< 操作成功 */
    W25QXX_ERROR,  /*!< 操作出错 */
    W25QXX_TIMEOUT /*!< 操作超时 */
} w25qxx_result_t;

w25qxx_result_t w25qxx_init(w25qxx_handle_t *w25qxx, bool use_qspi);
w25qxx_result_t w25qxx_deinit(w25qxx_handle_t *w25qxx);
uint32_t w25qxx_read_id(w25qxx_handle_t *w25qxx);
uint8_t w25qxx_get_status(w25qxx_handle_t *w25qxx, uint8_t reg);
uint8_t w25qxx_set_status(w25qxx_handle_t *w25qxx, uint8_t reg, uint8_t status);
w25qxx_result_t w25qxx_write_enable(w25qxx_handle_t *w25qxx);

w25qxx_result_t w25qxx_read(w25qxx_handle_t *w25qxx, uint32_t address,
                            uint8_t *buf, uint32_t len);
w25qxx_result_t w25qxx_write(w25qxx_handle_t *w25qxx, uint32_t address,
                             const uint8_t *buf, uint32_t len);
w25qxx_result_t w25qxx_erase(w25qxx_handle_t *w25qxx, uint32_t address);
w25qxx_result_t w25qxx_chip_erase(w25qxx_handle_t *w25qxx);

w25qxx_result_t w25qxx_power_down(w25qxx_handle_t *w25qxx);
w25qxx_result_t w25qxx_release_power_down(w25qxx_handle_t *w25qxx);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __W25QXX_H */
