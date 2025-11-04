/**
 * @file    at24cxx.h
 * @author  Deadline039
 * @brief   AT24Cxx 系列芯片驱动
 * @version 1.1
 * @date    2024-09-03
 *****************************************************************************
 * A0 A1 A2 引脚电平, 用于定义地址.
 *      芯片型号                    器件地址格式 (8bit)
 *  AT24C01/02/32/64        1  0  1  0  A2  A1  A0  R/W
 *      24C04,              1  0  1  0  A2  A1  a8  R/W
 *      24C08,              1  0  1  0  A2  a9  a8  R/W
 *      24C16,              1  0  1  0  a10 a9  a8  R/W
 *    R/W      : 读 / 写控制位 0, 表示写；1, 表示读；
 *    A0/A1/A2 : 对应器件的 1,2,3 引脚, 如果芯片对应的引脚位置为数据地址, 则无用
 *    a8/a9/a10: 对应存储整列的高位地址,
 * 11bit 地址最多可以表示 2048 个位置, 可以寻址 24C16 及以内的型号
 * 对于 AT24C128/256, A2 必须为 0
 *****************************************************************************
 * Change Logs:
 * Date         Version     Author      Notes
 * 2024-09-03   1.0         Deadline039 第一次发布
 * 2025-01-26   1.1         Deadline039 支持多设备
 */

#ifndef __AT24CXX_H
#define __AT24CXX_H

#include <CSP_Config.h>

/**
 * @brief AT24CXX 型号定义, 值为最大容量 (byte)
 */
typedef enum {
    AT24C01 = 0x007FU,
    AT24C02 = 0x00FFU,
    AT24C04 = 0x01FFU,
    AT24C08 = 0x03FFU,
    AT24C16 = 0x07FFU,
    AT24C32 = 0x0FFFU,
    AT24C64 = 0x1FFFU,
    AT24C128 = 0x3FFFU,
    AT24C256 = 0x7FFFU
} at24cxx_model_t;

/**
 * @brief AT24CXX 地址枚举定义
 */
typedef enum {
    AT24CXX_ADDRESS_A000 = 0, /**< A2A1A0 000 */
    AT24CXX_ADDRESS_A001 = 1, /**< A2A1A0 001 */
    AT24CXX_ADDRESS_A010 = 2, /**< A2A1A0 010 */
    AT24CXX_ADDRESS_A011 = 3, /**< A2A1A0 011 */
    AT24CXX_ADDRESS_A100 = 4, /**< A2A1A0 100 */
    AT24CXX_ADDRESS_A101 = 5, /**< A2A1A0 101 */
    AT24CXX_ADDRESS_A110 = 6, /**< A2A1A0 110 */
    AT24CXX_ADDRESS_A111 = 7, /**< A2A1A0 111 */
} at24cxx_address_t;

/**
 * @brief 芯片操作结果
 */
typedef enum {
    AT24CXX_OK,   /*!< 操作成功 */
    AT24CXX_ERROR /*!< 操作出错 */
} at24cxx_result_t;

/**
 * @brief AT24CXX 句柄定义
 */
typedef struct {
    I2C_HandleTypeDef *hi2c; /*!< I2C 句柄定义 */
    at24cxx_model_t model;   /*!< 型号 (值为容量) */
    uint8_t address;         /*!< I2C 器件地址 */
} at24cxx_handle_t;

at24cxx_result_t at24cxx_init(at24cxx_handle_t *at24cxx,
                              I2C_HandleTypeDef *hi2c, at24cxx_model_t model,
                              at24cxx_address_t address);
at24cxx_result_t at24cxx_deinit(at24cxx_handle_t *at24cxx);

uint8_t at24cxx_read_byte(at24cxx_handle_t *at24cxx, uint16_t address);
at24cxx_result_t at24cxx_write_byte(at24cxx_handle_t *at24cxx, uint16_t address,
                                    const uint8_t byte);

at24cxx_result_t at24cxx_read(at24cxx_handle_t *at24cxx, uint16_t address,
                              uint8_t *data_buf, uint16_t data_len);
at24cxx_result_t at24cxx_write(at24cxx_handle_t *at24cxx, uint16_t address,
                               const uint8_t *data_buf, uint16_t data_len);

#endif /* __AT24CXX_H */
