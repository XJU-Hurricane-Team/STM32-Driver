/**
 * @file    at24cxx.c
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

#include "at24cxx.h"

#include <string.h>

/**
 * @brief AT24CXX I2C 读取一字节
 *
 * @param at24cxx 句柄
 * @param addr 地址
 * @return 读到的字节
 */
static uint8_t at24cxx_i2c_read_byte(at24cxx_handle_t *at24cxx, uint16_t addr) {
    uint16_t dev_address;
    uint16_t mem_address;
    uint16_t address_size;
    uint8_t byte;

    if (at24cxx->model > (uint16_t)AT24C16) {
        dev_address = at24cxx->address + ((addr / 65536) << 1);
        mem_address = addr % 65536;
        address_size = I2C_MEMADD_SIZE_16BIT;
    } else {
        dev_address = at24cxx->address + ((addr / 256) << 1);
        mem_address = addr % 256;
        address_size = I2C_MEMADD_SIZE_8BIT;
    }

    HAL_I2C_Mem_Read(at24cxx->hi2c, dev_address, mem_address, address_size,
                     &byte, 1, 1000);

    return byte;
}

/**
 * @brief AT24CXX I2C 写入一字节
 *
 * @param at24cxx 句柄
 * @param addr 地址
 * @param byte 要写入的字节
 * @return 操作状态
 */
static at24cxx_result_t at24cxx_i2c_write_byte(at24cxx_handle_t *at24cxx,
                                               uint16_t addr,
                                               const uint8_t byte) {
    uint16_t dev_address;
    uint16_t mem_address;
    uint16_t address_size;

    if (at24cxx->model > (uint16_t)AT24C16) {
        dev_address = at24cxx->address + ((addr / 65536) << 1);
        mem_address = addr % 65536;
        address_size = I2C_MEMADD_SIZE_16BIT;
    } else {
        dev_address = at24cxx->address + ((addr / 256) << 1);
        mem_address = addr % 256;
        address_size = I2C_MEMADD_SIZE_8BIT;
    }

    if (HAL_I2C_Mem_Write(at24cxx->hi2c, dev_address, mem_address, address_size,
                          (uint8_t *)&byte, 1, 1000) != HAL_OK) {
        return AT24CXX_ERROR;
    }

    /* Wait for the end of the transfer */
    while (HAL_I2C_GetState(at24cxx->hi2c) != HAL_I2C_STATE_READY)
        ;
    /* Check if the Devices is ready for a new operation */
    while (HAL_I2C_IsDeviceReady(at24cxx->hi2c, dev_address, 0xF, 0xF) !=
           HAL_OK)
        ;

    return AT24CXX_OK;
}

/**
 * @brief 初始化 AT24CXX 芯片
 *
 * @param at24cxx 句柄
 * @param hi2c I2C 句柄
 * @param model 型号
 * @param address 器件引脚模式
 * @return 初始化状态
 */
at24cxx_result_t at24cxx_init(at24cxx_handle_t *at24cxx,
                              I2C_HandleTypeDef *hi2c, at24cxx_model_t model,
                              at24cxx_address_t address) {
    if (at24cxx == NULL) {
        return AT24CXX_ERROR;
    }

    if (hi2c == NULL) {
        return AT24CXX_ERROR;
    }

    at24cxx->hi2c = hi2c;
    at24cxx->model = model;
    at24cxx->address = 0xA0;
    at24cxx->address |= address << 1;

    return AT24CXX_OK;
}

/**
 * @brief 反初始化 AT24CXX 芯片
 *
 * @param at24cxx
 * @return 操作状态
 */
at24cxx_result_t at24cxx_deinit(at24cxx_handle_t *at24cxx) {
    if (at24cxx == NULL) {
        return AT24CXX_ERROR;
    }

    memset(at24cxx, 0, sizeof(at24cxx_handle_t));

    return AT24CXX_OK;
}

/**
 * @brief AT24CXX 读一个字节
 *
 * @param at24cxx 句柄
 * @param address 地址
 * @return 读到的字节
 */
uint8_t at24cxx_read_byte(at24cxx_handle_t *at24cxx, uint16_t address) {
    if (at24cxx == NULL) {
        return 0;
    }

    return at24cxx_i2c_read_byte(at24cxx, address);
}

/**
 * @brief AT24CXX 写一个字节
 *
 * @param at24cxx 句柄
 * @param address 地址
 * @param byte 要写入的字节
 * @return 是否写入成功
 */
at24cxx_result_t at24cxx_write_byte(at24cxx_handle_t *at24cxx, uint16_t address,
                                    const uint8_t byte) {
    if (at24cxx == NULL) {
        return AT24CXX_ERROR;
    }

    return at24cxx_i2c_write_byte(at24cxx, address, byte);
}

/**
 * @brief AT24CXX 读取数据
 *
 * @param at24cxx 句柄
 * @param address 地址
 * @param[out] data_buf 数据缓冲区
 * @param data_len 要读取的长度
 * @return 是否读取成功
 */
at24cxx_result_t at24cxx_read(at24cxx_handle_t *at24cxx, uint16_t address,
                              uint8_t *data_buf, uint16_t data_len) {
    if (at24cxx == NULL) {
        return AT24CXX_ERROR;
    }

    if (data_buf == NULL) {
        return AT24CXX_ERROR;
    }

    for (uint32_t i = 0; i < data_len; ++i) {
        data_buf[i] = at24cxx_i2c_read_byte(at24cxx, address + i);
    }

    return AT24CXX_OK;
}

/**
 * @brief AT24CXX 写入数据
 *
 * @param at24cxx 句柄
 * @param address 地址
 * @param data_buf 数据缓冲区
 * @param data_len 要写入的长度
 * @return 是否写入成功
 */
at24cxx_result_t at24cxx_write(at24cxx_handle_t *at24cxx, uint16_t address,
                               const uint8_t *data_buf, uint16_t data_len) {
    if (at24cxx == NULL) {
        return AT24CXX_ERROR;
    }

    if (data_buf == NULL) {
        return AT24CXX_ERROR;
    }

    for (uint32_t i = 0; i < data_len; ++i) {
        if (at24cxx_i2c_write_byte(at24cxx, address + i, data_buf[i]) !=
            AT24CXX_OK) {
            return AT24CXX_ERROR;
        }
    }
    return AT24CXX_OK;
}
