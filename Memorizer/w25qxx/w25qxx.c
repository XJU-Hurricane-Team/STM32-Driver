/**
 * @file    w25qxx.c
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

#include "w25qxx.h"

#include "core/core_delay.h"
#include <stdlib.h>
#include <string.h>

#define W25QXX_MANUFACTURER_NORMEM      0x52
#define W25QXX_MANUFACTURER_BYTE        0x68
#define W25QXX_MANUFACTURER_GIGADEVICE  0xC8
#define W25QXX_MANUFACTURER_WINBOND     0xEF

#define W25QXX_DUMMY_BYTE               0xA5
#define W25QXX_GET_ID                   0x90
#define W25QXX_SET_READ_PARAM           0xC0
#define W25QXX_READ_DATA                0x03
#define W25QXX_FAST_READ                0x0B
#define W25QXX_WRITE_ENABLE             0x06
#define W25QXX_PAGE_PROGRAM             0x02
#define W25QXX_SECTOR_ERASE             0x20
#define W25QXX_CHIP_ERASE               0xc7
#define W25QXX_ENABLE_4BYTE_ADDR        0xb7
#define W25QXX_RELEASE_POWER_DOWN       0xab
#define W25QXX_POWER_DOWN               0xb9

#define W25QXX_SECTOR_ERASE_4B          0x21
#define W25QXX_PAGE_PROGRAM_QUAD_INP_4B 0x34
#define W25QXX_ENTER_QSPI_MODE          0x38
#define W25QXX_EXIT_QSPI_MODE           0xFF
#define W25QXX_ENABLE_RESET             0x66
#define W25QXX_READ_REGISTER_1          0x05
#define W25QXX_READ_REGISTER_2          0x35
#define W25QXX_READ_REGISTER_3          0x15

#define W25QXX_WRITE_REGISTER_1         0x01
#define W25QXX_WRITE_REGISTER_2         0x31
#define W25QXX_WRITE_REGISTER_3         0x11

#if W25QXX_USE_SPI

/**
 * @brief 启用片选信号 (CS 引脚拉低)
 *
 * @param w25qxx W25Qxx 句柄
 */
#define cs_on(w25qxx)                                                          \
    do {                                                                       \
        HAL_GPIO_WritePin(w25qxx->cs_port, w25qxx->cs_pin, GPIO_PIN_RESET);    \
    } while (0)

/**
 * @brief 启用片选信号 (CS 引脚拉高)
 *
 * @param w25qxx W25Qxx 句柄
 */
#define cs_off(w25qxx)                                                         \
    do {                                                                       \
        HAL_GPIO_WritePin(w25qxx->cs_port, w25qxx->cs_pin, GPIO_PIN_SET);      \
    } while (0)

/**
 * @brief 往 W25QXX 芯片发送数据
 *
 * @param w25qxx W25QXX 句柄
 * @param buf 发送的数据
 * @param len 发送数据长度 (字节)
 * @return 操作状态
 */
static w25qxx_result_t w25qxx_spi_transmit(w25qxx_handle_t *w25qxx,
                                           const uint8_t *buf, uint32_t len) {
    HAL_StatusTypeDef res = HAL_ERROR;

    if (w25qxx->handle.hspi->hdmatx != NULL) {
        res = HAL_SPI_Transmit_DMA(w25qxx->handle.hspi, buf, len);
    } else {
        res = HAL_SPI_Transmit(w25qxx->handle.hspi, buf, len, 1000);
    }

    if (res == HAL_OK) {
        return W25QXX_OK;
    }

    return W25QXX_ERROR;
}

/**
 * @brief 从 W25QXX 芯片接收数据
 *
 * @param w25qxx W25QXX 句柄
 * @param[out] buf 接收数据缓冲区
 * @param len 要接收的数据长度 (字节)
 * @return 操作状态
 */
static w25qxx_result_t w25qxx_spi_receive(w25qxx_handle_t *w25qxx, uint8_t *buf,
                                          uint32_t len) {
    HAL_StatusTypeDef res = HAL_ERROR;

    if (w25qxx->handle.hspi->hdmarx != NULL) {
        res = HAL_SPI_Receive_DMA(w25qxx->handle.hspi, buf, len);
    } else {
        res = HAL_SPI_Receive(w25qxx->handle.hspi, buf, len, 1000);
    }

    if (res == HAL_OK) {
        return W25QXX_OK;
    }

    return W25QXX_ERROR;
}

/**
 * @brief W25QXX 发送地址
 *
 * @param w25qxx W25QXX 句柄
 * @param address 地址
 */
static void w25qxx_send_addr(w25qxx_handle_t *w25qxx, uint32_t address) {
    uint8_t buf;
    if (w25qxx->block_count >= 256) {
        buf = ((uint8_t)(address >> 24));
        w25qxx_spi_transmit(w25qxx, &buf, 1);
    }
    buf = ((uint8_t)(address >> 16));
    w25qxx_spi_transmit(w25qxx, &buf, 1);
    buf = ((uint8_t)(address >> 8));
    w25qxx_spi_transmit(w25qxx, &buf, 1);
    buf = ((uint8_t)(address & 0xFF));
    w25qxx_spi_transmit(w25qxx, &buf, 1);
}

#endif /* W25QXX_USE_SPI */

#if W25QXX_USE_QSPI

/**
 * @brief 设置 W25QXX SPI模式
 * 
 * @param w25qxx W25QXX 句柄
 * @param enable_qspi 是否启用 QSPI
 * @return 操作状态
 */
w25qxx_result_t w25qxx_set_qspi_mode(w25qxx_handle_t *w25qxx,
                                     bool enable_qspi) {
    uint8_t res;

    if (enable_qspi) {
        res = qspi_send_cmd(W25QXX_ENTER_QSPI_MODE, 0,
                            (0 << 6) | (0 << 4) | (0 << 2) | (1 << 0), 0);
    } else {
        res = qspi_send_cmd(W25QXX_EXIT_QSPI_MODE, 0,
                            (0 << 6) | (0 << 4) | (0 << 2) | (3 << 0), 0);
    }

    if (res) {
        return W25QXX_ERROR;
    }

    w25qxx->enable_qspi = enable_qspi;
    return W25QXX_OK;
}

#endif /* W25QXX_USE_QSPI */

/**
 * @brief 初始化 W25Qxx 芯片
 *
 * @param w25qxx W25QXX 句柄
 * @param use_qspi 是否使用 QSPI (需要预先打开`W25QXX_USE_QSPI`宏定义)
 * @return 是否初始化成功
 * @note 需要预先初始化 SPI/QSPI. 函数会初始化片选引脚, 但是不会开启片选引脚时钟
 */
w25qxx_result_t w25qxx_init(w25qxx_handle_t *w25qxx, bool use_qspi) {
    uint8_t tmp;
    w25qxx_result_t result = W25QXX_OK;
    uint32_t id;

    if (w25qxx == NULL) {
        return W25QXX_ERROR;
    }

    w25qxx->use_qspi = use_qspi;

    if (w25qxx->use_qspi) {
#if W25QXX_USE_QSPI
        w25qxx_set_qspi_mode(w25qxx, false);
        w25qxx_set_qspi_mode(w25qxx, true);
#endif /* W25QXX_USE_QSPI */
    } else {
#if W25QXX_USE_SPI
        GPIO_InitTypeDef gpio_init_stuct = {.Pin = w25qxx->cs_pin,
                                            .Mode = GPIO_MODE_OUTPUT_PP,
                                            .Pull = GPIO_PULLUP,
                                            .Speed = GPIO_SPEED_FREQ_HIGH};
        HAL_GPIO_Init(w25qxx->cs_port, &gpio_init_stuct);
#endif /* W25QXX_USE_SPI */
    }

    id = w25qxx_read_id(w25qxx);

    if (id == 0) {
        /* 在低功耗模式下, 退出低功耗再读取一次 ID  */
        w25qxx_release_power_down(w25qxx);
        id = w25qxx_read_id(w25qxx);
    }

    if (id) {
        w25qxx->manufacturer_id = (uint8_t)(id >> 8);
        w25qxx->device_id = (uint16_t)(id & 0xFF);

        switch (w25qxx->device_id) {
            case 0x13:
                w25qxx->block_count = 8;
                break;

            case 0x14:
                w25qxx->block_count = 16;
                break;

            case 0x15:
                w25qxx->block_count = 32;
                break;

            case 0x16:
                w25qxx->block_count = 64;
                break;

            case 0x17:
                w25qxx->block_count = 128;
                break;

            case 0x18:
                w25qxx->block_count = 256;
                break;

            case 0x19:
                w25qxx->block_count = 512;
                break;

            case 0x20:
                w25qxx->block_count = 1024;
                break;

            case 0x21:
                w25qxx->block_count = 2048;
                break;

            default:
                result = W25QXX_ERROR;
                break;
        }
    } else {
        result = W25QXX_ERROR;
    }

    w25qxx->buf = (uint8_t *)CSP_MALLOC(4096);

    if (w25qxx->buf == NULL) {
        result = W25QXX_ERROR;
    }

    if (result == W25QXX_ERROR) {
        /* Zero the handle so it is clear initialization failed! */
        memset(w25qxx, 0, sizeof(w25qxx_handle_t));
        if (w25qxx->buf != NULL) {
            CSP_FREE(w25qxx->buf);
        }
        return result;
    }

    if (w25qxx->block_count >= 256) {
        /* Enable 4 byte address mode. */
        w25qxx_write_enable(w25qxx);
        tmp = w25qxx_get_status(w25qxx, W25QXX_READ_REGISTER_3);
        if ((tmp & 0x01) == 0) {
            w25qxx_write_enable(w25qxx);
            w25qxx_set_status(w25qxx, W25QXX_WRITE_REGISTER_3,
                              W25QXX_ENABLE_4BYTE_ADDR);
        }
        if (w25qxx->use_qspi) {
#if W25QXX_USE_QSPI
            /* 设置 QSPI 读参数指令 */
            qspi_send_cmd(W25QXX_SET_READ_PARAM, 0,
                          (3 << 6) | (0 << 4) | (0 << 2) | (3 << 0), 0);
            /* 设置 P4 & P5 = 11, 8 个 dummy clock, 104 M */
            tmp = 3 << 4;
            qspi_transmit(&tmp, 1);
#endif /* W25QXX_USE_QSPI */
        }
    }

    return result;
}

/**
 * @brief 反初始化 W25QXX
 *
 * @param w25qxx W25QXX 句柄
 * @return 操作状态
 * @note 该函数会反初始化片选引脚为 RESET 状态.
 */
w25qxx_result_t w25qxx_deinit(w25qxx_handle_t *w25qxx) {
    if (w25qxx == NULL) {
        return W25QXX_ERROR;
    }

    if (w25qxx->use_qspi != true) {
#if W25QXX_USE_SPI
        HAL_GPIO_DeInit(w25qxx->cs_port, w25qxx->cs_pin);
#endif /* W25QXX_USE_SPI */
    }

    if (w25qxx->buf != NULL) {
        CSP_FREE(w25qxx->buf);
    }

    memset(w25qxx, 0, sizeof(w25qxx_handle_t));
    return W25QXX_OK;
}

/**
 * @brief 读取芯片 ID
 *
 * @param w25qxx W25QXX 句柄
 * @return 制造商和设备 ID
 */
uint32_t w25qxx_read_id(w25qxx_handle_t *w25qxx) {
    uint32_t ret = 0;
    uint8_t buf[4];

    if (w25qxx == NULL) {
        return W25QXX_ERROR;
    }

    if (w25qxx->use_qspi) {
#if W25QXX_USE_QSPI
        if (w25qxx->enable_qspi) {
            qspi_send_cmd(W25QXX_GET_ID, 0,
                          (3 << 6) | (2 << 4) | (3 << 2) | (3 << 0), 0);
        } else {
            qspi_send_cmd(W25QXX_GET_ID, 0,
                          (1 << 6) | (2 << 4) | (1 << 2) | (1 << 0), 0);
        }
        qspi_receive(buf, 2);
#endif /* W25QXX_USE_QSPI */
    } else {
#if W25QXX_USE_SPI
        buf[0] = W25QXX_GET_ID;
        buf[1] = 0x00;
        buf[2] = 0x00;
        buf[3] = 0x00;

        cs_on(w25qxx);
        w25qxx_spi_transmit(w25qxx, buf, 4);
        w25qxx_spi_receive(w25qxx, buf, 2);
        cs_off(w25qxx);
#endif /* W25QXX_USE_SPI */
    }

    ret = (buf[0] << 8) | buf[1];
    return ret;
}

/**
 * @brief 获取状态寄存器的值
 *
 * @param w25qxx W25QXX 句柄
 * @param reg 要读取的寄存器
 * @return 当前状态寄存器的值
 */
uint8_t w25qxx_get_status(w25qxx_handle_t *w25qxx, uint8_t reg) {
    uint8_t ret = 0;
    uint8_t buf = reg;

    if (w25qxx == NULL) {
        return W25QXX_ERROR;
    }

    if (w25qxx->use_qspi) {
#if W25QXX_USE_QSPI
        if (w25qxx->enable_qspi) {
            qspi_send_cmd(reg, 0, (3 << 6) | (0 << 4) | (0 << 2) | (3 << 0), 0);

        } else {
            qspi_send_cmd(reg, 0, (1 << 6) | (0 << 4) | (0 << 2) | (1 << 0), 0);
        }

        qspi_receive(&buf, 1);
        ret = buf;
#endif /* W25QXX_USE_QSPI */
    } else {
#if W25QXX_USE_SPI
        cs_on(w25qxx);
        w25qxx_spi_transmit(w25qxx, &buf, 1);
        w25qxx_spi_receive(w25qxx, &buf, 1);
        ret = buf;
        cs_off(w25qxx);
#endif /* W25QXX_USE_SPI */
    }

    return ret;
}

/**
 * @brief 设置状态寄存器的值
 *
 * @param w25qxx W25QXX 句柄
 * @param reg 要设置的寄存器
 * @param status 要设置的值
 * @return 操作状态
 */
w25qxx_result_t w25qxx_set_status(w25qxx_handle_t *w25qxx, uint8_t reg,
                                  uint8_t status) {
    w25qxx_result_t ret = W25QXX_ERROR;
    uint8_t buf[2];

    if (w25qxx == NULL) {
        return W25QXX_ERROR;
    }

    if (w25qxx->use_qspi) {
#if W25QXX_USE_QSPI
        if (w25qxx->enable_qspi) {
            qspi_send_cmd(reg, 0, (3 << 6) | (0 << 4) | (0 << 2) | (3 << 0), 0);

        } else {
            qspi_send_cmd(reg, 0, (1 << 6) | (0 << 4) | (0 << 2) | (1 << 0), 0);
        }

        ret = qspi_transmit(&status, 1);
#endif /* W25QXX_USE_QSPI */
    } else {
#if W25QXX_USE_SPI
        buf[0] = reg;
        buf[1] = status;
        cs_on(w25qxx);
        ret = w25qxx_spi_transmit(w25qxx, buf, 2);
        cs_off(w25qxx);
#endif /* W25QXX_USE_SPI */
    }
    return ret;
}

/**
 * @brief 允许芯片写入
 *
 * @param w25qxx W25QXX 句柄
 * @return 操作状态
 */
w25qxx_result_t w25qxx_write_enable(w25qxx_handle_t *w25qxx) {
    w25qxx_result_t ret = W25QXX_ERROR;
    uint8_t buf[1];

    if (w25qxx == NULL) {
        return W25QXX_ERROR;
    }
    if (w25qxx->use_qspi) {
#if W25QXX_USE_QSPI
        if (w25qxx->enable_qspi) {
            ret = qspi_send_cmd(W25QXX_WRITE_ENABLE, 0,
                                (0 << 6) | (0 << 4) | (0 << 2) | (3 << 0), 0);
        } else {
            ret = qspi_send_cmd(W25QXX_WRITE_ENABLE, 0,
                                (0 << 6) | (0 << 4) | (0 << 2) | (1 << 0), 0);
        }
#endif /* W25QXX_USE_QSPI */
    } else {
#if W25QXX_USE_SPI
        cs_on(w25qxx);
        buf[0] = W25QXX_WRITE_ENABLE;
        ret = w25qxx_spi_transmit(w25qxx, buf, 1);
        cs_off(w25qxx);
#endif /* W25QXX_USE_SPI */
    }
    return ret;
}

/**
 * @brief 等待 W25QXX 就绪
 *
 * @param w25qxx W25QXX 句柄
 * @param timeout 超时时间
 * @return 状态
 * @retval - `W25QXX_OK`:      就绪
 * @retval - `W25QXX_TIMEOUT`: 超时
 */
static w25qxx_result_t w25qxx_wait_for_ready(w25qxx_handle_t *w25qxx,
                                             uint32_t timeout) {
    w25qxx_result_t ret = W25QXX_OK;
    uint32_t start_time = HAL_GetTick();
    while (HAL_GetTick() - start_time <= timeout) {
        if ((w25qxx_get_status(w25qxx, W25QXX_READ_REGISTER_1) & 0x01) !=
            0x01) {
            break;
        }
    }

    if (HAL_GetTick() - start_time >= timeout) {
        ret = W25QXX_TIMEOUT;
    }

    return ret;
}

/**
 * @brief 读取 W25QXX
 *
 * @param w25qxx W25QXX 句柄
 * @param address 地址
 * @param[out] buf 缓冲区
 * @param len 读取长度
 * @return 操作结果
 */
w25qxx_result_t w25qxx_read(w25qxx_handle_t *w25qxx, uint32_t address,
                            uint8_t *buf, uint32_t len) {
    uint8_t cmd = W25QXX_READ_DATA;
    w25qxx_result_t ret;

    if (w25qxx == NULL) {
        return W25QXX_ERROR;
    }

    if (w25qxx->use_qspi) {
#if W25QXX_USE_QSPI
        /* QSPI 快速读数据, 4线传输数据_32位地址_4线传输地址_4线传输指令, 8空周期 */
        ret = qspi_send_cmd(W25QXX_FAST_READ, address,
                            (3 << 6) | (3 << 4) | (3 << 2) | (3 << 0), 8);
        if (ret) {
            return W25QXX_ERROR;
        }

        ret = qspi_receive(buf, len);
#endif /* W25QXX_USE_QSPI */
    } else {
#if W25QXX_USE_SPI
        cs_on(w25qxx);
        w25qxx_spi_transmit(w25qxx, &cmd, 1);
        w25qxx_send_addr(w25qxx, address);
        ret = w25qxx_spi_receive(w25qxx, buf, len);
        cs_off(w25qxx);
#endif /* W25QXX_USE_SPI */
    }

    return ret;
}

/**
 * @brief 在指定地址开始写入最大 256 字节的数据
 *
 * @param w25qxx W25QXX 句柄
 * @param buf 数据缓冲区
 * @param addr 写入的地址
 * @param len 要写入的长度 (最大 256), 不应该超过该页的剩余字节数!
 * @return 操作状态
 */
static w25qxx_result_t w25qxx_write_page(w25qxx_handle_t *w25qxx,
                                         const uint8_t *buf, uint32_t addr,
                                         uint16_t len) {
    uint8_t cmd = W25QXX_PAGE_PROGRAM;

    if (w25qxx_write_enable(w25qxx) != W25QXX_OK) {
        return W25QXX_ERROR;
    }

    if (len > 256) {
        return W25QXX_ERROR;
    }

    if (w25qxx->use_qspi) {
#if W25QXX_USE_QSPI
        if (qspi_send_cmd(W25QXX_PAGE_PROGRAM, addr,
                          (3 << 6) | (3 << 4) | (3 << 2) | (3 << 0), 0)) {
            return W25QXX_ERROR;
        }

        if (qspi_transmit((uint8_t *)buf, len)) {
            return W25QXX_ERROR;
        }
#endif /* W25QXX_USE_QSPI */
    } else {
#if W25QXX_USE_SPI
        cs_on(w25qxx);

        w25qxx_spi_transmit(w25qxx, &cmd, 1);
        w25qxx_send_addr(w25qxx, addr);

        if (w25qxx_spi_transmit(w25qxx, buf, len) != W25QXX_OK) {
            return W25QXX_ERROR;
        }

        cs_off(w25qxx);
#endif /* W25QXX_USE_SPI */
    }

    if (w25qxx_wait_for_ready(w25qxx, 1000) != W25QXX_OK) {
        return W25QXX_TIMEOUT;
    }

    return W25QXX_OK;
}

/**
 * @brief 无检验写 FLASH
 *
 * @param w25qxx W25QXX 句柄
 * @param buf 数据缓冲区
 * @param addr 写入的地址
 * @param len 要写入的长度 (最大 256), 不应该超过该页的剩余字节数!
 * @return 操作状态
 * @note 必须确保所写的地址范围内的数据全部为 0xFF, 否则在非 0xFF
 *       处写入的数据将失败! 具有自动换页功能
 *       在指定地址开始写入指定长度的数据, 但是要确保地址不越界!
 */
static w25qxx_result_t w25qxx_write_no_check(w25qxx_handle_t *w25qxx,
                                             const uint8_t *buf, uint32_t addr,
                                             uint16_t len) {
    uint16_t page_remain = 256 - addr % 256; /* 单页剩余的字节 */

    if (len <= page_remain) {
        /* 不大于 256 个字节 */
        page_remain = len;
    }

    while (1) {
        /**
         * 当写入字节比页内剩余地址还少的时候, 一次性写完
         * 当写入直接比页内剩余地址还多的时候, 先写完整个页内剩余地址,
         * 然后根据剩余长度进行不同处理
         */
        if (w25qxx_write_page(w25qxx, buf, addr, page_remain) != W25QXX_OK) {
            return W25QXX_ERROR;
        }

        if (len == page_remain) {
            /* 写入完毕 */
            break;
        }

        /* pbuf指针地址偏移, 前面已经写了 page_remain 字节 */
        buf += page_remain;
        /* 写地址偏移, 前面已经写了 page_remain 字节 */
        addr += page_remain;
        /* 写入总长度减去已经写入了的字节数 */
        len -= page_remain;

        if (len > 256) {
            /* 剩余数据还大于一页, 可以一次写一页 */
            page_remain = 256;
        } else {
            /* 剩余数据小于一页,可以一次写完 */
            page_remain = len; /* 不够256个字节了 */
        }
    }
    return W25QXX_OK;
}

/**
 * @brief 写入 W25QXX
 *
 * @param w25qxx W25QXX 句柄
 * @param address 地址
 * @param buf 写入的数据
 * @param len 写入长度
 * @return 操作结果
 */
w25qxx_result_t w25qxx_write(w25qxx_handle_t *w25qxx, uint32_t address,
                             const uint8_t *buf, uint32_t len) {
    uint32_t pos;
    uint16_t offset;
    uint16_t remain;

    uint16_t i;

    if (w25qxx == NULL) {
        return W25QXX_ERROR;
    }

    pos = address / 4096;    /* 扇区地址 */
    offset = address % 4096; /* 在扇区内的偏移 */
    remain = 4096 - offset;  /* 扇区剩余空间大小 */

    if (len <= remain) {
        /* 不大于4096个字节 */
        remain = len;
    }

    while (1) {
        /* 读出整个扇区的内容 */
        if (w25qxx_read(w25qxx, pos * 4096, w25qxx->buf, 4096) != W25QXX_OK) {
            return W25QXX_ERROR;
        }

        for (i = 0; i < remain; i++) {
            /* 校验数据, 写入区域必须全是 0xFF */
            if (w25qxx->buf[offset + i] != 0xFF) {
                break;
            }
        }

        if (i < remain) {
            /* 需要擦除 */
            if (w25qxx_erase(w25qxx, pos) != W25QXX_OK) {
                /* 擦除这个扇区 */
                return W25QXX_ERROR;
            }

            for (i = 0; i < remain; ++i) {
                /* 复制要写入的数据 */
                w25qxx->buf[i + offset] = buf[i];
            }

            if (w25qxx_write_no_check(w25qxx, w25qxx->buf, pos * 4096, 4096) !=
                W25QXX_OK) {
                /* 写入整个扇区 */
                return W25QXX_ERROR;
            }

        } else {
            /* 写已经擦除了的, 直接写入扇区剩余区间. */
            if (w25qxx_write_no_check(w25qxx, buf, address, remain) !=
                W25QXX_OK) {
                return W25QXX_ERROR;
            }
        }

        if (len == remain) {
            break; /* 写入结束了 */
        } else {
            pos++;      /* 扇区地址增1 */
            offset = 0; /* 偏移位置为0 */

            buf += remain;     /* 指针偏移 */
            address += remain; /* 写地址偏移 */
            len -= remain;     /* 字节数递减 */

            if (len > 4096) {
                remain = 4096; /* 下一个扇区还是写不完 */
            } else {
                remain = len; /* 下一个扇区可以写完了 */
            }
        }
    }

    return W25QXX_OK;
}

/**
 * @brief 擦除 W25QXX 扇区
 *
 * @param w25qxx W25QXX 句柄
 * @param address 擦除的扇区地址, 根据实际容量设置
 * @return 操作结果
 * @note 注意是扇区地址, 不是字节地址! 擦除一个扇区至少 150 ms
 */
w25qxx_result_t w25qxx_erase(w25qxx_handle_t *w25qxx, uint32_t address) {
    uint8_t cmd = W25QXX_SECTOR_ERASE;

    if (w25qxx == NULL) {
        return W25QXX_ERROR;
    }

    address *= 4096;
    if (w25qxx_write_enable(w25qxx) != W25QXX_OK) {
        return W25QXX_ERROR;
    }

    if (w25qxx_wait_for_ready(w25qxx, 1000) != W25QXX_OK) {
        return W25QXX_TIMEOUT;
    }

    if (w25qxx->use_qspi) {
#if W25QXX_USE_QSPI
        if (qspi_send_cmd(cmd, address,
                          (0 << 6) | (3 << 4) | (3 << 2) | (3 << 0), 0)) {
            return W25QXX_ERROR;
        }
#endif /* W25QXX_USE_QSPI */
    } else {
#if W25QXX_USE_SPI
        cs_on(w25qxx);
        w25qxx_spi_transmit(w25qxx, &cmd, 1);
        w25qxx_send_addr(w25qxx, address);
        cs_off(w25qxx);
#endif /* W25QXX_USE_SPI */
    }

    if (w25qxx_wait_for_ready(w25qxx, 1000) != W25QXX_OK) {
        return W25QXX_TIMEOUT;
    }

    return W25QXX_OK;
}

/**
 * @brief W25QXX 全片擦除
 *
 * @param w25qxx W25QXX 句柄
 * @return 操作结果
 */
w25qxx_result_t w25qxx_chip_erase(w25qxx_handle_t *w25qxx) {
    uint8_t cmd = W25QXX_CHIP_ERASE;

    if (w25qxx == NULL) {
        return W25QXX_ERROR;
    }

    w25qxx_write_enable(w25qxx);
    if (w25qxx_wait_for_ready(w25qxx, 1000) != W25QXX_OK) {
        return W25QXX_TIMEOUT;
    }

    if (w25qxx->use_qspi) {
#if W25QXX_USE_QSPI
        if (qspi_send_cmd(cmd, 0, (0 << 6) | (0 << 4) | (0 << 2) | (3 << 0),
                          0)) {
            return W25QXX_ERROR;
        }
#endif /* W25QXX_USE_QSPI */
    } else {
#if W25QXX_USE_SPI
        cs_on(w25qxx);
        w25qxx_spi_transmit(w25qxx, &cmd, 1);
        cs_off(w25qxx);
#endif /* W25QXX_USE_SPI */
    }

    w25qxx_wait_for_ready(w25qxx, HAL_MAX_DELAY);

    return W25QXX_OK;
}

/**
 * @brief W25QXX 进入低功耗
 *
 * @param w25qxx W25QXX 句柄
 * @return 操作结果
 */
w25qxx_result_t w25qxx_power_down(w25qxx_handle_t *w25qxx) {
    uint8_t cmd = W25QXX_POWER_DOWN;

    if (w25qxx == NULL) {
        return W25QXX_ERROR;
    }

    if (w25qxx_wait_for_ready(w25qxx, 1000) != W25QXX_OK) {
        return W25QXX_TIMEOUT;
    }

    if (w25qxx->use_qspi) {
#if W25QXX_USE_QSPI
        if (qspi_send_cmd(cmd, 0, (0 << 6) | (0 << 4) | (0 << 2) | (1 << 0),
                          0)) {
            return W25QXX_ERROR;
        }
#endif /* W25QXX_USE_QSPI */
    } else {
#if W25QXX_USE_SPI
        cs_on(w25qxx);
        w25qxx_spi_transmit(w25qxx, &cmd, 1);
        cs_off(w25qxx);
#endif /* W25QXX_USE_SPI */
    }

    return W25QXX_OK;
}

/**
 * @brief W25QXX 退出低功耗
 * 
 * @param w25qxx W25QXX 句柄
 * @return 操作结果
 */
w25qxx_result_t w25qxx_release_power_down(w25qxx_handle_t *w25qxx) {
    uint8_t data[4] = {W25QXX_RELEASE_POWER_DOWN, 0xFF, 0xFF, 0xFF};

    if (w25qxx == NULL) {
        return W25QXX_ERROR;
    }

    if (w25qxx->use_qspi) {
#if W25QXX_USE_QSPI
        if (qspi_send_cmd(W25QXX_RELEASE_POWER_DOWN, 0,
                          (0 << 6) | (0 << 4) | (0 << 2) | (1 << 0), 0)) {
            return W25QXX_ERROR;
        }
#endif /* W25QXX_USE_QSPI */
    } else {
#if W25QXX_USE_SPI
        cs_on(w25qxx);
        w25qxx_spi_transmit(w25qxx, data, 1);
        cs_off(w25qxx);
#endif /* W25QXX_USE_SPI */
    }

    if (w25qxx_wait_for_ready(w25qxx, 1000) != W25QXX_OK) {
        return W25QXX_TIMEOUT;
    }

    return W25QXX_OK;
}