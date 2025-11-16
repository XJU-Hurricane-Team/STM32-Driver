/**
 * @file myantenna_L1.c
 * @author CV-Engineer-Chen
 * @brief Myantenna_L1激光模块
 * @version 1.0
 * @date 2025-03-31
 * @note 本文件基于MyantennaL1s激光模块协议编写;
 *       参考资料:http://www.imyantenna.com/list-8.html
 *
 * @copyright Copyright (c) 2025
 * 
 */
#include "myantenna_L1.h"
#include "bsp.h"
#include "bcc/bcc.h"
#include "crc/crc.h"
#include <string.h>
#include <stdio.h>
#include <inttypes.h>

/* OK标志位 */
#define __OK_POS 29U
#define OK_MSK   (1U << (__OK_POS))
/* \r标志位 */
#define __CR_POS 30U
#define CR_MSK   (1U << (__CR_POS))
/* \n标志位 */
#define __LF_POS 31U
#define LF_MSK   (1U << (__LF_POS))

/**
 * @brief 处理激光buffer ascii数据
 * 
 * @param laser_handle 激光句柄
 */
static void ascii_data_parse(myantenna_laser_handle_t *laser_handle) {
    if (laser_handle == NULL) {
        return;
    }

    switch (laser_handle->buffer[0]) {
        case 'E':
            sscanf((const char *)laser_handle->buffer, "E=%" SCNu16 "\r\n",
                   &laser_handle->error_type);
            break;
        case 'O':
            if (laser_handle->buffer[1] == 'K') {
                break;
            }
            sscanf((const char *)laser_handle->buffer, "OFFSET=%" SCNi16 "\r\n",
                   &laser_handle->offset);
            break;
        case 'R':
            sscanf((const char *)laser_handle->buffer, "RANGE=%u\r\n",
                   &laser_handle->range);
            break;
        case 'B':
            sscanf((const char *)laser_handle->buffer, "BOUND=%u\r\n",
                   &laser_handle->bound);
            break;
        case 'P':
            sscanf((const char *)laser_handle->buffer,
                   "PROTOCOL=%" SCNu8 "\r\n", &laser_handle->protocol);
            break;
        case 'D':
            if (laser_handle->buffer[1] == 'A') {
                sscanf((const char *)laser_handle->buffer,
                       "DATA=%" SCNu8 "\r\n", &laser_handle->datatype);
                break;
            }
            sscanf((const char *)laser_handle->buffer, "D=%fm\r\n",
                   &laser_handle->distance);
            break;
        case 'A':
            if (laser_handle->buffer[1] == 'U') {
                sscanf((const char *)laser_handle->buffer,
                       "AUTMEAS=%" SCNu8 "\r\n", &laser_handle->autmeas);
            } else {
                sscanf((const char *)laser_handle->buffer,
                       "ADDRESS=%" SCNu8 "\r\n", &laser_handle->address);
            }
            break;
        case 'F':
            sscanf((const char *)laser_handle->buffer,
                   "FREQUENCY=%" SCNu8 "\r\n", &laser_handle->frequence);
            break;
        case 'L':
            break;
        case 'S':
            break;
        default:
            laser_handle->length = 0;
            memset(&laser_handle->buffer, 0, sizeof(laser_handle->buffer));
            break;
    }
}

/**
 * @brief 处理激光buffer modbus数据
 * 
 * @param laser_handle 激光句柄
 */
static void modbus_data_parse(myantenna_laser_handle_t *laser_handle) {
    uint32_t realdata = 0;

    for (uint8_t i = 0; i < laser_handle->length; i++) {
        realdata |=
            (laser_handle->buffer[i] << (8 * (laser_handle->length - 1 - i)));
    }
    laser_handle->distance = realdata / 1000.0f;
}

static void hex_data_parse(myantenna_laser_handle_t *laser_handle) {
    if (laser_handle->buffer[0] == 0x05) {
        /* 停止激光发送 */
        return;
    }
    uint32_t realdata = 0;
    for (uint8_t i = 0; i < 4; i++) {
        realdata |= (laser_handle->buffer[i + 1] << (8 * (3 - i)));
    }
    laser_handle->distance = realdata / 1000.0f;
}

/**
 * @brief 清除激光buffer数据
 * 
 * @param laser_handle 
 */
static void clear_buf(myantenna_laser_handle_t *laser_handle);

/**
 * @brief 处理ascii数据,存入laser的buffer中
 * 
 * @param laser_handle 激光句柄
 * @param rx_data 串口rx缓冲区中的数据
 * @param length 数据长度
 */
static void myantenna_ascii_data_parse(myantenna_laser_handle_t *laser_handle,
                                       uint8_t *rx_data, uint32_t length) {
    uint16_t data_byte = 0; /* 处理数据位 */

    if (laser_handle->protocol != MYANTENNA_ASCII) {
        return;
    }

    for (data_byte = 0; data_byte < length; data_byte++) {
        if ((rx_data[data_byte] == 'O') && (rx_data[data_byte + 1] == 'K')) {
            /* 如果收到了OK,OK标志位置1 */
            SET_BIT(laser_handle->length, OK_MSK);
            if (READ_BIT(laser_handle->length, LF_MSK) == 1) {
                /* 如果已经接收了一个\r\n,再接收一次 */
                CLEAR_BIT(laser_handle->length, CR_MSK);
                CLEAR_BIT(laser_handle->length, LF_MSK);
            }
        } else if (rx_data[data_byte - 1] == '\n' &&
                   (READ_BIT(laser_handle->length, LF_MSK)) &&
                   (READ_BIT(laser_handle->length, CR_MSK)) &&
                   (READ_BIT(laser_handle->length, OK_MSK) != (OK_MSK))) {
            /* 如果\r\n后无OK，直接置标志位 */
            SET_BIT(laser_handle->length, OK_MSK);
        }

        if ((laser_handle->length & (OK_MSK | CR_MSK | LF_MSK)) ==
            ((OK_MSK | CR_MSK | LF_MSK))) {
            /* 已经处理完成数据 */
            laser_handle->length = 0;
            ascii_data_parse(laser_handle);
            return;
        }

        if (READ_BIT(laser_handle->length, CR_MSK)) {
            if (rx_data[data_byte] == '\n') {
                /* 接收到了'\r'后又接收到了'\n',标志位置1 */
                SET_BIT(laser_handle->length, LF_MSK);
            } else {
                /* 接收到了'\r', 但没有接收到'\n', 错误状态, 重新接收 */
                laser_handle->length = 0;
                CLEAR_BIT(laser_handle->length, CR_MSK);
            }
        }

        if (rx_data[data_byte] == '\r') {
            /* 接受到\r */
            SET_BIT(laser_handle->length, CR_MSK);
        }

        /* 一般字符, 放入缓冲区 */
        laser_handle->buffer[laser_handle->length & 0x0000ffff] =
            rx_data[data_byte];
        ++laser_handle->length;

        /* 防止超域 */
        if ((laser_handle->length & 0x0000FFFF) > BUF_LENGTH - 1) {
            clear_buf(laser_handle);
            return;
        }
    }
}

/**
 * @brief 处理modbus数据,存入laser的buffer中
 * 
 * @param laser_handle 激光句柄
 * @param rx_data 串口rx缓冲区的数据
 * @param length 数据长度
 */
static void myantenna_modbus_data_parse(myantenna_laser_handle_t *laser_handle,
                                        uint8_t *rx_data, uint32_t length) {
    if (laser_handle->protocol != MYANTENNA_MODBUS) {
        /* 通信协议错误 */
        return;
    }
    if (rx_data[0] != laser_handle->address) {
        /* address错误 */
        return;
    }
    /* 判定错误帧 */
    if ((rx_data[1] & 0x80) == 0x80) {
        /* 异常码 */
        laser_handle->error_type = rx_data[2];
        return;
    }
    if ((rx_data[3] & 0x80) == 0x80) {
        /* 故障码 */
        laser_handle->error_type = ((rx_data[5] << 2) | (rx_data[6]));
        return;
    }
    /* crc16校验 */
    uint16_t crc_check = 0;
    uint8_t lsb = 0, msb = 0;
    crc_check = crc16(rx_data, length - 2);
    lsb = crc_check & 0xFF;
    msb = (crc_check >> 8) & 0xFF;
    if ((lsb != rx_data[length - 2]) || (msb != rx_data[length - 1])) {
        /* crc16校验出错 */
        laser_handle->error_type = 0x05;
        return;
    }

    /* 数据处理 */
    clear_buf(laser_handle);
    uint8_t data_length = 0;
    for (; data_length < rx_data[2]; data_length++) {
        laser_handle->buffer[laser_handle->length] = rx_data[3 + data_length];
        ++laser_handle->length;
        if ((laser_handle->length > BUF_LENGTH) | (data_length > length)) {
            clear_buf(laser_handle);
            return;
        }
    }

    modbus_data_parse(laser_handle);
}

/**
 * @brief 处理hex数据,存入laser的buffer中
 * 
 * @param laser_handle 激光句柄
 * @param rx_data 串口rx缓冲区的数据
 * @param length 数据长度
 */
static void myatenna_hex_data_parse(myantenna_laser_handle_t *laser_handle,
                                    uint8_t *rx_data, uint32_t length) {
    if (laser_handle->protocol != MYANTENNA_HEX) {
        /* 解析格式出错 */
        return;
    }
    if (length != 8) {
        /* 接收出错 */
        return;
    }
    if ((rx_data[0] != 0xB4) || (rx_data[1] != 0x69)) {
        /* 帧头错 */
        return;
    }
    if ((rx_data[2] & 0x80) == 0x80) {
        /* 高位为1表示故障 */
        uint32_t data = 0;
        for (uint8_t i = 0; i < 4; i++) {
            data |= (rx_data[i] << (8 * (3 - i)));
        }
        laser_handle->error_type = (uint16_t)data;
        return;
    }
    /* BCC异或校验 */
    uint8_t bcc_check = 0;
    bcc_check = bcc8(rx_data, length - 1);
    if (bcc_check != rx_data[length - 1]) {
        return;
    }
    /* 数据处理 */
    clear_buf(laser_handle);
    uint8_t data_length = 0;
    for (; data_length < 5; data_length++) {
        laser_handle->buffer[laser_handle->length] = rx_data[2 + data_length];
        ++laser_handle->length;
    }
    hex_data_parse(laser_handle);
}

/**
 * @brief laser初始化
 * 
 * @param laser_handle Laser激光模块控制句柄
 * @param protocol 通信协议格式(comtool设定)
 *  @arg 'MYANTENNA_ASCII',使用ascii协议
 *  @arg 'MYANTENNA_HEX',使用hex协议
 *  @arg 'MYANTENNA_MODBUS',使用modbus
 * @param address 仅与modbus相关,
 */
uint8_t myantenna_laser_init(myantenna_laser_handle_t *laser_handle,
                             myantenna_protocol_t protocol, uint8_t address,
                             UART_HandleTypeDef *uart_handle) {
    laser_handle->uart_handle = uart_handle;
    laser_handle->address = address;
    laser_handle->protocol = protocol;

    switch (laser_handle->protocol) {
        case MYANTENNA_ASCII:
            laser_handle->myantenna_data_parse = myantenna_ascii_data_parse;
            break;
        case MYANTENNA_MODBUS:
            laser_handle->myantenna_data_parse = myantenna_modbus_data_parse;
            break;
        case MYANTENNA_HEX:
            laser_handle->myantenna_data_parse = myatenna_hex_data_parse;
            break;
        default:
            break;
    }

    return 0;
}

/**
 * @brief 清除缓冲区
 * 
 * @param laser_handle 激光句柄
 * @return * void 
 */
void clear_buf(myantenna_laser_handle_t *laser_handle) {
    memset(laser_handle->buffer, 0, sizeof(laser_handle->buffer));
    laser_handle->length = 0;
}

/**
 * @brief ascii get data 
 * 
 * @param laser_handle 激光句柄
 * @param cmd_type 发送命令类型
 *  @arg 'MYANTENNA_CMD_OFFSET',距离偏移量
 *  @arg 'MYANTENNA_CMD_RANGE',量程
 *  @arg 'MYANTENNA_CMD_BOUND',波特率
 *  @arg 'MYANTENNA_CMD_PROTOCOL',协议类型
 *  @arg 'MYANTENNA_CMD_DATATYPE',输出距离数字格式
 *  @arg 'MYANTENNA_CMD_ADDRESS',从机设备地址
 *  @arg 'MYANTENNA_CMD_FREQUENCY',输出速率
 *  @arg 'MYANTENNA_CMD_AUTMEAS',上电自动测量标识
 * 
 * @return Operational status:
 * @retval - 0:Success.
 * @retval - 1:protocol error.
 */
uint8_t myantenna_get_status(myantenna_laser_handle_t *laser_handle,
                             myantenna_cmd_t cmd_type) {
    if (laser_handle->protocol != MYANTENNA_ASCII) {
        return 1;
    }
    uint8_t cmd[16] = {0};
    sprintf((char *)cmd, "iGET:%d", cmd_type);

    clear_buf(laser_handle);

    HAL_UART_Transmit(laser_handle->uart_handle, cmd, strlen((const char *)cmd),
                      0xFF);
    delay_ms(200);

    return 0;
}

/**
 * @brief ascii设置距离偏移量
 * 
 * @param laser_handle 激光句柄
 * @param offset 偏移量大小
 * @return Operational status:
 * @retval - 0:Success.
 * @retval - 1:protocol error.
 */
uint8_t myantenna_set_offset(myantenna_laser_handle_t *laser_handle,
                             int16_t offset) {
    if (laser_handle->protocol != MYANTENNA_ASCII) {
        return 1;
    }
    uint8_t cmd[32] = {0};
    sprintf((char *)cmd, "iSET:%d,%d", MYANTENNA_CMD_OFFSET, offset);

    clear_buf(laser_handle);
    HAL_UART_Transmit(laser_handle->uart_handle, cmd, strlen((const char *)cmd),
                      0xFF);
    delay_ms(200);
    return 0;
}

/**
 * @brief 设置量程
 * 
 * @param laser_handle 激光句柄
 * @param range 偏移量大小,单位mm
 * @return Operational status:
 * @retval - 0:Success.
 * @retval - 1:protocol error.
 */
uint8_t myantenna_set_range(myantenna_laser_handle_t *laser_handle,
                            int16_t range) {
    if (laser_handle->protocol != MYANTENNA_ASCII) {
        return 1;
    }
    uint8_t cmd[32] = {0};
    sprintf((char *)cmd, "iSET:%d,%d", MYANTENNA_CMD_RANGE, range);

    clear_buf(laser_handle);
    HAL_UART_Transmit(laser_handle->uart_handle, cmd, strlen((const char *)cmd),
                      0xFF);
    delay_ms(200);
    return 0;
}

/**
 * @brief 设置波特率
 * 
 * @param laser_handle 激光句柄
 * @param bound 波特率
 * @return Operational status:
 * @retval - 0:Success.
 * @retval - 1:protocol error.
 */
uint8_t myantenna_set_bound(myantenna_laser_handle_t *laser_handle,
                            int16_t bound) {
    if (laser_handle->protocol != MYANTENNA_ASCII) {
        return 1;
    }
    uint8_t cmd[32] = {0};
    sprintf((char *)cmd, "iSET:%d,%d", MYANTENNA_CMD_BOUND, bound);

    clear_buf(laser_handle);
    HAL_UART_Transmit(laser_handle->uart_handle, cmd, strlen((const char *)cmd),
                      0xFF);
    delay_ms(200);
    return 0;
}

/**
 * @brief 设定laser通信协议
 * 
 * @param laser_handle 
 * @return Operational status:
 * @retval - 0:Success.
 */
uint8_t myantenna_set_protocol(myantenna_laser_handle_t *laser_handle) {
    uint8_t cmd[16] = {0};
    sprintf((char *)cmd, "iSET:%d%d", MYANTENNA_CMD_PROTOCOL,
            laser_handle->protocol);
    HAL_UART_Transmit(laser_handle->uart_handle, cmd, strlen((const char *)cmd),
                      0xFF);
    delay_ms(200);
    return 0;
}

/**
 * @brief 设置距离输出格式
 * 
 * @param laser_handle 激光句柄
 * @param datatype 距离输出格式
 *  @arg 'MYANTENNA_3POINTS',输出小数点后三位
 *  @arg 'MAYNTENNA_4POINTS',输出小数点后四位
 * @return Operational status:
 * @retval - 0:Success.
 * @retval - 1:protocol error.
 */
uint8_t myantenna_set_datatype(myantenna_laser_handle_t *laser_handle,
                               myantenna_datatype_t datatype) {
    if (laser_handle->protocol != MYANTENNA_ASCII) {
        return 1;
    }
    uint8_t cmd[16] = {0};
    sprintf((char *)cmd, "iSET:%d,%d", MYANTENNA_CMD_DATATYPE, datatype);

    clear_buf(laser_handle);
    HAL_UART_Transmit(laser_handle->uart_handle, cmd, strlen((const char *)cmd),
                      0xFF);
    delay_ms(200);
    return 0;
}

/**
 * @brief 设置从机地址
 * 
 * @param laser_handle 激光句柄
 * @param address 距离输出格式
 * @return Operational status:
 * @retval - 0:Success.
 * @retval - 1:protocol error.
 */
uint8_t myantenna_set_address(myantenna_laser_handle_t *laser_handle,
                              uint8_t address) {
    if (laser_handle->protocol != MYANTENNA_ASCII) {
        return 1;
    }
    uint8_t cmd[16] = {0};
    sprintf((char *)cmd, "iSET:%d,%d", MYANTENNA_CMD_ADDRESS, address);

    clear_buf(laser_handle);
    HAL_UART_Transmit(laser_handle->uart_handle, cmd, strlen((const char *)cmd),
                      0xFF);
    delay_ms(200);
    return 0;
}

/**
 * @brief 设置输出速率
 * 
 * @param laser_handle 激光句柄
 * @param frequence 距离输出格式
 *  @arg 'MYANTENNA_10HZ',10赫兹
 *  @arg 'MYANTENNA_20HZ',20赫兹
 * 
 * @return Operational status:
 * @retval - 0:Success.
 * @retval - 1:protocol error.
 */
uint8_t myantenna_set_frequence(myantenna_laser_handle_t *laser_handle,
                                myantenna_frequence_t frequence) {
    if (laser_handle->protocol != MYANTENNA_ASCII) {
        return 1;
    }
    uint8_t cmd[16] = {0};
    sprintf((char *)cmd, "iSET:%d,%d", MYANTENNA_CMD_FREQUENCY, frequence);

    clear_buf(laser_handle);
    HAL_UART_Transmit(laser_handle->uart_handle, cmd, strlen((const char *)cmd),
                      0xFF);
    return 0;
}

/**
 * @brief 设置上电自动测量标识
 * 
 * @param laser_handle 激光句柄
 * @param autmeas 自动测量标识
 *  @arg 'MYANTENNA_NO_AUTO',不自动测量
 *  @arg 'MYANTENNA_AUTO',自动测量
 *  @arg 'MYANTENNA_AUTO_FAST',自动快速测量
 * 
 * @return Operational status:
 * @retval - 0:Success.
 * @retval - 1:protocol error.
 */
uint8_t myantenna_set_autmeas(myantenna_laser_handle_t *laser_handle,
                              myantenna_auto_t autmeas) {
    if (laser_handle->protocol != MYANTENNA_ASCII) {
        return 1;
    }
    uint8_t cmd[16] = {0};
    sprintf((char *)cmd, "iSET:%d,%d", MYANTENNA_CMD_AUTMEAS, autmeas);

    clear_buf(laser_handle);
    HAL_UART_Transmit(laser_handle->uart_handle, cmd, strlen((const char *)cmd),
                      0xFF);
    delay_ms(200);
    return 0;
}

/**
 * @brief 单次测量
 * @param laser_handle 激光句柄
 * 
 * @return Operational status:
 * @retval - 0:Success.
 * @retval - 1:protocol error.
 */
uint8_t myantenna_measuring_single(myantenna_laser_handle_t *laser_handle) {
    if (laser_handle->protocol == MYANTENNA_ASCII) {
        clear_buf(laser_handle);
        HAL_UART_Transmit(laser_handle->uart_handle, (uint8_t *)"iSM", 3, 0xFF);
        return 0;
    } else if (laser_handle->protocol == MYANTENNA_MODBUS) {
        clear_buf(laser_handle);
        uint8_t cmd[8] = {0};
        uint16_t crc = 0;
        int length = 0;
        cmd[length++] = laser_handle->address;
        cmd[length++] = 0x03;
        cmd[length++] = 0x00;
        cmd[length++] = 0x0f;
        cmd[length++] = 0x00;
        cmd[length++] = 0x02;
        crc = crc16(cmd, 6);
        cmd[length++] = crc & 0xFF;
        cmd[length++] = (crc >> 8) & 0xFF;
        HAL_UART_Transmit(laser_handle->uart_handle, cmd, length, 0xFF);
        return 0;
    } else if (laser_handle->protocol == MYANTENNA_HEX) {
        uint8_t cmd[5] = {0xA5, 0x5A, 0x02, 0x00, 0xFD};
        HAL_UART_Transmit(laser_handle->uart_handle, cmd, 5, 0xFF);
        return 0;
    }
    return 1;
}

/**
 * @brief 连续测量
 * 
 * @param laser_handle 激光句柄 
 * @return Operational status:
 * @retval - 0:Success.
 * @retval - 1:protocol error.
 */
uint8_t myantenna_measuring_continue(myantenna_laser_handle_t *laser_handle) {
    if (laser_handle->protocol == MYANTENNA_ASCII) {
        clear_buf(laser_handle);
        HAL_UART_Transmit(laser_handle->uart_handle, (uint8_t *)"iACM", 4,
                          0xFF);
        delay_ms(200);
        return 0;
    } else if (laser_handle->protocol == MYANTENNA_HEX) {
        clear_buf(laser_handle);
        uint8_t cmd[5] = {0xA5, 0x5A, 0x03, 0x00, 0xFC};
        HAL_UART_Transmit(laser_handle->uart_handle, cmd, 5, 0xFF);
    }
}

/**
 * @brief 快速连续测量
 * 
 * @param laser_handle 激光句柄 
 * @return Operational status:
 * @retval - 0:Success.
 * @retval - 1:protocol error.
 */
uint8_t myantenna_measuring_continue_fast(
    myantenna_laser_handle_t *laser_handle) {
    if (laser_handle->protocol == MYANTENNA_ASCII) {
        clear_buf(laser_handle);
        HAL_UART_Transmit(laser_handle->uart_handle, (uint8_t *)"iFACM", 5,
                          0xFF);
        return 0;
    } else if (laser_handle->protocol == MYANTENNA_HEX) {
        uint8_t cmd[5] = {0xA5, 0x5A, 0x04, 0x00, 0xFB};
        HAL_UART_Transmit(laser_handle->uart_handle, cmd, 5, 0xFF);
        return 0;
    }
    return 1;
}

/**
 * @brief 停止测量
 * 
 * @param laser_handle 激光句柄 
 * @return Operational status:
 * @retval - 0:Success.
 * @retval - 1:protocol error.
 */
uint8_t myantenna_measuring_stop(myantenna_laser_handle_t *laser_handle) {
    if (laser_handle->protocol == MYANTENNA_ASCII) {
        clear_buf(laser_handle);
        HAL_UART_Transmit(laser_handle->uart_handle, (uint8_t *)"iHALT", 5,
                          0xFF);
        delay_ms(200);
        return 0;
    } else if (laser_handle->protocol == MYANTENNA_HEX) {
        clear_buf(laser_handle);
        uint8_t cmd[5] = {0xA5, 0x5A, 0x05, 0x00, 0xFA};
        HAL_UART_Transmit(laser_handle->uart_handle, cmd, 5, 0xFF);
        delay_ms(200);
        return 0;
    }
    return 1;
}

/**
 * @brief 设置激光状态:打开or关闭
 * 
 * @param laser_handle 
 * @param statu 
 *  @arg 'MYANTENNA_CLOSE',关闭激光
 *  @arg 'MYANTENNA_OPEN',打开激光
 * @return Operational status:
 * @retval - 0:Success.
 * @retval - 1:protocol error.
 */
uint8_t myantenna_set_statu(myantenna_laser_handle_t *laser_handle,
                            myantenna_state_t statu) {
    if (laser_handle->protocol != MYANTENNA_ASCII) {
        return 1;
    }
    uint8_t cmd[16] = {0};
    sprintf((char *)cmd, "iLD:%d", statu);

    clear_buf(laser_handle);
    HAL_UART_Transmit(laser_handle->uart_handle, cmd, strlen((const char *)cmd),
                      0xFF);
    delay_ms(200);
    return 0;
}
