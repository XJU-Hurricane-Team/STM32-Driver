#ifndef __MYANTENNA_L1_H
#define __MYANTENNA_L1_H

#include "CSP_Config.h"
#define BUF_LENGTH 64
typedef enum __packed {
    MYANTENNA_MODBUS = 0U,
    MYANTENNA_ASCII,
    MYANTENNA_HEX,
} myantenna_protocol_t;

typedef enum __packed {
    MYANTENNA_3POINTS = 0U,
    MAYNTENNA_4POINTS
} myantenna_datatype_t;

typedef enum __packed {
    MYANTENNA_10HZ = 10U,
    MYANTENNA_20HZ = 20U
} myantenna_frequence_t;

typedef enum __packed {
    MYANTENNA_NO_AUTO = 0U,
    MYANTENNA_AUTO,
    MYANTENNA_AUTO_FAST
} myantenna_auto_t;

typedef enum __packed {
    MYANTENNA_CMD_OFFSET = 1U,
    MYANTENNA_CMD_RANGE,
    MYANTENNA_CMD_BOUND,
    MYANTENNA_CMD_PROTOCOL,
    MYANTENNA_CMD_DATATYPE,
    MYANTENNA_CMD_ADDRESS,
    MYANTENNA_CMD_FREQUENCY,
    MYANTENNA_CMD_AUTMEAS
} myantenna_cmd_t;

typedef enum __packed {
    MYANTENNA_CLOSE = 0U,
    MYANTENNA_OPEN
} myantenna_state_t;

enum myantenna_uart_rx_buf_length {
    MYANTENNA_ASCII_UART_RX_BUF_LEN = 16U,
    MYANTENNA_MODBUS_UART_RX_BUF_LEN = 9U,
    MYANTENNA_HEX_UART_RX_BUF_LEN = 8U
};

typedef struct myantenna_laser_handle myantenna_laser_handle_t;

typedef void (*myantenna_data_parse_t)(
    myantenna_laser_handle_t * /*laser_handle */, uint8_t * /* rx_data */,
    uint32_t /* length */);

struct myantenna_laser_handle {
    uint8_t buffer[BUF_LENGTH];      /* 数据接收缓冲区 */
    uint32_t length;                 /* 数据长度 */
    UART_HandleTypeDef *uart_handle; /* 串口 */

    int16_t offset;                  /* 距离偏移量 */
    uint32_t range;                  /* 量程(单位mm) */
    uint32_t bound;                  /* 波特率 */
    myantenna_protocol_t protocol;   /* 通信协议 */
    myantenna_datatype_t datatype;   /* 输出距离格式 */
    uint8_t address;                 /* 从机设备地址 */
    myantenna_frequence_t frequence; /* 输出速率 */
    myantenna_auto_t autmeas;        /* 上电自动测量标识 */

    myantenna_data_parse_t myantenna_data_parse; /* 数据解析函数 */

    float distance;      /* 距离 */
    uint16_t error_type; /* 故障码 */
};

uint8_t myantenna_laser_init(myantenna_laser_handle_t *laser_handle,
                             myantenna_protocol_t protocol, uint8_t address,
                             UART_HandleTypeDef *uart_handle);

uint8_t myantenna_get_status(myantenna_laser_handle_t *laser_handle,
                             myantenna_cmd_t cmd_type);

uint8_t myantenna_set_offset(myantenna_laser_handle_t *laser_handle,
                             int16_t offset);
uint8_t myantenna_set_range(myantenna_laser_handle_t *laser_handle,
                            int16_t range);
uint8_t myantenna_set_bound(myantenna_laser_handle_t *laser_handle,
                            int16_t bound);
uint8_t myantenna_set_datatype(myantenna_laser_handle_t *laser_handle,
                               myantenna_datatype_t datatype);
uint8_t myantenna_set_address(myantenna_laser_handle_t *laser_handle,
                              uint8_t address);
uint8_t myantenna_set_frequence(myantenna_laser_handle_t *laser_handle,
                                myantenna_frequence_t frequence);
uint8_t myantenna_set_autmeas(myantenna_laser_handle_t *laser_handle,
                              myantenna_auto_t autmeas);
uint8_t myantenna_measuring_single(myantenna_laser_handle_t *laser_handle);
uint8_t myantenna_measuring_continue(myantenna_laser_handle_t *laser_handle);
uint8_t myantenna_measuring_continue_fast(
    myantenna_laser_handle_t *laser_handle);
uint8_t myantenna_measuring_stop(myantenna_laser_handle_t *laser_handle);
uint8_t myantenna_set_statu(myantenna_laser_handle_t *laser_handle,
                            myantenna_state_t statu);

#endif // !__MYANTENNA_L1_H