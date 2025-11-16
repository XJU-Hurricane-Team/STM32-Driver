/**
 * @file    action_position.h
 * @author  Deadline039
 * @brief   东大全场定位解析代码
 * @version 0.2
 * @date    2023-11-11
 * @ref
 */

#include "action_position.h"
#include "math.h"
/* 串口通信句柄 */
static UART_HandleTypeDef *uart_handle = NULL;
static uint32_t time = 0;
/**
 * @brief 全场定位数据
 */
act_pos_data_t g_position_data;

/* 从串口接收到的字符 */
static uint8_t g_uart_byte;

/**
 * @brief 从串口数据读取字节, 转换成坐标数据
 *
 * @param recv_byte 接收的一个字节
 * @note 此程序会更新外部变量g_position_x, g_position_y, g_position_roll,
 *       g_position_pitch, g_position_yaw, g_position_yaw_speed的值
 */
static void uart_receive_callback(UART_HandleTypeDef *huart) {
    UNUSED(huart);

    /* 接收计数 */
    static uint8_t recv_count = 0;
    /* 数据索引 */
    static uint8_t data_index = 0;

    /* 数据接收结构 */
    static union {
        uint8_t recv_data[24];
        float act_val[6];
    } data_buffer;

    switch (recv_count) {
        case 0: {
            if (g_uart_byte == 0x0D) {
                recv_count++;
            } else {
                recv_count = 0;
            }
        } break;

        case 1: {
            if (g_uart_byte == 0x0A) {
                data_index = 0;
                recv_count++;
            } else if (g_uart_byte == 0x0D) {
            } else {
                recv_count = 0;
            }
        } break;

        case 2: {
            data_buffer.recv_data[data_index] = g_uart_byte;
            data_index++;
            if (data_index >= 24) {
                data_index = 0;
                recv_count++;
            }
        } break;

        case 3: {
            if (g_uart_byte == 0x0A) {
                recv_count++;
            } else {
                recv_count = 0;
            }
        } break;

        case 4: {
            if (g_uart_byte == 0x0D) {
                g_position_data.yaw = data_buffer.act_val[0];
                g_position_data.roll = data_buffer.act_val[1];
                g_position_data.pitch = data_buffer.act_val[2];
                g_position_data.x = -data_buffer.act_val[3];
                g_position_data.y = -data_buffer.act_val[4];
                g_position_data.yaw_speed = data_buffer.act_val[5];
                g_position_data.v = (sqrt(pow(g_position_data.x, 2) +
                                          pow(g_position_data.y, 2))) /
                                    (HAL_GetTick() - time);
                time = HAL_GetTick();
            }
            recv_count = 0;
        } break;

        default: {
            recv_count = 0;
        } break;
    }
    HAL_UART_Receive_IT(huart, &g_uart_byte, 1);
}

/**
 * @brief 注册串口, 将通过这个串口收发数据
 *
 * @param huart
 */
void act_position_register_uart(UART_HandleTypeDef *huart) {
    if (huart == NULL) {
        return;
    }

    uart_handle = huart;
    HAL_UART_Receive_IT(huart, &g_uart_byte, 1);
    HAL_UART_RegisterCallback(huart, HAL_UART_RX_COMPLETE_CB_ID,
                              uart_receive_callback);
}

/**
 * @brief 字符串拼接, 构造全场定位的发送数据
 *
 * @param[out] output_str 输出字符串
 * @param[in] source_str 源字符串
 * @param[in] len 字符串长度
 */
static void stract(uint8_t *output_str, uint8_t *source_str, uint16_t len) {

    uint16_t start_ptr = 0;
    while (output_str[start_ptr] != '\0') {
        start_ptr++;
    }
    for (uint32_t i = 0; i < len; i++) {
        output_str[start_ptr++] = source_str[i];
    }
}

/**
 * @brief 设置新数据结构体, 由于是共用体,
 *        将小数放入时会自动将data数组变成小数的内存内容,
 *        无需再进行各种二进制操作把浮点数转换成byte
 */
static union {
    float new_data;
    uint8_t data[4];
} new_set;

/**
 * @brief 更新X坐标
 *
 * @param new_x 新的x
 * @note 为了防止重复更新全场定位数据忘记延时, 导致全场定位数据出错,
 *       函数内部会延时10ms
 */
void act_position_update_x(float new_x) {
    uint8_t update_data[8] = "ACTX";
    new_set.new_data = new_x;
    stract(update_data, new_set.data, 4);
    HAL_UART_Transmit(uart_handle, update_data, 8, 0xFFFF);
    HAL_Delay(10);
}

/**
 * @brief 更新Y坐标
 *
 * @param new_y 新的y
 * @note 为了防止重复更新全场定位数据忘记延时, 导致全场定位数据出错,
 *       函数内部会延时10ms
 */
void act_position_update_y(float new_y) {
    uint8_t update_data[8] = "ACTY";
    new_set.new_data = new_y;
    stract(update_data, new_set.data, 4);
    HAL_UART_Transmit(uart_handle, update_data, 8, 0xFFFF);
    HAL_Delay(10);
}

/**
 * @brief 更新航向角(z轴)
 *
 * @param new_yaw 新的航向角
 * @note 为了防止重复更新全场定位数据忘记延时, 导致全场定位数据出错,
 *       函数内部会延时10ms
 */
void act_position_update_yaw(float new_yaw) {
    uint8_t update_data[8] = "ACTJ";
    new_set.new_data = new_yaw;
    stract(update_data, new_set.data, 4);
    HAL_UART_Transmit(uart_handle, update_data, 8, 0xFFFF);
    HAL_Delay(10);
}

/**
 * @brief 清空全场定位数据, 从0开始
 *
 * @note 为了防止重复更新全场定位数据忘记延时, 导致全场定位数据出错,
 *       函数内部会延时10ms
 */
void act_position_reset_data(void) {
    uint8_t update_data[8] = "ACT0";
    HAL_UART_Transmit(uart_handle, update_data, 8, 0xFFFF);
    HAL_Delay(10);
}
