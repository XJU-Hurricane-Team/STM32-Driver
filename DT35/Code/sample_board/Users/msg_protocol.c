/**
 * @file    msg_protocol.c
 * @author  Deadline039
 * @brief   消息协议以及收发
 * @version 2.4
 * @date    2024-03-01
 */

#include "msg_protocol.h"

#include <string.h>
#include <stdbool.h>

#if MSG_ENABLE_CRC8
#include "crc/crc.h"
#endif /* MSG_ENABLE_CRC8 */

#if MSG_ENABLE_RTOS
#include "FreeRTOS.h"
#include "semphr.h"
#endif /* MSG_ENABLE_RTOS */

#ifdef __GNUC__
#pragma GCC diagnostic ignored "-Wzero-length-array"
#endif /* __GNUC__ */

/**
 * @brief 判断是否是 2 的幂次方
 * 
 * @param n 数字
 * @retval - 0:   不是
 * @retval - 其他: 是
 * 
 */
static inline uint32_t is_pow_of_2(uint32_t n) {
    return (0 != n) && (0 == (n & (n - 1)));
}

/**
 * @brief 环形缓冲区
 */
typedef struct {
    uint32_t size;          /*!< 缓冲区大小 */
    uint32_t mask;          /*!< 大小掩码 */
    uint8_t frame_len;      /*!< 帧长度 */
    bool new_frame;         /*!< 是否是新的一帧 (写长度用) */
    volatile uint32_t head; /*!< 头指针 */
    volatile uint32_t tail; /*!< 尾指针 */
    uint8_t buf[0];         /*!< 缓冲区 */
} msg_fifo_t;

struct msg_instance {
    msg_recv_callback_t recv_callback; /*!< 接收回调函数 */
    UART_HandleTypeDef *send_uart;     /*!< 发送串口句柄 */
    UART_HandleTypeDef *recv_uart;     /*!< 接收串口句柄 */

    uint8_t *send_buf;     /*!< 发送缓冲区 */
    uint32_t send_buf_len; /*!< 发送缓冲区大小 */

#if MSG_ENABLE_RTOS
    SemaphoreHandle_t send_buf_semp; /*!< 发送缓冲区二值型号量 */
#endif                               /* MSG_ENABLE_RTOS */

    uint8_t *recv_buf;      /*!< 接收缓冲区 */
    uint32_t recv_buf_size; /*!< 接收缓冲区大小 */

#ifdef MSG_ESC
    bool escape; /*!< 是否要将下一个字符转义 */
#endif           /* MSG_ESC */

    msg_fifo_t *fifo;          /*!< 接收缓冲区 */
    uint32_t fifo_element_len; /*!< 当前队列元素个数 */

#if MSG_ENABLE_STATISTICS
    uint32_t send_count; /*!< 发送计数 */

    uint32_t recv_success; /*!< 接收成功计数 */
    uint32_t recv_error;   /*!< 接收错误计数 */
#if MSG_ENABLE_CRC8
    uint32_t crc_check_error; /*!< CRC 校验错误计数 */
#endif                        /* MSG_ENABLE_CRC8 */

    uint32_t max_fifo_element_len; /*!< 最大队列元素个数 */
    uint32_t fifo_overflow;        /*!< 队列溢出清空计数 */
#endif                             /* MSG_ENABLE_STATISTICS */
};

struct msg_instance *msg_list[MSG_ID_RESERVE_LEN];
static msg_fifo_t *msg_fifo_init(uint32_t fifo_size);

/**
 * @brief 注册数据发送句柄
 *
 * @param msg_id 数据含义
 * @param huart 发送串口句柄
 * @param buf_size 缓冲区大小
 */
void message_register_send_uart(msg_id_t msg_id, UART_HandleTypeDef *huart,
                                uint32_t buf_size) {
    if (msg_id >= MSG_ID_RESERVE_LEN) {
        return;
    }

    if (msg_list[msg_id] == NULL) {
        msg_list[msg_id] =
            (struct msg_instance *)MSG_MALLOC(sizeof(struct msg_instance));
        memset(msg_list[msg_id], 0, sizeof(struct msg_instance));
    }

    struct msg_instance *msg = msg_list[msg_id];

    msg->send_uart = huart;
    if (msg->send_buf != NULL) {
        MSG_FREE(msg->send_buf);
    }

    msg->send_buf = (uint8_t *)MSG_MALLOC(buf_size);
    if (msg->send_buf == NULL) {
        return;
    }

    msg->send_buf_len = buf_size;
#if MSG_ENABLE_RTOS
    if (msg->send_buf_semp == NULL) {
        msg->send_buf_semp = xSemaphoreCreateMutex();
    }
#endif /* MSG_ENABLE_RTOS */
}

/**
 * @brief 注册接收回调函数指针
 *
 * @param msg_id 数据含义
 * @param msg_callback 回调指针
 * @note 如果更换回调函数, 重新调用该函数即可
 */
void message_register_recv_callback(msg_id_t msg_id,
                                    msg_recv_callback_t msg_callback) {
    if (msg_id >= MSG_ID_RESERVE_LEN) {
        return;
    }

    if (msg_list[msg_id] == NULL) {
        msg_list[msg_id] =
            (struct msg_instance *)MSG_MALLOC(sizeof(struct msg_instance));
        memset(msg_list[msg_id], 0, sizeof(struct msg_instance));
    }

    msg_list[msg_id]->recv_callback = msg_callback;
}

/**
 * @brief 注册数据接口串口句柄
 *
 * @param msg_id 数据含义
 * @param huart 接收串口句柄
 * @param buf_size 串口缓冲区大小
 * @param fifo_size 队列大小 (必须是 2 的幂次方! )
 */
void message_register_polling_uart(msg_id_t msg_id, UART_HandleTypeDef *huart,
                                   uint32_t buf_size, uint32_t fifo_size) {
    if (msg_id >= MSG_ID_RESERVE_LEN) {
        return;
    }

    if (is_pow_of_2(fifo_size) == 0) {
        /* 不是 2 的幂次方 */
        return;
    }

    if (msg_list[msg_id] == NULL) {
        msg_list[msg_id] =
            (struct msg_instance *)MSG_MALLOC(sizeof(struct msg_instance));
        memset(msg_list[msg_id], 0, sizeof(struct msg_instance));
    }

    struct msg_instance *msg = msg_list[msg_id];
    msg->recv_uart = huart;
    msg->recv_buf = (uint8_t *)MSG_MALLOC(buf_size);
    if (msg->recv_buf == NULL) {
        return;
    }
    msg->recv_buf_size = buf_size;

    msg->fifo = msg_fifo_init(fifo_size);
    if (msg->fifo == NULL) {
        return;
    }
}

/**
 * @brief 填充并发送数据, 支持多种类型
 *
 * @param msg_id 数据含义
 * @param data_type 数据类型
 * @param data 数据内容
 * @param data_len 发送长度
 */
void message_send_data(msg_id_t msg_id, msg_type_t data_type, uint8_t *data,
                       uint32_t data_len) {
    if (data == NULL || data_len == 0) {
        return;
    }

    if (msg_id >= MSG_ID_RESERVE_LEN) {
        return;
    }

    if (msg_list[msg_id] == NULL) {
        return;
    }

    struct msg_instance *msg = msg_list[msg_id];

    if (msg->send_uart == NULL || msg->send_buf == NULL) {
        return;
    }

#if MSG_ENABLE_RTOS
    xSemaphoreTake(msg->send_buf_semp, portMAX_DELAY);
#endif /* MSG_ENABLE_RTOS */

    if (msg->send_buf_len <= data_len + 5) {
        /* 不够, 扩容当前数据长度的 2 倍 */
        uint8_t *new_buf = (uint8_t *)MSG_REALLOC(msg->send_buf, data_len * 2);
        if (new_buf == NULL) {
            return;
        }

        msg->send_buf = new_buf;
        msg->send_buf_len = data_len * 2;
    } else if ((data_len + 3) * 3 <= msg->send_buf_len) {
        /* 长度小于 1/3, 缩容到原来的 1/2 */
        uint8_t *new_buf =
            (uint8_t *)MSG_REALLOC(msg->send_buf, msg->send_buf_len / 2);
        if (new_buf == NULL) {
            return;
        }

        msg->send_buf = new_buf;
        msg->send_buf_len = msg->send_buf_len / 2;
    }

    uint8_t *send_buf = (uint8_t *)msg->send_buf;
    uint32_t buf_idx = 0;

#if MSG_ENABLE_CRC8
    /* CRC8 校验结果 */
    uint8_t crc8_value = calc_crc8(data, data_len);
#endif /* MSG_ENABLE_CRC8 */

    /* 第一个字节, 高四位标记 ID, 低四位标记数据类型 */
    send_buf[buf_idx] = (uint8_t)(msg_id << 4) | data_type;
    ++buf_idx;
    /* 第二个字节, 标记数据长度 */
    send_buf[buf_idx] = (uint8_t)data_len;
    ++buf_idx;

    /* 复制数据到字节流 */
    for (uint32_t data_idx = 0; data_idx < data_len; ++data_idx) {
#ifdef MSG_ESC
        if ((data[data_idx] == MSG_EOF) || (data[data_idx] == MSG_ESC)) {
            /* 转义 */
            send_buf[buf_idx] = MSG_ESC;
            ++buf_idx;
        }
#endif /* MSG_ESC */

        send_buf[buf_idx] = data[data_idx];
        ++buf_idx;
    }

#if MSG_ENABLE_CRC8
    /* 添加 CRC8 帧校验数据, 拆成两个字节, 每个字节小于 0x10, 这样可以避免转义 */
    send_buf[buf_idx] = (crc8_value >> 4) & 0x0F;
    ++buf_idx;
    send_buf[buf_idx] = (crc8_value & 0x0F);
    ++buf_idx;
#endif /* MSG_ENABLE_CRC8 */

    /* 最后一个字节, 标记数据末尾 */
    send_buf[buf_idx] = MSG_EOF;
    ++buf_idx;

    if (msg->send_uart->hdmatx != NULL) {
        uart_dmatx_write(msg->send_uart, send_buf, buf_idx);
        uart_dmatx_send(msg->send_uart);
    } else {
        HAL_UART_Transmit(msg->send_uart, send_buf, buf_idx, 0xFFFF);
    }

#if MSG_ENABLE_STATISTICS
    ++msg->send_count;
#endif /* MSG_ENABLE_STATISTICS */

#if MSG_ENABLE_RTOS
    xSemaphoreGive(msg->send_buf_semp);
#endif /* MSG_ENABLE_RTOS */
}

static void message_data_enqueue(struct msg_instance *msg, uint32_t recv_len);
static void message_data_dequeue(struct msg_instance *msg);

/**
 * @brief 轮询数据, 并调用相应的函数
 *
 */
void message_polling_data(void) {
    struct msg_instance *msg;
    uint32_t recv_len;
    for (msg_id_t i = 0; i < MSG_ID_RESERVE_LEN; ++i) {
        msg = msg_list[i];
        if (msg == NULL) {
            continue;
        }

        if (msg->recv_uart == NULL) {
            continue;
        }

        if (msg->fifo == NULL) {
            continue;
        }

        message_data_dequeue(msg);

        recv_len =
            uart_dmarx_read(msg->recv_uart, msg->recv_buf, msg->recv_buf_size);
        if (recv_len == 0) {
            continue;
        }

        message_data_enqueue(msg, recv_len);
    }
}

/**
 * @brief 消息数据入队
 * 
 * @param msg 消息实例
 * @param recv_len 接收到的数据长度
 */
static void message_data_enqueue(struct msg_instance *msg, uint32_t recv_len) {
    msg_fifo_t *fifo = msg->fifo;
    for (uint32_t i = 0; i < recv_len; ++i) {
#ifdef MSG_ESC
        if ((msg->recv_buf[i] == MSG_ESC) && (msg->escape == false)) {
            /* 遇到转义, 跳过这一字节到下一字节 */
            msg->escape = true;
            continue;
        }
#endif /* MSG_ESC */
        if (fifo->new_frame) {
            /* 空一个字节写长度, 长度写 0 */
            fifo->buf[fifo->tail & fifo->mask] = 0;
            ++fifo->tail;
            fifo->new_frame = false;
        }

        /* 将数据写入队列 */
        fifo->buf[fifo->tail & fifo->mask] = msg->recv_buf[i];
        ++fifo->tail;
        ++fifo->frame_len;

        if (fifo->size == (fifo->tail - fifo->head)) {
            /* FIFO 已满, 先覆盖旧数据, 有新的帧直接清空 FIFO, 从头开始存 */
            fifo->head = 0;
            fifo->tail = 0;
            msg->fifo_element_len = 0;
            fifo->frame_len = 0;
#if MSG_ENABLE_STATISTICS
            ++msg->fifo_overflow;
#endif /* MSG_ENABLE_STATISTICS */
        }

#ifdef MSG_ESC
        if (msg->escape) {
            msg->escape = false;
            /* 这是被转义的字符, 避免执行到下面的结束标识被当成结束符处理 */
            continue;
        }
#endif /* MSG_ESC */

        if (msg->recv_buf[i] == MSG_EOF) {
            /* 写入上帧长度, 算上 FIFO 元素大小 */
            fifo->buf[(fifo->tail - fifo->frame_len - 1) & fifo->mask] =
                fifo->frame_len + 1;

            /* 帧长度清零 */
            fifo->frame_len = 0;

            /* 下次空出一个字节写 */
            fifo->new_frame = true;

            ++msg->fifo_element_len;
        }

#if MSG_ENABLE_STATISTICS
        if (msg->fifo_element_len > msg->max_fifo_element_len) {
            msg->max_fifo_element_len = msg->fifo_element_len;
        }
#endif /* MSG_ENABLE_STATISTICS */
    }
}

/**
 * @brief 消息数据出队并调用回调函数
 *
 * @param msg 消息实例
 */
static void message_data_dequeue(struct msg_instance *msg) {
    if (msg->fifo_element_len == 0) {
        /* 队列中没有元素 */
        return;
    }

    msg_fifo_t *fifo = msg->fifo;
    uint8_t frame_len;
    /* 回调消息 ID & type */
    uint8_t call_id_type;
    /* 回调消息数据 */
    uint8_t *call_data;
    /* 回调消息长度 */
    uint32_t call_len;
    /* 实际在缓冲区的位置指针 */
    uint32_t head, tail;

#if MSG_ENABLE_CRC8
    /* 接收到的 CRC8 校验值 */
    uint8_t crc_recv;
    /* 计算得到的 CRC8 校验值 */
    uint8_t crc_value;
#endif /* MSG_ENABLE_CRC8 */

    /* 队空条件: head == tail */
    while (fifo->head != fifo->tail) {
        /* 头存储的是帧长度 */
        frame_len = fifo->buf[fifo->head & fifo->mask];
        if (frame_len == 0) {
            /* 最后一帧还没存完, 存完才会写长度, 已经结束了, 跳出循环 */
            break;
        }

        if (fifo->head > fifo->tail) {
            /* 正常不可能头比尾还大 */
#if MSG_ENABLE_STATISTICS
            ++msg->recv_error;
#endif /* MSG_ENABLE_STATISTICS */
            break;
        }

        head = fifo->head & fifo->mask;
        tail = (fifo->head + frame_len) & fifo->mask;

        /* 验证数据包长度与实际接收长度是否一致, 数据包第二个字节是长度 */
#if MSG_ENABLE_CRC8
        /* 1 byte 标识, 1 byte 长度, 1 byte 结束符, 1 byte FIFO 元素大小
         * 2 byte CRC8 校验值
         * 总共 6 byte. */
        if ((frame_len - 6) != fifo->buf[(fifo->head + 2) & fifo->mask]) {
#else  /* MSG_ENABLE_CRC8 */
        /* 1 byte 标识, 1 byte 长度, 1 byte 结束符, 1 byte FIFO 元素大小
         * 总共 4 byte. */
        if ((frame_len - 4) != fifo->buf[(fifo->head + 2) & fifo->mask]) {
#endif /* MSG_ENABLE_CRC8 */

            /* 不一致, 出队到下一个 */
            fifo->head += frame_len;
            --msg->fifo_element_len;
#if MSG_ENABLE_STATISTICS
            ++msg->recv_error;
#endif /* MSG_ENABLE_STATISTICS */
            continue;
        }

        if (tail < head) {
            /* 数据在头尾, 分成两段, 直接复制到`recv_buf` (下一次读取就覆盖了) */
            if (frame_len <= msg->recv_buf_size) {
                /* 复制前半段 */
                memcpy(msg->recv_buf, &fifo->buf[head], fifo->size - head);
                /* 复制后半段 */
                memcpy(&msg->recv_buf[fifo->size - head], &fifo->buf[0], tail);
                call_id_type = msg->recv_buf[1];
#if MSG_ENABLE_CRC8
                call_len = frame_len - 6;
#else  /* MSG_ENABLE_CRC8 */
                call_len = frame_len - 4;
#endif /* MSG_ENABLE_CRC8 */
                call_data = &msg->recv_buf[3];
            } else {
                /* 空间不够, 不复制, 出队到下一个 */
                fifo->head += frame_len;
                --msg->fifo_element_len;
#if MSG_ENABLE_STATISTICS
                ++msg->recv_error;
#endif /* MSG_ENABLE_STATISTICS */
                continue;
            }
        } else {
            /* 完整的一帧没有被截断 */
            call_id_type = fifo->buf[(fifo->head + 1) & fifo->mask];
            call_len = fifo->buf[(fifo->head + 2) & fifo->mask];
            call_data = &fifo->buf[(fifo->head + 3) & fifo->mask];
        }

#if MSG_ENABLE_CRC8
        /* 校验 CRC8 */
        crc_value = calc_crc8(call_data, call_len);
        /* 接收到的 CRC8 校验值 */
        crc_recv =
            (call_data[frame_len - 5] & 0x0F) | (call_data[frame_len - 6] << 4);
        if (crc_value != crc_recv) {
            /* 校验结果不一致, 出队到下一个 */
            fifo->head += frame_len;
            --msg->fifo_element_len;
#if MSG_ENABLE_STATISTICS
            ++msg->crc_check_error;
#endif /* MSG_ENABLE_STATISTICS */
            continue;
        }
#endif /* MSG_ENABLE_CRC8 */

        if (msg->recv_callback) {
            msg->recv_callback(call_len, call_id_type, call_data);
        }
#if MSG_ENABLE_STATISTICS
        ++msg->recv_success;
#endif /* MSG_ENABLE_STATISTICS */

        /* 出队到下一个 */
        fifo->head += frame_len;
        --msg->fifo_element_len;
    }
}

/**
 * @brief 初始化消息队列
 * 
 * @param fifo_size 队列长度
 * @return 消息队列
 */
static msg_fifo_t *msg_fifo_init(uint32_t fifo_size) {
    msg_fifo_t *fifo = (msg_fifo_t *)MSG_MALLOC(sizeof(msg_fifo_t) + fifo_size);
    if (fifo == NULL) {
        return NULL;
    }

    fifo->size = fifo_size;
    fifo->mask = fifo_size - 1;
    fifo->head = 0;
    fifo->tail = 0;
    fifo->new_frame = true;

    return fifo;
}
