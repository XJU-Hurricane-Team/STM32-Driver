/**
 * @file unitree_motor.c
 * @author meiwenhuaqingnian, xinglu, PickingChip
 * @brief 宇树GO-M8010-6电机驱动 + rs485通信
 * @version 1.4
 * @date 2026/4/1
 *
 *
 */
#include <string.h>
#include <stdbool.h>
#include <limits.h>

#include "unitree_motor.h"
#include "./crc_ccitt/crc_ccitt.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

#define UNITREE_UART_WAIT_TICKS pdMS_TO_TICKS(5) /* 等待可用串口时间 */

#define SATURATE(_IN, _MIN, _MAX)                                              \
    {                                                                          \
        if ((_IN) <= (_MIN))                                                   \
            (_IN) = (_MIN);                                                    \
        else if ((_IN) >= (_MAX))                                              \
            (_IN) = (_MAX);                                                    \
    }

typedef struct {
    UART_HandleTypeDef *huart;
} queue_msg_t;

static table_t *rs_table;
static RIS_MotorData_t motor_recv_data = {0};

static QueueHandle_t unitree_queue_handle = NULL;
static TaskHandle_t unitree_receive_task_handle = NULL;
static SemaphoreHandle_t uart_available = NULL;
static queue_msg_t send_msg_from_isr;

/**
 * @brief 将浮点数转换为Q8格式
 * 
 * @param x 
 * @return int16_t 
 */
static int16_t q8_from_float(float x) {
    const float min_v = -128.0f;
    const float max_v = 127.99609375f;
    float scaled;

    if (x < min_v) {
        x = min_v;
    } else if (x > max_v) {
        x = max_v;
    }

    scaled = x * 256.0f;
    if (scaled >= 0.0f) {
        return (int16_t)((int32_t)(scaled + 0.5f));
    }

    return (int16_t)((int32_t)(scaled - 0.5f));
}

/**
 * @brief 将归一化浮点数转换为Q15格式
 * 
 * @param x 
 * @return int16_t 
 */

static int16_t q15_i16_from_norm(float x) {
    const float min_v = -1.0f;
    const float max_v = 0.9999694824f;
    float scaled;

    if (x < min_v) {
        x = min_v;
    } else if (x > max_v) {
        x = max_v;
    }

    scaled = x * 32768.0f;
    if (scaled >= 0.0f) {
        return (int16_t)((int32_t)(scaled + 0.5f));
    }

    return (int16_t)((int32_t)(scaled - 0.5f));
}

/**
 * @brief 将浮点数转换为Q15格式
 * 
 * @param x 
 * @return int32_t 
 */
static int32_t q15_i32_from_float(float x) {
    const double scaled = (double)x * 32768.0;

    if (scaled > (double)INT32_MAX) {
        return INT32_MAX;
    }
    if (scaled < (double)INT32_MIN) {
        return INT32_MIN;
    }

    return (int32_t)(scaled >= 0.0 ? (scaled + 0.5) : (scaled - 0.5));
}


/**
 * @brief 调整电机控制数据并发送
 *
 * @param motor         电机结构体指针
 * @param ctrl_param    控制参数结构体
 */
void unitree_send_data(UART_HandleTypeDef *huart, unitree_motor_handle_t *motor,
                       ctrl_param_t ctrl_param) {
    if (motor == NULL || motor->send_data == NULL) {
        return;
    }

    motor->send_data->head[0] = 0xFE;
    motor->send_data->head[1] = 0xEE;

    /* 限制控制参数范围 */
    SATURATE(ctrl_param.id, 0, 15);
    SATURATE(ctrl_param.mode, 0, 7);
    SATURATE(ctrl_param.K_P, 0.0f, 25.599f);
    SATURATE(ctrl_param.K_W, 0.0f, 25.599f);
    SATURATE(ctrl_param.T, -127.99f, 127.99f);
    SATURATE(ctrl_param.W, -804.00f, 804.00f);
    SATURATE(ctrl_param.Pos, -181.546f, 181.546f);

    if (!(motor->got_offset)) {
        /* 如果还未得到偏移量就不进行输出 */
        ctrl_param.W = 0.0f;
        ctrl_param.T = 0.0f;
        ctrl_param.K_P = 0.0f;
        ctrl_param.K_W = 0.0f;
    }

    motor->send_data->mode.id = ctrl_param.id;
    motor->send_data->mode.status = ctrl_param.mode;
    motor->send_data->comd.k_pos = q15_i16_from_norm(ctrl_param.K_P / 25.6f);
    motor->send_data->comd.k_spd = q15_i16_from_norm(ctrl_param.K_W / 25.6f);
    motor->send_data->comd.pos_des = q15_i32_from_float(
        (ctrl_param.Pos * REDUCTION_RATIO + motor->offset_angle) / 6.28318f);
    motor->send_data->comd.spd_des =
        q8_from_float((ctrl_param.W * REDUCTION_RATIO) / 6.28318f);
    motor->send_data->comd.tor_des =
        q8_from_float(ctrl_param.T * REDUCTION_RATIO);
    motor->send_data->CRC16 =
        crc_ccitt(0, (uint8_t *)motor->send_data,
                  sizeof(RIS_ControlData_t) - sizeof(motor->send_data->CRC16));

    /* 确保串口没有被占用 */
    if (xSemaphoreTake(uart_available, UNITREE_UART_WAIT_TICKS) != pdTRUE) {
        /* 防止异常帧导致串口长期被占用，超时后强制恢复接收状态 */
        HAL_UART_AbortReceive(huart);
        RS485_RxMode();
        xSemaphoreGive(uart_available);
        return;
    }

    RS485_TxMode();
    if (huart->hdmatx != NULL) {
        /* 初始化了 DMA 使用 DMA 发送数据 */
        if (HAL_UART_Transmit_DMA(huart, (uint8_t *)motor->send_data,
                                  sizeof(RIS_ControlData_t)) != HAL_OK) {
            xSemaphoreGive(uart_available);
            return;
        }
    } else {
        /* 否则阻塞式发送 */
        if (HAL_UART_Transmit(huart, (uint8_t *)motor->send_data,
                              sizeof(RIS_ControlData_t), 20) != HAL_OK) {
            xSemaphoreGive(uart_available);
            return;
        }

        RS485_RxMode(); /* 发送完成后立即启动接收 */
        if (huart->hdmarx != NULL) {
            /* 初始化了 DMA 使用 DMA 接收数据 */
            if (HAL_UART_Receive_DMA(huart, (uint8_t *)&motor_recv_data,
                                     sizeof(RIS_MotorData_t)) != HAL_OK) {
                /* 接收 DMA 启动失败时释放串口，避免发送任务永久阻塞 */
                xSemaphoreGive(uart_available);
                return;
            }
        } else {
            /* 否则中断式接收 */
            if (HAL_UART_Receive_IT(huart, (uint8_t *)&motor_recv_data,
                                    sizeof(RIS_MotorData_t)) != HAL_OK) {
                xSemaphoreGive(uart_available);
                return;
            }
        }
    }
}

/**
 * @brief 宇树电机串口发送完成回调函数
 * @note 在usart_ex.c中被HAL_UART_TxCpltCallback调用
 * @param huart 串口句柄
 */
void unitree_uart_tx_cplt_callback(UART_HandleTypeDef *huart) {
    if (huart->Instance == UNITREE_UART) {
        /* 发送完成后立即启动接收 */
        RS485_RxMode();
        if (huart->hdmarx != NULL) {
            /* 初始化了 DMA 使用 DMA 接收数据 */
            if (HAL_UART_Receive_DMA(huart, (uint8_t *)&motor_recv_data,
                                     sizeof(RIS_MotorData_t)) != HAL_OK) {
                /* 接收 DMA 启动失败时释放串口，避免发送任务永久阻塞 */
                xSemaphoreGiveFromISR(uart_available, NULL);
                return;
            }
        } else {
            /* 否则阻塞式接收 */
            if (HAL_UART_Receive_IT(huart, (uint8_t *)&motor_recv_data,
                                    sizeof(RIS_MotorData_t)) != HAL_OK) {
                xSemaphoreGiveFromISR(uart_available, NULL);
                return;
            }
        }
    }
}

/**
 * @brief 宇树电机串口接收完成回调函数
 * @note 在usart_ex.c中被HAL_UART_RxCpltCallback调用
 * @param huart 串口句柄
 */
void unitree_uart_rx_cplt_callback(UART_HandleTypeDef *huart) {

    if (huart->Instance == UNITREE_UART) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        if (unitree_queue_handle == NULL) {
            return;
        }

        send_msg_from_isr.huart = huart;
        xQueueSendFromISR(unitree_queue_handle, &send_msg_from_isr,
                          &xHigherPriorityTaskWoken);
        /* 接收完成释放对串口的占用 */
        xSemaphoreGiveFromISR(uart_available, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}


/**
 * @brief 宇树电机接收任务，负责处理接收到的数据并更新电机状态
 * 
 * @param pvParameters 
 */
void unitree_receive_task(void *pvParameters) {
    UNUSED(pvParameters);

    queue_msg_t recv_msg;
    rs_node_t *node;
    unitree_motor_handle_t *p;

    while (1) {
        xQueueReceive(unitree_queue_handle, &recv_msg, portMAX_DELAY);

        if (motor_recv_data.head[0] != 0xFD ||
            motor_recv_data.head[1] != 0xEE) {
            continue;
        }

        /* 查找电机结点 */
        node = rs_table->table[motor_recv_data.mode.id % rs_table->len];
        while ((node != NULL) && node->id != motor_recv_data.mode.id) {
            node = node->next;
        }
        if (node == NULL) {
            continue;
        }

        p = node->rs_data;

        p->calc_crc =
            crc_ccitt(0, (uint8_t *)&motor_recv_data,
                      sizeof(RIS_MotorData_t) - sizeof(motor_recv_data.CRC16));
        if (motor_recv_data.CRC16 != p->calc_crc) {
            memset(&motor_recv_data, 0, sizeof(RIS_MotorData_t));
            p->correct = 0;
            p->bad_msg++;
            continue;
        } else {

            p->mode = motor_recv_data.mode.status;
            p->Temp = motor_recv_data.fbk.temp;
            p->MError = motor_recv_data.fbk.MError;
            p->W = (((float)motor_recv_data.fbk.speed / 256.0f) * 6.28318f) *
                   REDUCTION_RATIO;
            p->T = (((float)motor_recv_data.fbk.torque) / 256.0f) *
                   REDUCTION_RATIO;
            p->Pos = (6.28318f * ((float)motor_recv_data.fbk.pos) / 32768.0f -
                      p->offset_angle) /
                     REDUCTION_RATIO;
            p->footForce = motor_recv_data.fbk.force;
            p->correct = 1;

            if (!(p->got_offset)) {
                p->offset_angle =
                    6.28318f * ((float)motor_recv_data.fbk.pos) / 32768.0f;
                p->got_offset = true;
            }
        }
    }
}



/**
 * @brief 添加新节点
 *
 * @param node_data 结点数据，一般为电机结构体
 * @param id        结点ID，与电机id保持一致
 * @return uint8_t
 */
uint8_t rs_list_add_new_node(void *node_data, uint8_t id) {
    if (node_data == NULL) {
        return 1;
    }
    rs_node_t *new_node = (rs_node_t *)malloc(sizeof(rs_node_t));
    if (new_node == NULL) {
        return 2;
    }

    new_node->rs_data = node_data;
    new_node->id = id;

    rs_node_t **table_head = &rs_table->table[id % rs_table->len];

    new_node->next = *table_head;
    *table_head = new_node;
    return 0;
}


/**
 * @brief 电机初始化
 *
 * @param motor      电机结构体指针
 * @param motor_id   电机ID
 * @param mode 	 	 电机模式（0:空闲 1:FOC控制 2:电机标定）
 * @return uint8_t
 */
uint8_t unitree_motor_init(unitree_motor_handle_t *motor, uint8_t motor_id,
                           uint8_t mode) {
    if (motor == NULL) {
        return 1;
    }

    motor->send_data = malloc(sizeof(RIS_ControlData_t));
    if (motor->send_data == NULL) {
        return 1;
    }

    motor->motor_id = motor_id;
    motor->mode = mode;
    motor->got_offset = false;

    rs_list_add_new_node(motor, motor_id);

    RS485_RxMode();
    return 0;
}


/**
 * @brief 初始化消息哈希表
 *
 * @param len 桶组长度，大于等于电机数量
 * @return uint8_t
 */
uint8_t rs_list_init(uint8_t len) {
    rs_table = (table_t *)calloc(1, sizeof(table_t));
    if (rs_table == NULL) {
        return 1;
    }

    rs_table->table = (rs_node_t **)calloc(len, sizeof(rs_node_t *));
    if (rs_table->table == NULL) {
        return 2;
    }

    rs_table->len = len;

    if (unitree_queue_handle == NULL) {
        /* 创建接收任务与接收队列 */
        unitree_queue_handle = xQueueCreate(1, sizeof(queue_msg_t));
        xTaskCreate(unitree_receive_task, "UnitreeReceiveTask", 256, NULL, 4,
                    &unitree_receive_task_handle);
        /* 创建串口锁 */
        uart_available = xSemaphoreCreateBinary();
        xSemaphoreGive(uart_available);
    }

    return 0;
}