/**
 * @file unitree_motor.c
 * @author meiwenhuaqingnian, xinglu
 * @brief 宇树GO-M8010-6电机驱动 + rs485通信 
 * @version 1.2
 * @date 2026-3-22
 * 
 * 
 */

#include "unitree_motor.h"
#include <CSP_Config.h>
#include <string.h>

#define SATURATE(_IN, _MIN, _MAX)                                              \
    {                                                                          \
        if ((_IN) <= (_MIN))                                                   \
            (_IN) = (_MIN);                                                    \
        else if ((_IN) >= (_MAX))                                              \
            (_IN) = (_MAX);                                                    \
    }

uint8_t rs_list_add_new_node(void *node_data, uint8_t id);

RIS_ControlData_t motor_send_data = {0}; // 电机发送数据结构体
RIS_MotorData_t motor_recv_data = {0};

table_t *rs_table;

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

    motor->motor_id = motor_id;
    motor->mode = mode;
    motor->got_offset = false;

    rs_list_add_new_node(motor, motor_id);

    RS485_RxMode();
    return 0;
}

/**
 * @brief 调整电机控制数据并发送
 * 
 * @param motor         电机结构体指针
 * @param ctrl_param    控制参数结构体
 */
void unitree_send_data(UART_HandleTypeDef *huart,unitree_motor_handle_t *motor, ctrl_param_t ctrl_param) {

    motor_send_data.head[0] = 0xFE;
    motor_send_data.head[1] = 0xEE;

    SATURATE(ctrl_param.id, 0, 15);
    SATURATE(ctrl_param.mode, 0, 7);
    SATURATE(ctrl_param.K_P, 0.0f, 25.599f);
    SATURATE(ctrl_param.K_W, 0.0f, 25.599f);
    SATURATE(ctrl_param.T, -127.99f, 127.99f);
    SATURATE(ctrl_param.W, -804.00f, 804.00f);
    SATURATE(ctrl_param.Pos, -181.546f, 181.546f);

    if (!(motor->got_offset)) {
        ctrl_param.W = 0.0f;
        ctrl_param.T = 0.0f;
        ctrl_param.K_P = 0.0f;
        ctrl_param.K_W = 0.0f;
    }   

    motor_send_data.mode.id = ctrl_param.id;
    motor_send_data.mode.status = ctrl_param.mode;
    motor_send_data.comd.k_pos = ctrl_param.K_P / 25.6f * 32768.0f;
    motor_send_data.comd.k_spd = ctrl_param.K_W / 25.6f * 32768.0f;
    motor_send_data.comd.pos_des = (ctrl_param.Pos * REDUCTION_RATIO + motor->offset_angle) / 6.28318f * 32768.0f;
    motor_send_data.comd.spd_des = (ctrl_param.W * REDUCTION_RATIO) / 6.28318f * 256.0f;
    motor_send_data.comd.tor_des = (ctrl_param.T * REDUCTION_RATIO) * 256.0f;
    motor_send_data.CRC16 =
        crc_ccitt(0, (uint8_t *)&motor_send_data,
                  sizeof(RIS_ControlData_t) - sizeof(motor_send_data.CRC16));

    RS485_TxMode();

    uart_dmatx_write(huart, &motor_send_data, sizeof(motor_send_data));
    uart_dmatx_send(huart);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        RS485_RxMode();
    }
}

/**
 * @brief 接收电机数据
 * 
 * @param huart 接收使用的串口句柄
 * @return uint8_t 
 */
uint8_t unitree_receive_data(UART_HandleTypeDef *huart) {
    uint8_t rx_res = 0;

    rx_res = uart_dmarx_read(huart, (uint8_t *)&motor_recv_data,
                             sizeof(motor_recv_data));
    if (rx_res == 0) {
        return 1;
    }

    if (motor_recv_data.head[0] != 0xFD || motor_recv_data.head[1] != 0xEE) {
        return 2;
    }

    rs_node_t *node;
    unitree_motor_handle_t *p;

    /* 查找电机结点 */
    node = rs_table->table[motor_recv_data.mode.id % rs_table->len];
    while ((node != NULL) && node->id != motor_recv_data.mode.id) {
        node = node->next;
    }
    if (node == NULL) {
        return 3;
    }

    p = node->rs_data;

    p->calc_crc =
        crc_ccitt(0, (uint8_t *)&motor_recv_data,
                  sizeof(RIS_MotorData_t) - sizeof(motor_recv_data.CRC16));
    if (motor_recv_data.CRC16 != p->calc_crc) {
        memset(&motor_recv_data, 0, sizeof(RIS_MotorData_t));
        p->correct = 0;
        p->bad_msg++;
        return 4;
    } else {

        p->mode = motor_recv_data.mode.status;
        p->Temp = motor_recv_data.fbk.temp;
        p->MError = motor_recv_data.fbk.MError;
        p->W = (((float)motor_recv_data.fbk.speed / 256.0f) * 6.28318f) * REDUCTION_RATIO;
        p->T = (((float)motor_recv_data.fbk.torque) / 256.0f) * REDUCTION_RATIO;
        p->Pos = (6.28318f * ((float)motor_recv_data.fbk.pos) / 32768.0f - p->offset_angle) / REDUCTION_RATIO;
        p->footForce = motor_recv_data.fbk.force;
        p->correct = 1;

        if (!(p->got_offset)) {
            p->offset_angle = 6.28318f * ((float)motor_recv_data.fbk.pos) / 32768.0f;
            p->got_offset = true;
        }
    }
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
    return 0;
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
