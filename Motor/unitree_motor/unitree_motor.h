/**
 * @file unitree_motor.h
 * @author meiwenhuaqingnian, xinglu,PickingChip
 * @brief GO-M8010-6关节电机驱动 通讯协议&数据包
 * @version 1.4
 * @date 2026/4/1
 *
 * @note 输出力矩 ：𝜏 = 𝜏𝑓𝑓 + 𝑘𝑝 × (𝑝𝑑𝑒𝑠 − 𝑝) + 𝑘𝑑 × (𝜔𝑑𝑒𝑠 − 𝜔)
 */
#ifndef __UNITREE_MOTOR_H
#define __UNITREE_MOTOR_H

#include <cubemx.h>

#define UNITREE_UART         UART4

// #define RS485_RE1_Pin       GPIO_PIN_14
// #define RS485_RE1_GPIO_Port GPIOD

// 减速比定义
#define REDUCTION_RATIO      6.33f

/* 需单独初始化配置IO */
#define RS485_RxMode()                                                         \
    (HAL_GPIO_WritePin(RS485_RE1_GPIO_Port, RS485_RE1_Pin, GPIO_PIN_RESET))
#define RS485_TxMode()                                                         \
    (HAL_GPIO_WritePin(RS485_RE1_GPIO_Port, RS485_RE1_Pin, GPIO_PIN_SET))

#pragma pack(1) /* 所有结构体按照1字节对齐 */
/**
 * @brief 电机模式控制信息
 *
 */
typedef struct {
    uint8_t id : 4; // 电机ID: 0,1...,13,14 15表示向所有电机广播数据(此时无返回)
    uint8_t status : 3;  // 工作模式: 0.锁定 1.FOC闭环 2.编码器校准 3.保留
    uint8_t reserve : 1; // 保留位
} RIS_Mode_t;            // 控制模式 1Byte

/**
 * @brief 电机状态控制信息
 *
 */
typedef struct {
    int16_t tor_des; // 期望关节输出扭矩 unit: N.m      (q8)
    int16_t spd_des; // 期望关节输出速度 unit: rad/s    (q8)
    int32_t pos_des; // 期望关节输出位置 unit: rad      (q15)
    int16_t k_pos;   // 期望关节刚度系数 unit: -1.0-1.0 (q15)
    int16_t k_spd;   // 期望关节阻尼系数 unit: -1.0-1.0 (q15)

} RIS_Comd_t; // 控制参数 12Byte

/**
 * @brief 电机状态反馈信息
 *
 */
typedef struct {
    int16_t torque; // 实际关节输出扭矩 unit: N.m     (q8)
    int16_t speed;  // 实际关节输出速度 unit: rad/s   (q8)
    int32_t pos;    // 实际关节输出位置 unit: rad     (q15)
    int8_t temp;    // 电机温度: -128~127°C
    uint8_t MError
        : 3; // 电机错误标识: 0.正常 1.过热 2.过流 3.过压 4.编码器故障 5-7.保留
    uint16_t force : 12; // 足端气压传感器数据 12bit (0-4095)
    uint8_t none : 1;    // 保留位
} RIS_Fbk_t;             // 状态数据 11Byte

/**
 * @brief 控制数据包格式
 *
 */
typedef struct {
    uint8_t head[2]; // 包头         2Byte
    RIS_Mode_t mode; // 电机控制模式  1Byte
    RIS_Comd_t comd; // 电机期望数据 12Byte
    uint16_t CRC16;  // CRC          2Byte

} RIS_ControlData_t; // 主机控制命令     17Byte

/**
 * @brief 电机反馈数据包格式
 *
 */
typedef struct {
    uint8_t head[2]; // 包头         2Byte
    RIS_Mode_t mode; // 电机控制模式  1Byte
    RIS_Fbk_t fbk;   // 电机反馈数据 11Byte
    uint16_t CRC16;  // CRC          2Byte

} RIS_MotorData_t; // 电机返回数据     16Byte

#pragma pack() /* 所有结构体按照1字节对齐 */

typedef struct {
    unsigned short id;   // 电机ID，15代表广播数据包
    unsigned short mode; // 0:空闲 1:FOC控制 2:电机标定
    float T;             // 期望关节的输出力矩(电机本身的力矩)(Nm)
    float W;             // 期望关节速度(电机本身的速度)(rad/s)
    float Pos;           // 期望关节位置(rad)
    float K_P;           // 关节刚度系数(0-25.599)
    float K_W;           // 关节速度系数(0-25.599)

} ctrl_param_t;

/**
 * @brief 电机参数结构体
 *
 */
typedef struct {
    uint8_t motor_id;   // 电机ID
    uint8_t mode;       // 0:空闲 1:FOC控制 2:电机标定
    int Temp;           // 温度
    int MError;         // 错误码
    float T;            // 当前实际电机输出力矩(电机本身的力矩)(Nm)
    float W;            // 当前实际电机速度(电机本身的速度)(rad/s)
    float Pos;          // 当前电机转子位置(rad)
    float offset_angle; // 电机上电时角度偏移量
    int correct;        // 接收数据是否完整(1完整，0不完整)
    int footForce;      // 足端力传感器原始数值
    bool got_offset;

    RIS_ControlData_t *send_data; // 指向发送数据的指针
    uint16_t calc_crc;
    uint32_t bad_msg; // CRC校验错误 数量

} unitree_motor_handle_t;

/**
 * @brief 消息结点
 *
 */
typedef struct rs_node {
    void *rs_data; /* 消息结点数据，一般为电机结构体 */
    uint8_t id;    /* 结点ID，与电机id保持一致 */
    struct rs_node *next;
} rs_node_t;

/**
 * @brief hash表
 *
 */
typedef struct {
    rs_node_t **table; /* 桶组 */
    uint8_t len;       /* 表长 */
} table_t;

uint8_t unitree_motor_init(unitree_motor_handle_t *motor, uint8_t motor_id,
                           uint8_t mode);
void unitree_send_data(UART_HandleTypeDef *huart, unitree_motor_handle_t *motor,
                       ctrl_param_t ctrl_param);
uint8_t rs_list_init(uint8_t len);

#endif /* __UNITREE_MOTOR_H */
