/**
 * @file    ak_motor.cpp
 * @author  Deadline039
 * @brief   AK 电机驱动代码
 * @version 1.0
 * @date    2023-11-27
 * @see     https://github.com/Yangwen-li13/CubeMars-AK60-6/
 */

#include "ak_motor.h"

#include "buffer_append/buffer_append.h"
#include "can_list/can_list.h"

/******************************************************************************
 * @defgroup 伺服模式驱动
 * @{
 */

/**
 * @brief 伺服模式阈值控制
 *
 */
#define MAX_PWM               1.0F      /*!< 最大占空比 */
#define MAX_CURRENT           60000.0F  /*!< 最大电流 */
#define MAX_VELOCITY          100000.0F /*!< 最大速率 */
#define MAX_POSITION          36000.0F  /*!< 最大位置 */
#define MAX_POSITION_VELOCITY 32767.0F  /*!< 旋转最大速率 */
#define MIN_POSITION_VELOCITY -32768.0F /*!< 旋转最小速率 */
#define MAX_ACCELERATION      32767.0F  /*!< 最大加速度 */

/**
 * @brief CAN 消息，作为 CAN ID 封装
 */
typedef enum {
    CAN_PACKET_SET_PWM = 0U,
    CAN_PACKET_SET_CURRENT,
    CAN_PACKET_SET_CURRENT_BRAKE,
    CAN_PACKET_SET_RPM,
    CAN_PACKET_SET_POS,
    CAN_PACKET_SET_ORIGIN_HERE,
    CAN_PACKET_SET_POS_SPD
} ak_can_msg_t;
/**
 * @}
 */

/******************************************************************************
 * @defgroup 运控模式驱动
 * @{
 */

/**
 * @brief 运控模式阈值控制
 */
#define AK_MIT_POSITION_LIMIT 12.5F  /*!< 最大位置 */
#define AK_MIT_KP_LIMIT       500.0F /*!< 最大 KP */
#define AK_MIT_KD_LIMIT       5.0F   /*!< 最大 KD */

/**
 * @brief ak_mit_param_limit 数组下标
 * @note 不同型号电机只有速度和扭矩不同，其他都是一样的
 */
enum {
    MIT_SPEED_LIMIT_INDEX = 0, /*!< 最大速度下标定义 */
    MIT_TORQUE_LIMMIT_INDEX    /*!< 最大扭矩下标定义 */
};

/* 电机阈值表 */
static const float ak_mit_param_limit[7][2] = {
    {50.0f, 65.0f}, {45.0f, 15.0f}, {50.0f, 25.0f}, {76.0f, 12.0f},
    {50.0f, 18.0f}, {8.0f, 144.0f}, {37.5f, 32.0f}};

/**
 * @}
 */

/**
 * @brief 获得电机状态参数，运控模式和伺服模式是一样的，只是帧格式不同
 *
 * @param can_ptr CAN 列表中的指针，在这里就是 AK 电机对象
 * @param can_rx_header CAN 消息头
 * @param recv_msg 接收到的数据
 */
static void ak_can_callback(void *can_ptr, can_rx_header_t *can_rx_header,
                            uint8_t *recv_msg) {
    ak_motor_handle_t *ak_target = (ak_motor_handle_t *)can_ptr;
    int32_t buffer_index = 0;

    if (can_rx_header->id_type == CAN_ID_EXT) {
        /* 扩展帧，伺服模式 */
        ak_target->pos = buffer_get_float16(recv_msg, 10.0f, &buffer_index);
        ak_target->spd = buffer_get_float16(recv_msg, 0.01f, &buffer_index);
        ak_target->current_troq =
            buffer_get_float16(recv_msg, 10.0f, &buffer_index);

    } else if (can_rx_header->id_type == CAN_ID_STD) {
        /* 标准帧，运控模式 */
        uint16_t pos_int = (recv_msg[1] << 8) | recv_msg[2];
        uint16_t spd_int = (recv_msg[3] << 4) | (recv_msg[4] >> 4);
        uint16_t torq_int = ((recv_msg[4] & 0xF) << 8) | recv_msg[5];

        ak_target->pos = uint_to_float(pos_int, -AK_MIT_POSITION_LIMIT,
                                       AK_MIT_POSITION_LIMIT, 16);
        ak_target->spd = uint_to_float(
            spd_int,
            -ak_mit_param_limit[ak_target->model][MIT_SPEED_LIMIT_INDEX],
            ak_mit_param_limit[ak_target->model][MIT_SPEED_LIMIT_INDEX], 12);
        ak_target->current_troq = uint_to_float(
            torq_int,
            -ak_mit_param_limit[ak_target->model][MIT_TORQUE_LIMMIT_INDEX],
            ak_mit_param_limit[ak_target->model][MIT_TORQUE_LIMMIT_INDEX], 12);
    }

    ak_target->motor_temperature = recv_msg[6];
    ak_target->error_code = recv_msg[7];
}

/**
 * @brief 初始化 AK 电机
 *
 * @param motor AK 电机对象
 * @param id CAN ID
 * @param model AK 电机型号
 * @param mode AK 电机模式
 * @param can_select 选择 CAN
 * @return 初始化状态：
 * @retval - 0：成功
 * @retval - 1: `motor` 指针为空
 * @retval - 2: 添加 CAN 接收列表错误
 * @retval - 3: 参数错误
 */
uint8_t ak_motor_init(ak_motor_handle_t *motor, uint32_t id, ak_model_t model,
                      ak_mode_t mode, can_selected_t can_select) {
    if (motor == NULL) {
        return 1;
    }

    motor->id = id;
    motor->model = model;
    motor->can_select = can_select;

    uint32_t id_type, id_mask;
    if (mode == AK_MODE_MIT) {
        id_type = CAN_ID_STD;
        id_mask = 0x7FF;
    } else if (mode == AK_MODE_SERVO) {
        id_type = CAN_ID_EXT;
        id_mask = 0xFF;
    } else {
        return 3;
    }

    if (model >= AK_MODEL_RESERVE) {
        return 3;
    }

    if (can_list_add_new_node(can_select, (void *)motor, id, 0xFF, id_type,
                              ak_can_callback) != 0) {
        return 2;
    }

    return 0;
}

/**
 * @brief 销毁 AK 电机，从链表中删除
 *
 * @param motor AK 电机对象指针
 * @return 反初始化状态
 * @retval - 0: 成功
 * @retval - 1: `motor` 为空
 * @retval - 2: 移除错误
 */
uint8_t ak_motor_deinit(ak_motor_handle_t *motor) {
    if (motor == NULL) {
        return 1;
    }

    uint32_t id_type;

    if (motor->mode == AK_MODE_MIT) {
        id_type = CAN_ID_STD;
    } else if (motor->mode == AK_MODE_SERVO) {
        id_type = CAN_ID_EXT;
    } else {
        return 3;
    }

    if (can_list_del_node_by_id(motor->can_select, id_type, motor->id) != 0) {
        return 2;
    }

    return 0;
}

/******************************************************************************
 * @defgroup 伺服模式驱动
 * @{
 */

/**
 * @brief 模式和 ID 整合
 *
 * @param id CAN ID
 * @param mode 电机模式
 * @return 整合好的 ID
 */
static inline uint32_t canid_append_mode(uint8_t id, ak_can_msg_t mode) {
    return (uint32_t)(id | mode << 8);
}
/**
 * @brief 输出参数限制
 *
 * @param [out] value 传入值和结果
 * @param [in] min_value 最小值
 * @param [in] max_value 最大值
 */
static inline void param_limit(float *value, float min_value, float max_value) {
    if (*value > max_value) {
        *value = max_value;
    } else if (*value < min_value) {
        *value = min_value;
    }
}

/**
 * @brief 占空比模式设置电机转速
 *
 * @param motor 电机对象
 * @param duty 占空比，范围 `0 ~ 1.0`
 */
void ak_servo_set_duty(ak_motor_handle_t *motor, float duty) {
    if (motor == NULL) {
        return;
    }

    param_limit(&duty, 0, MAX_PWM);
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_int32(buffer, (int32_t)(duty * 100000.0f), &send_index);

    can_send_message(motor->can_select, CAN_ID_EXT,
                     canid_append_mode(motor->id, CAN_PACKET_SET_PWM),
                     send_index, buffer);
}

/**
 * @brief 设置电机电流
 *
 * @param motor 电机对象
 * @param current 电流值，范围 `-60000 ~ 60000 mA`
 * @note 由于电机 `输出扭矩 = iq * KT`, 所以可以当作扭矩环使用
 */
void ak_servo_set_current(ak_motor_handle_t *motor, float current) {
    if (motor == NULL) {
        return;
    }
    param_limit(&current, -MAX_CURRENT, MAX_CURRENT);
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_int32(buffer, (int32_t)(current * 1000.0f), &send_index);

    can_send_message(motor->can_select, CAN_ID_EXT,
                     canid_append_mode(motor->id, CAN_PACKET_SET_CURRENT),
                     send_index, buffer);
}

/**
 * @brief 设置电机刹车电流
 *
 * @param motor 电机对象
 * @param current 刹车电流，范围 `0 ~ 60000 mA`
 */
void ak_servo_set_cb(ak_motor_handle_t *motor, float current) {
    if (motor == NULL) {
        return;
    }
    param_limit(&current, -MAX_CURRENT, MAX_CURRENT);
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_int32(buffer, (int32_t)(current * 1000.0f), &send_index);

    can_send_message(motor->can_select, CAN_ID_EXT,
                     canid_append_mode(motor->id, CAN_PACKET_SET_CURRENT_BRAKE),
                     send_index, buffer);
}

/**
 * @brief 速度环模式设置速度
 *
 * @param motor 电机对象
 * @param rpm 速率，范围 `-100000 ~ 100000 erpm`
 * @note `erpm = rpm * 极对数`
 */
void ak_servo_set_rpm(ak_motor_handle_t *motor, float rpm) {
    if (motor == NULL) {
        return;
    }
    param_limit(&rpm, -MAX_VELOCITY, MAX_VELOCITY);
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_int32(buffer, (int32_t)rpm, &send_index);

    can_send_message(motor->can_select, CAN_ID_EXT,
                     canid_append_mode(motor->id, CAN_PACKET_SET_RPM),
                     send_index, buffer);
}

/**
 * @brief 位置环模式设置位置
 *
 * @param motor 电机对象
 * @param pos 位置 (角度，范围 `-36 000° ~ 36 000°`)
 * @note 默认速度 12000erpm, 加速度 40000erpm
 */
void ak_servo_set_pos(ak_motor_handle_t *motor, float pos) {
    if (motor == NULL) {
        return;
    }
    param_limit(&pos, -MAX_POSITION, MAX_POSITION);
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_int32(buffer, (int32_t)(pos * 10000.0f), &send_index);

    can_send_message(motor->can_select, CAN_ID_EXT,
                     canid_append_mode(motor->id, CAN_PACKET_SET_POS),
                     send_index, buffer);
}

/**
 * @brief 设置原点
 *
 * @param motor 电机对象
 * @param set_origin_mode 设置原点模式
 */
void ak_servo_set_origin(ak_motor_handle_t *motor,
                         ak_origin_mode_t set_origin_mode) {
    if (motor == NULL) {
        return;
    }
    uint8_t buffer = set_origin_mode;

    can_send_message(motor->can_select, CAN_ID_EXT,
                     canid_append_mode(motor->id, CAN_PACKET_SET_ORIGIN_HERE),
                     1, &buffer);
}

/**
 * @brief 速度位置环模式
 *
 * @param motor 电机对象
 * @param pos 位置
 *  @arg 传入范围 `-36 000° ~ 36 000°`
 * @param spd 速度
 *  @arg 传入范围 `-32 768 ~ -32 767`, 对应转速 `-327 680 ~ -327 670 erpm`
 * @param rpa 加速度
 *  @arg 传入范围 `0 ~ 32 767`, 对应加速度 `0 ~ 327 670 erpm/s^2`
 */
void ak_servo_set_pos_spd(ak_motor_handle_t *motor, float pos, float spd,
                          float rpa) {
    if (motor == NULL) {
        return;
    }
    param_limit(&pos, -MAX_POSITION, MAX_POSITION);
    param_limit(&rpa, 0.0f, MAX_ACCELERATION);
    param_limit(&spd, MIN_POSITION_VELOCITY, MAX_POSITION);

    int32_t send_index = 0;
    uint8_t buffer[8];
    buffer_append_int32(buffer, (int32_t)(pos * 10000.0f), &send_index);
    buffer_append_int16(buffer, (int16_t)(spd / 10.0f), &send_index);
    buffer_append_int16(buffer, (int16_t)(rpa / 10.0f), &send_index);
    can_send_message(motor->can_select, CAN_ID_EXT,
                     canid_append_mode(motor->id, CAN_PACKET_SET_POS_SPD),
                     send_index, buffer);
}

/**
 * @}
 */

/******************************************************************************
 * @defgroup 运控模式驱动
 * @{
 */

/**
 * @brief 运控模式进入电机控制
 *
 * @param motor 电机对象
 * @attention 必须先进入控制模式才可以控制电机！
 */
void ak_mit_enter_motor(ak_motor_handle_t *motor) {
    if (motor == NULL) {
        return;
    }
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0XFC};
    can_send_message(motor->can_select, CAN_ID_STD, motor->id, 8, data);
}

/**
 * @brief 运控模式设置电机原点
 *
 * @param motor 电机对象
 */
void ak_mit_set_origin(ak_motor_handle_t *motor) {
    if (motor == NULL) {
        return;
    }
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0XFE};
    can_send_message(motor->can_select, CAN_ID_STD, motor->id, 8, data);
}

/**
 * @brief 让电机进入控制
 *
 * @param motor 电机对象
 * @param pos 电机位置
 * @param spd 电机速度
 * @param kp 运动比例系数
 * @param kd 运动阻尼系数
 * @param torque 扭矩
 * @attention 必须先进入控制模式才可以控制电机！
 */
void ak_mit_send_data(ak_motor_handle_t *motor, float pos, float spd, float kp,
                      float kd, float torque) {
    if (motor == NULL) {
        return;
    }
    /* 转换成整数 */
    int16_t pos_int =
        float_to_uint(pos, -AK_MIT_POSITION_LIMIT, AK_MIT_POSITION_LIMIT, 16);
    int16_t spd_int = float_to_uint(
        spd, -ak_mit_param_limit[motor->model][MIT_SPEED_LIMIT_INDEX],
        ak_mit_param_limit[motor->model][MIT_SPEED_LIMIT_INDEX], 12);
    int16_t kp_int = float_to_uint(kp, 0, AK_MIT_KP_LIMIT, 12);
    int16_t kd_int = float_to_uint(kd, 0, AK_MIT_KD_LIMIT, 12);
    int16_t torque_int = float_to_uint(
        torque, -ak_mit_param_limit[motor->model][MIT_TORQUE_LIMMIT_INDEX],
        ak_mit_param_limit[motor->model][MIT_TORQUE_LIMMIT_INDEX], 12);

    /* 填充缓冲区 */
    uint8_t data[8];
    data[0] = pos_int >> 8;   /* 位置高 8 位 */
    data[1] = pos_int & 0xFF; /* 位置低 8 位 */
    data[2] = spd_int >> 4;   /* 速度高 8 位 */
    data[3] =
        ((spd_int & 0xF) << 4) | (kp_int >> 8); /* 速度低 4 位，kp 高 4 位 */
    data[4] = kp_int & 0xFF;                    /* kp 低 8 位 */
    data[5] = kd_int >> 4;                      /* kd 高 8 位 */
    data[6] =
        ((kd_int & 0xF) << 4) | (torque_int >> 8); /* kp 低 4 位，扭矩高 4 位 */
    data[7] = torque_int & 0xFF;                   /* 扭矩低 8 位 */
    can_send_message(motor->can_select, CAN_ID_STD, motor->id, 8, data);
}

/**
 * @brief 让电机退出控制
 *
 * @param motor 电机对象
 */
void ak_mit_exit_motor(ak_motor_handle_t *motor) {
    if (motor == NULL) {
        return;
    }
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0XFD};
    can_send_message(motor->can_select, CAN_ID_STD, motor->id, 8, data);
}

/**
 * @}
 */
