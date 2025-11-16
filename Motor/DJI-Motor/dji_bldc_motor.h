/**
 * @file    dji_bldc_motor.h
 * @author  Deadline039
 * @brief   M3508, M2006 直流无刷电机驱动
 * @version 1.5
 * @date    2024-03-02
 *
 ******************************************************************************
 *    Date    | Version |   Author    | Version Info
 * -----------+---------+-------------+----------------------------------------
 * 2024-03-02 |   1.0   | Deadline039 | 初版
 * 2024-03-16 |   1.1   | Deadline039 | 添加 GM6020 代码
 * 2024-04-04 |   1.2   | Deadline039 | 添加 CAN2 代码，两个 CAN 可以同时使用
 * 2024-04-13 |   1.3   | Deadline039 | 添加转子绝对位置 (rotor_degree)
 * 2024-08-13 |   1.4   | Deadline039 | 移除缺省参，添加电机型号宏开关
 * 2024-11-30 |   1.5   | Deadline039 | 移除专用回调函数，统一使用 can_list 回调
 */

#ifndef __DJI_BLDC_MOTOR_H
#define __DJI_BLDC_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include <CSP_Config.h>

#include <stdbool.h>

/* 是否使用 M3508 或者 M2006 */
#define DJI_MOTOR_USE_M3508_2006 1
/* 是否使用 GM6020 */
#define DJI_MOTOR_USE_GM6020     1

#if (DJI_MOTOR_USE_M3508_2006 == 1)

#define DJI_MOTOR_GROUP1 0x200 /* M3508/2006 标识符 */
#define DJI_MOTOR_GROUP2 0x1FF /* M3508/2006 标识符 */

#endif /* DJI_MOTOR_USE_M3508_2006 == 1 */

#if (DJI_MOTOR_USE_GM6020 == 1)

#define DJI_GM6020_VOLTAGE_GROUP1 0x1FF /* GM6020 电压控制标识符 */
#define DJI_GM6020_VOLTAGE_GROUP2 0x2FF /* GM6020 电压控制标识符 */

#define DJI_GM6020_CURRENT_GROUP1 0x1FE /* GM6020 电流控制标识符 */
#define DJI_GM6020_CURRENT_GROUP2 0x2FE /* GM6020 电流控制标识符 */

#endif /* DJI_MOTOR_USE_GM6020 == 1 */

/**
 * @brief 电机型号
 */
typedef enum {
    DJI_M3508 = 0x00, /*!< M3508 电机 */
    DJI_M2006 = 0x01, /*!< M2006 电机 */
    DJI_GM6020 = 0x02 /*!< GM6020 电机 */
} dji_motor_model_t;

/**
 * @brief CAN ID 定义
 * @note GM6020 与 M3508/2006 公用 Motor5-8 的 ID
 */
typedef enum {
#if (DJI_MOTOR_USE_M3508_2006 == 1)
    CAN_Motor1_ID = 0x201,
    CAN_Motor2_ID,
    CAN_Motor3_ID,
    CAN_Motor4_ID,
    CAN_Motor5_ID,
    CAN_Motor6_ID,
    CAN_Motor7_ID,
    CAN_Motor8_ID,
#endif /* DJI_MOTOR_USE_M3508_2006 == 1 */

#if (DJI_MOTOR_USE_GM6020 == 1)
    CAN_GM6020_ID1 = 0x205,
    CAN_GM6020_ID2,
    CAN_GM6020_ID3,
    CAN_GM6020_ID4,
    CAN_GM6020_ID5,
    CAN_GM6020_ID6,
    CAN_GM6020_ID7
#endif /* DJI_MOTOR_USE_GM6020 == 1 */
} dji_can_id_t;

/**
 * @brief 电机参数结构体
 */
typedef struct {

#if (DJI_MOTOR_USE_M3508_2006 == 1)

    /* 3508/2006 参数 */
    float real_current;    /*!< 实际电流 */
    int16_t given_current; /*!< 期望电流，M3508 电机才会赋值 */

#endif /* DJI_MOTOR_USE_M3508_2006 == 1 */

#if (DJI_MOTOR_USE_GM6020 == 1)

    /* GM6020 参数 */
    int16_t torque_current; /*!< 实际转矩电流 */
    uint8_t temperature;    /*!< 温度 */

#endif /* DJI_MOTOR_USE_GM6020 == 1 */

    /* 共用参数 */
    uint8_t hall; /*!< 可能是霍尔传感器值 */

    bool got_offset;       /*!< 上电以后获取一次角度偏移 */
    uint16_t offset_angle; /*!< 上电后角度初始位置 */

    uint16_t last_angle; /*!< 上次角度 */
    uint16_t angle;      /*!< 角度，绝对角度，一圈为 8192 */
    int32_t total_angle; /*!< 上电以后为 0 点，以此为基准的总角度 */
    int32_t round_cnt;  /*!< 圈数计数 */
    float rotor_degree; /*!< 转子角度
                             对于 3508 与 2006, 是轴的相对位置.
                             上电后为 0 度，轴转一圈为 360, 0 (360)
                             度附近不会跳变. 角度会累加，已经除过减速比； 对于
                             6020, 是绝对位置 (0 ~ 360). 上电后不为 0,
                             角度不会累加，0 (360) 度附近会跳变. */

    int16_t set_value; /*!< 设置的值，电压或电流值 */
    int16_t speed_rpm; /*!< 速度 */

    dji_can_id_t motor_id;         /*!< 电机 ID */
    dji_motor_model_t motor_model; /*!< 电机型号 */
    can_selected_t can_select;     /*!< 选择 CAN 通信 */
} dji_motor_handle_t;

uint8_t dji_motor_init(dji_motor_handle_t *motor, dji_motor_model_t motor_model,
                       dji_can_id_t can_id, can_selected_t can_select);
uint8_t dji_motor_deinit(dji_motor_handle_t *motor);

#if (DJI_MOTOR_USE_M3508_2006 == 1)
void dji_motor_set_current(can_selected_t can_select, uint16_t can_identify,
                           int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);
#endif /* DJI_MOTOR_USE_M3508_2006 == 1 */

#if (DJI_MOTOR_USE_GM6020 == 1)
void dji_gm6020_voltage_control(can_selected_t can_select,
                                uint16_t can_identify, int16_t voltage1,
                                int16_t voltage2, int16_t voltage3,
                                int16_t voltage4);

void dji_gm6020_current_control(can_selected_t can_select,
                                uint16_t can_identify, int16_t current1,
                                int16_t current2, int16_t current3,
                                int16_t current4);
#endif /* DJI_MOTOR_USE_GM6020 == 1 */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __DJI_BLDC_MOTOR_H */
