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

#include <cubemx.h>

#include <stdbool.h>
#include <stdint.h>

/* 是否使用 M3508 或者 M2006 */
#define DJI_MOTOR_USE_M3508_2006 1
/* 是否使用 GM6020 */
#define DJI_MOTOR_USE_GM6020     1

#if (DJI_MOTOR_USE_M3508_2006 == 1)

#define DJI_MOTOR_GROUP1 0x200 /* M3508/2006 标识符 */
#define DJI_MOTOR_GROUP2 0x1FF /* M3508/2006 标识符 */

#define GEAR_RATIO_M3508 (3591.0f / 187.0f)       /* 3508 正常减速比为 1:19 */
#define GEAR_RATIO_M2006 36.0f                  /* 2006 正常减速比为 1:36*/

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

/* 省略 offset 时使用首帧零点，保留此值作为自动归零标记。 */
#define DJI_OFFSET_AUTO INT32_MIN

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
    float real_current;    /*!< 转矩电流 Iq（A），3508: ±20 A，2006: ±10 A */

#endif /* DJI_MOTOR_USE_M3508_2006 == 1 */

#if (DJI_MOTOR_USE_GM6020 == 1)

    /* GM6020 参数 */
    int16_t torque_current; /*!< 实际转矩电流（原始值，±16384 对应 ±3 A） */

#endif /* DJI_MOTOR_USE_GM6020 == 1 */

    /* 共用参数 */
    uint8_t hall; /*!< 保留字段，原 DATA[6]，目前不赋值 */

    bool zero_from_first_frame; /*!< 是否把上电首帧位置当作 0 位 */
    bool got_offset;       /*!< 首帧角度与计圈状态已初始化 */
    uint16_t offset_angle; /*!< 零位对应的编码器计数，范围 0 ~ 8191 */

    uint16_t last_angle; /*!< 上一次回调的转子单圈计数 */
    uint16_t angle;      /*!< 当前回调的转子单圈编码器计数，0 ~ 8191 */
    int32_t total_angle; /*!< 相对零位的转子累计编码器计数 */
    int32_t round_cnt;   /*!< 转子圈数计数 */
    float rotor_degree;  /*!< 输出轴角度
                              3508/2006：相对首帧回调读到数据的累计角，已除减速比；
                              6020：显式 offset 时为相对该零位的单圈角 [0, 360)，
                              省略 offset 时把上电首帧获取的数据当作 0 位并累计 */

    int16_t set_value; /*!< 设置的值（电流或电压） */
    int16_t speed_rpm; /*!< 转子转速（RPM）；3508/2006 需除减速比得输出轴转速 */

    uint8_t temperature;    /*!< 温度（℃），3508/6020 有效，2006 无反馈 */
    dji_can_id_t motor_id;  /*!< 电机 ID */
    dji_motor_model_t motor_model; /*!< 电机型号 */
    can_selected_t can_select;     /*!< 选择 CAN 通信 */
} dji_motor_handle_t;

uint8_t dji_motor_init_impl(dji_motor_handle_t *motor, dji_motor_model_t motor_model,
                           dji_can_id_t can_id, can_selected_t can_select,
                           int32_t offset);

/* 四参数默认首帧归零，五参数指定编码器零位 offset，单位为计数。
 * 零位位于编码器原点 + offset，反馈角度扣除 offset；传 0 使用编码器原点。
 * 取前五个参数；末尾的 0 保证辅助宏的可变参数非空，兼容 C99。 */
#define DJI_INIT_ARGS(motor, model, id, can, offset, ...) \
    motor, model, id, can, offset
#define dji_motor_init(...) \
    dji_motor_init_impl(DJI_INIT_ARGS(__VA_ARGS__, DJI_OFFSET_AUTO, 0))

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
