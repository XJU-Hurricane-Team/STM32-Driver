/**
 * @file    damiao.h
 * @author  Deadline039
 * @brief   达妙电机驱动
 * @version 1.2
 * @date    2024-11-27
 ******************************************************************************
 *    Date    | Version |   Author    | Version Info
 * -----------+---------+-------------+----------------------------------------
 * 2024-11-27 |   1.0   | Deadline039 | 初版
 * 2026-4-13  |   1.1   | Jackrainman | 添加 PVT 模式
 * 2026-5-25  |   1.2   | Jackrainman | 添加读写寄存器相关功能
 */

#ifndef __DAMIAO_H
#define __DAMIAO_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include <cubemx.h>

/**
 * @brief 电机型号
 */
typedef enum {
    DM_J3507 = 0x00U,
    DM_J4310,
    DM_J4340,
    DM_J6006,
    DM_J8006,
    DM_J8009,
    DM_J10010,
    DM_S3519,
    DM_H6215,
    DM_G6220
} dm_model_t;

#define DM_KP_MIN 0.0f
#define DM_KP_MAX 500.0f
#define DM_KD_MIN 0.0f
#define DM_KD_MAX 5.0f

/**
 * @brief 故障信息
 */
typedef enum {
    DM_OK_DISABLED = 0x00U,      /*!< 无故障, 失能状态*/
    DM_OK_ENABLED,               /*!< 无故障, 始能状态 */
    DM_ERR_OVER_VOLTAGE = 0x08U, /*!< 过压错误 */
    DM_ERR_UNDER_VOLTAGE,        /*!< 欠压错误 */
    DM_ERR_OVER_CURRENT,         /*!< 过流错误 */
    DM_ERR_MOS_TEMPERATURE,      /*!< MOS 过温 */
    DM_ERR_MOTOR_TEMPERATURE,    /*!< 电机线圈过温 */
    DM_ERR_LOST_COMMUNICATION,   /*!< 通信丢失 */
    DM_ERR_OVER_LOAD             /*!< 过载 */
} dm_error_t;

/**
 * @brief 当前模式
 */
typedef enum {
    DM_MODE_MIT       = 0x01U,       /*!< MIT 控制模式 */
    DM_MODE_POS_SPEED = 0x02U,       /*!< 位置速度控制模式 */
    DM_MODE_SPEED     = 0x03U,       /*!< 速度控制模式 */
    DM_MODE_PVT       = 0x04U        /*!< 力位混控模式 */
} dm_mode_t;

/**
 * @brief 电机控制结构体
 */
typedef struct {
    uint32_t master_id;        /*!< 反馈主机 ID */
    uint32_t device_id;        /*!< 控制设备 ID */
    can_selected_t can_select; /*!< 选择 CAN 通信 */
    dm_model_t model;          /*!< 型号 */
    dm_mode_t mode;            /*!< 当前模式 */

    float position;          /*!< 位置 */
    float speed;             /*!< 速度 */
    float torque;            /*!< 扭矩 */
    float mos_temperature;   /*!< MOS 温度 */
    float motor_temperature; /*!< 电机线圈温度 */
    dm_error_t error;        /*!< 错误信息 */

    /* 以下参数需要与上位机设定值一致, 否则会导致回传与控制的值发送错误 */

    float pos_limit;  /*!< 位置绝对值范围 */
    float spd_limit;  /*!< 速度绝对值范围 */
    float torq_limit; /*!< 扭矩绝对值范围 */

    /* 寄存器读/写/存参 应答缓存 (can_callback 收到 MST_ID 帧时写入) */
    volatile uint8_t  reg_read_addr;    /*!< 应答里的 RID (0x33/0x55); 存参完成时是 0x01 */
    volatile uint32_t reg_read_value;   /*!< 应答里的 uint32 (LE 已解出); 存参时是 0 */
    volatile uint8_t  reg_read_pending; /*!< 1=请求已发但应答未到; 0=空闲或已到达 */
} dm_handle_t;

uint8_t dm_motor_init(dm_handle_t *motor, uint32_t master_id,
                      uint32_t device_id, dm_mode_t mode, dm_model_t model,
                      float pos_limit, float spd_limit, float torq_limit,
                      can_selected_t can_select);
uint8_t dm_motor_deinit(dm_handle_t *motor);
void dm_motor_enable(dm_handle_t *motor);
void dm_motor_disable(dm_handle_t *motor);
void dm_save_zero(dm_handle_t *motor);
void dm_clear_error(dm_handle_t *motor);

void dm_mit_ctrl(dm_handle_t *motor, float position, float speed, float kp,
                 float kd, float torque);
void dm_pos_speed_ctrl(dm_handle_t *motor, float position, float speed);
void dm_speed_ctrl(dm_handle_t *motor, float speed);
void dm_pvt_ctrl(
    dm_handle_t *motor, float position, float speed,
    float i_des); // 传入 speed 范围为 0 到 100, i_des 范围为 0 到 1

uint8_t dm_set_control_mode(dm_handle_t *motor, dm_mode_t mode);
uint8_t dm_write_register(dm_handle_t *motor, uint8_t reg_addr, uint32_t value);
uint8_t dm_read_register(dm_handle_t *motor, uint8_t reg_addr);
uint8_t dm_save_param(dm_handle_t *motor);


/**
 * @brief 可读写寄存器地址 (RW)
 *
 * 配合写参数功能码 0x55 使用。尾注格式: <type> <range> <描述>。
 * 写入后立即在 RAM 生效, 不写 Flash; 掉电恢复为上位机最后保存值。
 */
typedef enum {
    /* 保护参数 (0x00 ~ 0x03) */
    DM_REG_UV_VALUE  = 0x00, /*!< float  (10.0, fmax]  低压保护值 */
    DM_REG_KT_VALUE  = 0x01, /*!< float  [0.0, fmax]   扭矩系数 */
    DM_REG_OT_VALUE  = 0x02, /*!< float  [80.0, 200)   过温保护值 */
    DM_REG_OC_VALUE  = 0x03, /*!< float  (0.0, 1.0)    过流保护值 */

    /* 运动限值 (0x04 ~ 0x06) */
    DM_REG_ACC       = 0x04, /*!< float  (0.0, fmax)   加速度 */
    DM_REG_DEC       = 0x05, /*!< float  [-fmax, 0.0)  减速度 */
    DM_REG_MAX_SPD   = 0x06, /*!< float  (0.0, fmax]   最大速度 */

    /* 通信参数 (0x07 ~ 0x09) */
    DM_REG_MST_ID    = 0x07, /*!< uint32 [0, 0x7FF]    反馈 ID */
    DM_REG_ESC_ID    = 0x08, /*!< uint32 [0, 0x7FF]    接收 ID */
    DM_REG_TIMEOUT   = 0x09, /*!< uint32 [0, 2^32-1]   超时警报时间 */

    /* 控制模式 (0x0A) */
    DM_REG_CTRL_MODE = 0x0A, /*!< uint32 [1, 4]        1=MIT 2=POS_SPD 3=SPD 4=PVT */

    /* 位置/速度/扭矩映射 (0x15 ~ 0x17) */
    DM_REG_PMAX      = 0x15, /*!< float  (0.0, fmax]   位置映射范围 */
    DM_REG_VMAX      = 0x16, /*!< float  (0.0, fmax]   速度映射范围 */
    DM_REG_TMAX      = 0x17, /*!< float  (0.0, fmax]   扭矩映射范围 */

    /* PID / 控制环 (0x18 ~ 0x1C) */
    DM_REG_I_BW      = 0x18, /*!< float  [100.0, 1e4]  电流环控制带宽 */
    DM_REG_KP_ASR    = 0x19, /*!< float  [0.0, fmax]   速度环 Kp */
    DM_REG_KI_ASR    = 0x1A, /*!< float  [0.0, fmax]   速度环 Ki */
    DM_REG_KP_APR    = 0x1B, /*!< float  [0.0, fmax]   位置环 Kp */
    DM_REG_KI_APR    = 0x1C, /*!< float  [0.0, fmax]   位置环 Ki */

    /* 过压 / 速度环优化 (0x1D ~ 0x22) */
    DM_REG_OV_VALUE  = 0x1D, /*!< float  TBD           过压保护值 */
    DM_REG_GREF      = 0x1E, /*!< float  (0.0, 1.0]    齿轮力矩效率 */
    DM_REG_DETA      = 0x1F, /*!< float  [1.0, 30.0]   速度环阻尼系数 */
    DM_REG_V_BW      = 0x20, /*!< float  (0.0, 500.0)  速度环滤波带宽 */
    DM_REG_IQ_C1     = 0x21, /*!< float  [100.0, 1e4]  电流环增强系数 */
    DM_REG_VL_C1     = 0x22, /*!< float  (0.0, 1e4]    速度环增强系数 */

    /* CAN 波特率 (0x23) */
    DM_REG_CAN_BR    = 0x23, /*!< uint32 [0, 4]        CAN 波特率代码 */
} dm_reg_addr_t;

/**
 * @brief 只读寄存器地址 (RO)
 *
 * 仅 dm_read_register 可用。电调收到对 RO 地址的 0x55 写帧会被自身拒绝。
 */
typedef enum {
    /* 电机模型参数 (0x0B ~ 0x14) */
    DM_REG_RO_DAMP     = 0x0B, /*!< float   电机粘滞系数 */
    DM_REG_RO_INERTIA  = 0x0C, /*!< float   电机转动惯量 */
    DM_REG_RO_HW_VER   = 0x0D, /*!< uint32  硬件版本 (保留) */
    DM_REG_RO_SW_VER   = 0x0E, /*!< uint32  软件版本号 */
    DM_REG_RO_SN       = 0x0F, /*!< uint32  序列号 (保留) */
    DM_REG_RO_NPP      = 0x10, /*!< uint32  电机极对数 */
    DM_REG_RO_RS       = 0x11, /*!< float   电机相电阻 */
    DM_REG_RO_LS       = 0x12, /*!< float   电机相电感 */
    DM_REG_RO_FLUX     = 0x13, /*!< float   电机磁链值 */
    DM_REG_RO_GR       = 0x14, /*!< float   齿轮减速比 */

    /* 版本号 (0x24 ~ 0x25) */
    DM_REG_RO_SUB_VER  = 0x24, /*!< uint32  子版本号 */
    DM_REG_RO_BOOT_VER = 0x25, /*!< uint32  Boot 版本号 */

    /* 校准 / 监控 (0x37 ~ 0x41) */
    DM_REG_RO_DIR      = 0x37, /*!< float   方向 */
    DM_REG_RO_M_OFF    = 0x38, /*!< float   电机侧角度偏移 */
    DM_REG_RO_IMAX     = 0x3B, /*!< float   驱动板最大电流 */
    DM_REG_RO_VBUS     = 0x3C, /*!< float   电源电压 */
    DM_REG_RO_TPCB     = 0x3D, /*!< float   驱动板温度 */
    DM_REG_RO_TMTR     = 0x3E, /*!< float   电机温度 */
    DM_REG_RO_IU_OFF   = 0x3F, /*!< float   U 相电流偏置 */
    DM_REG_RO_IV_OFF   = 0x40, /*!< float   V 相电流偏置 */
    DM_REG_RO_IW_OFF   = 0x41, /*!< float   W 相电流偏置 */

    /* 位置反馈 (0x50 ~ 0x51) */
    DM_REG_RO_P_M      = 0x50, /*!< float   电机当前位置 */
    DM_REG_RO_XOUT     = 0x51, /*!< float   输出轴位置 */
} dm_reg_ro_addr_t;


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __DAMIAO_H */
