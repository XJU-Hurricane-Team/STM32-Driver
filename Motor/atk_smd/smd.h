/**
 ****************************************************************************************************
 * @file        smd.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.1
 * @date        2026-04-18
 * @brief       步进电机驱动器 控制指令代码
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 */
#ifndef __SMD_H
#define __SMD_H

#include "stdbool.h"
#include "stdint.h"

/* 消息解析宏 */
#define COMM_TYPE  0      /* 0串口通信， 1 can通信 */
#define CAN_EXTID  0x1000 /* 发送帧的can扩展帧ID，支持0x100x */

#define FRAME_HEAD 0xC5 /* 帧头 */
#define FRAME_TAIL 0x5C /* 帧尾 */

/* SMD */
#define SMD_INFO_BUF_SIZE   256 /* 电机信息文本缓存大小 */

#define SMD_MASK_INFO        (1UL << 0)
#define SMD_MASK_LAST_ERROR  (1UL << 1)
#define SMD_MASK_PULSE_CNT   (1UL << 2)
#define SMD_MASK_POS_ERR     (1UL << 3)
#define SMD_MASK_BUS_VOLT    (1UL << 4)
#define SMD_MASK_SPEED_RPM   (1UL << 5)
#define SMD_MASK_REAL_POS    (1UL << 6)
#define SMD_MASK_TARGET_POS  (1UL << 7)
#define SMD_MASK_MOTOR_STA   (1UL << 8)
#define SMD_MASK_ENABLE_STA  (1UL << 9)
#define SMD_MASK_ARRIVED_STA (1UL << 10)
#define SMD_MASK_CLOG_FLAG   (1UL << 11)

#define MOTOR_NUM_MAX 10     /* 最多支持电机数量 */

/* 功能码定义 */
typedef enum {
    /* 系统指令：(0x00~0x0F) */
    FCT_IDLE = 0x00,          /* 空闲功能码 */
    FCT_CAL_ENCODER = 0x01,   /* 校准编码器 */
    FCT_RESTART = 0x02,       /* 复位重启 */
    FCT_RESET_FACTORY = 0x03, /* 恢复出厂设置 */
    FCT_PARAM_SAVE = 0x04,    /* 参数保存 */

    /* 读参数指令：(0x20~0x3F) */
    FCT_READ_SOFT_HARD_VER = 0x20, /* 读取软硬件版本信息 */
    FCT_READ_PSI = 0x21,           /* 读取电机磁链 */
    FCT_READ_PHASE_RES_IND = 0x22, /* 读取相电阻和相电感 */
    FCT_READ_PHASE_MA = 0x23,      /* 读取相电流 */
    FCT_READ_VOL = 0x24,           /* 读取总线电压 */
    FCT_READ_MA_PID = 0x25,        /* 读取电流环PID参数 */
    FCT_READ_SPEED_PID = 0x26,     /* 读取速度环PID参数 */
    FCT_READ_POS_PID = 0x27,       /* 读取位置环PID参数 */
    FCT_READ_TOTAL_PULSE = 0x28,   /* 读取输入累计脉冲数 */
    FCT_READ_ROTATE_SPEED = 0x29,  /* 读电机实时转速 */
    FCT_READ_POS = 0x2A,           /* 读取电机实时位置 */
    FCT_READ_POS_ERROR = 0x2B,     /* 读取电机位置误差 */
    FCT_READ_MOTOR_STA = 0x2C,     /* 读取电机运行状态 */
    FCT_READ_CLOG_FLAG = 0x2D,     /* 读取堵转标志 */
    FCT_READ_CLOG_CUR = 0x2E,      /* 读取堵转电流 */
    FCT_READ_ENABLE_STA = 0x2F,    /* 读使能状态 */
    FCT_READ_ARRIVED_STA = 0x30,   /* 读取到位状态 */
    FCT_READ_SYS_PARAM = 0x31,     /* 读取系统参数 */
    FCT_READ_DRIVE_PARAMS = 0x32,  /* 读取驱动参数 */

    /* 设置参数指令：(0x60~0x7F) */
    FCT_SET_SLAVE_ADD = 0x60,        /* 设置从机地址 */
    FCT_SET_GROUP_ADD = 0x61,        /* 设置分组地址 */
    FCT_SET_MODE = 0x62,             /* 设置工作模式 */
    FCT_SET_POS_PID = 0x63,          /* 设置位置环PID */
    FCT_SET_POS_TORQUE = 0x64,       /* 设置位置环最大力矩限制 */
    FCT_SET_STEP = 0x65,             /* 设置细分 */
    FCT_SET_MA = 0x66,               /* 设置目标电流 */
    FCT_SET_UART_BAUD = 0x67,        /* 设置串口波特率 */
    FCT_SET_CAN_BAUD = 0x68,         /* 设置CAN波特率 */
    FCT_SET_MODBUS = 0x69,           /* 设置MODBUS协议 */
    FCT_SET_CLOG_PRO = 0x6A,         /* 设置堵转保护 */
    FCT_SET_CLOG_CUR = 0x6B,         /* 设置堵转电流 */
    FCT_SET_CAN_ID = 0x6C,           /* 设置CAN_ID */
    FCT_SET_DIR_LEVEL = 0x6D,        /* 设置DIR正转电平 */
    FCT_SET_EN_LEVEL = 0x6E,         /* 设置EN脚有效电平 */
    FCT_SET_CMD_ECHO = 0x6F,         /* 设置指令回响 */
    FCT_SET_KEY_LOCK = 0x70,         /* 设置按键锁定 */
    FCT_SET_AUTO_NOT_DISPLAY = 0x71, /* 设置自动熄屏 */
    FCT_SET_IO_START_LEVEL = 0x72,   /* 设置IO启动电平 */
    FCT_SET_SPEED_PID = 0x73,        /* 设置速度环PID */

    /* 限位回零相关指令：(0x90~0x9F) */
    FCT_ORIGIN_SET_LEFT_POS = 0x90,  /* 设左限位原点位置 */
    FCT_ORIGIN_LIMIT_HOME = 0x91,    /* 有无限位回零 */
    FCT_ORIGIN_TRIG = 0x92,          /* 触发回零 */
    FCT_ORIGIN_BREAK = 0x93,         /* 强制中断并退出回零操作 */
    FCT_ORIGIN_READ_PARAMS = 0x94,   /* 读取回零参数 */
    FCT_ORIGIN_SET_PARAMS = 0x95,    /* 修改原点回零超时时间 */
    FCT_ORIGIN_READ_STA = 0x96,      /* 读取回零状态 */
    FCT_ORIGIN_AOTO_ZERO = 0x97,     /* 上电自动回零设置 */
    FCT_ORIGIN_SET_RIGHT_POS = 0x98, /* 设右限位原点位置 */
    FCT_ORIGIN_SWITCH = 0x99,        /* 左右限位开关 */

    /* 运动控制相关指令：(0xE0~0xFF) */
    FCT_OL_SPEED_MODE = 0xE0,   /* 开环速度模式控制 */
    FCT_OL_POS_MODE = 0xE1,     /* 开环绝对位置模式控制 */
    FCT_OL_POS_REL_MODE = 0xE2, /* 开环相对位置模式控制 */
    FCT_OL_PULSES_MODE = 0xE3,  /* 开环脉冲模式 */

    FCT_IO_RUN_MODE = 0xE4, /* IO启停模式 */

    FCT_TORQUE_MODE = 0xF0,            /* 力矩模式控制 */
    FCT_SPEED_MODE = 0xF1,             /* 速度模式控制 */
    FCT_POS_MODE = 0xF2,               /* 绝对位置模式控制 */
    FCT_POS_REL_MODE = 0xF3,           /* 相对位置模式控制 */
    FCT_PULSES_MODE = 0xF4,            /* 脉冲模式 */
    FCT_PULSE_WIDTH_POS_MODE = 0xF5,   /* 脉宽位置模式 */
    FCT_PULSE_WIDTH_MA_MODE = 0xF6,    /* 脉宽电流模式 */
    FCT_PULSE_WIDTH_SPEED_MODE = 0xF7, /* 脉宽速度模式 */
    FCT_ANGLE_ZERO = 0xF8,             /* 将当前的位置清零 */
    FCT_CLEAR_CLOG_PRO = 0xF9,         /* 解除堵转状态 */
    FCT_MOTOR_ENABLE = 0xFA,           /* 电机使能控制 */
    FCT_CLEAR_STATE = 0xFB,            /* 清除状态（堵转、刹车，失能） */
    FCT_STOP_NOW = 0xFC,               /* 立即停止（刹车） */
} FUN_CODE_TYPE;

/* 解析帧结构 */
typedef struct {
    uint8_t slave_addr;    /* 从机地址 */
    uint8_t function_code; /* 功能码 */
    uint8_t error_code;    /* 错误码 */
    uint8_t data[128];     /* 指令数据缓冲区 */
    uint8_t data_len;      /* 指令数据长度 */
    uint16_t checksum;     /* 校验和 */
} SERIAL_FRAME;

typedef enum {
    ACK_SUCCEED = 0x01,              /* 应答成功 */
    ACK_FRAME_TOO_SHORT = 0xE1,      /* 帧长度不足（小于最小帧长度） */
    ACK_INVALID_HEADER = 0xE2,       /* 帧头错误（非0xC5） */
    ACK_INVALID_FOOTER = 0xE3,       /* 帧尾错误（非0x5C） */
    ACK_CHECKSUM_MISMATCH = 0xE4,    /* 校验和错误 */
    ACK_UNSUPPORTED_FUNCTION = 0xE5, /* 不支持的功能码 */
    ACK_ERR_ILLEGAL_VAL = 0xE6       /* 数据不合法 */
} ACK_STA;

/**
 * @brief 步进电机结构体
 * @note  valid_mask 用于标记“本次解析有更新”的字段（1=本次有更新，0=本次未更新）
 *
 *        bit[0]  : info        文本消息缓存有更新
 *        bit[1]  : last_error  错误码有更新
 *        bit[2]  : pulse_cnt   累计脉冲数有更新
 *        bit[3]  : pos_err     位置误差有更新
 *        bit[4]  : bus_volt    总线电压有更新
 *        bit[5]  : speed_rpm   转速有更新
 *        bit[6]  : real_pos    实时位置有更新
 *        bit[7]  : target_pos  目标位置有更新
 *        bit[8]  : motor_sta   电机状态有更新
 *        bit[9]  : enable_sta  使能状态有更新
 *        bit[10] : arrived_sta 到位状态有更新
 *        bit[11] : clog_flag   堵转标志有更新
 *
 *        字段取值说明：
 *        - motor_sta   : 0 空闲, 1 已完成, 2 运行中, 3 过载, 4 堵转, 5 欠压
 *        - enable_sta  : 0 使能, 1 失能
 *        - arrived_sta : 0 未到位, 1 到位
 *        - clog_flag   : 0 未堵转, 1 堵转
 */

typedef struct {
    uint8_t slave_addr;  /* 电机地址同id */
    int16_t speed_rpm;   /* 电机转速 */
    int32_t real_pos;    /* 电机位置 */
    int32_t target_pos;  /* 目标位置 */
    int32_t pos_err;     /* 位置误差 */
    int32_t pulse_cnt;   /* 累计脉冲数 */
    float bus_volt;      /* 总线电压 */
    uint8_t motor_sta;   /* 电机状态 */
    uint8_t enable_sta;  /* 使能状态 */
    uint8_t arrived_sta; /* 到位状态 */
    uint8_t clog_flag;   /* 堵转标志 */
    uint8_t last_error;  /* 最后一次错误码 */
    uint8_t *info;       /* 电机信息缓冲区 */
    uint32_t valid_mask; /* 有效字段掩码 */
} smd_motor_t;

/**********************************************************
              步进电机驱动器指令接口函数声明
**********************************************************/
void smd_cal_encoder(uint8_t addr);
void smd_restart(uint8_t addr);
void smd_reset_factory(uint8_t addr);
void smd_param_save(uint8_t addr);

void smd_read_soft_hard_ver(uint8_t addr);
void smd_read_psi(uint8_t addr);
void smd_read_phase_res_ind(uint8_t addr);
void smd_read_phase_ma(uint8_t addr);
void smd_read_vol(uint8_t addr);
void smd_read_ma_pid(uint8_t addr);
void smd_read_speed_pid(uint8_t addr);
void smd_read_pos_pid(uint8_t addr);
void smd_read_tatal_pulse(uint8_t addr);
void smd_read_rotate_speed(uint8_t addr);
void smd_read_pos(uint8_t addr);
void smd_read_pos_error(uint8_t addr);
void smd_read_motor_sta(uint8_t addr);
void smd_read_clog_flag(uint8_t addr);
void smd_read_clog_current(uint8_t addr);
void smd_read_enable_sta(uint8_t addr);
void smd_read_arrived_sta(uint8_t addr);
void smd_read_sys_params(uint8_t addr);
void smd_read_drive_params(uint8_t addr);

void smd_set_slave_add(uint8_t addr, uint8_t new_addr);
void smd_set_group_add(uint8_t addr, uint8_t new_addr);
void smd_set_mode(uint8_t addr, uint8_t mode);
void smd_set_pos_pid(uint8_t addr, uint32_t kp, uint32_t ki, uint32_t kd);
void smd_set_pos_torque(uint8_t addr, int16_t torque);
void smd_set_step(uint8_t addr, uint16_t step);
void smd_set_ma(uint8_t addr, int16_t ma);
void smd_set_uart_baud(uint8_t addr, uint32_t baud);
void smd_set_can_baud(uint8_t addr, uint16_t baud);
void smd_set_modbus(uint8_t addr, uint8_t modbus);
void smd_set_clog_pro(uint8_t addr, uint8_t en);
void smd_set_clog_current(uint8_t addr, int16_t ma);
void smd_set_can_id(uint8_t addr, uint32_t id);
void smd_set_dir_level(uint8_t addr, uint8_t dir);
void smd_set_en_level(uint8_t addr, uint8_t en);
void smd_set_cmd_echo(uint8_t addr, uint8_t echo);
void smd_set_key_lock(uint8_t addr, uint8_t lock);
void smd_set_auto_not_display(uint8_t addr, uint8_t en);
void smd_set_io_start_level(uint8_t addr, uint8_t level);
void smd_set_speed_pid(uint8_t addr, uint32_t kp, uint32_t ki, uint32_t kd);

void smd_origin_set_left_pos(uint8_t addr, int32_t pos);
void smd_origin_homing_by_limit(uint8_t addr, uint8_t limit_enable, uint8_t dir,
                                int32_t speed_rpm, int16_t curr_limit);
void smd_origin_trig(uint8_t addr, uint8_t mode);
void smd_origin_break(uint8_t addr);
void smd_origin_read_params(uint8_t addr);
void smd_origin_set_params(uint8_t addr, uint32_t timout);
void smd_origin_read_sta(uint8_t addr);
void smd_origin_aoto_zero(uint8_t addr, uint8_t flag);
void smd_origin_set_right_pos(uint8_t addr, int32_t pos);
void smd_origin_l_r_switch(uint8_t addr, uint8_t ctrl);

void smd_torque_mode(uint8_t addr, uint8_t dir, uint16_t current);
void smd_speed_mode(uint8_t addr, uint8_t dir, uint8_t acc, float speed);
void smd_pos_mode(uint8_t addr, uint8_t dir, uint8_t acc, uint16_t speed,
                  uint32_t pulses);
void smd_pos_rel_mode(uint8_t addr, uint8_t dir, uint8_t acc, uint16_t speed,
                      uint32_t pulses);
void smd_pulse_mode(uint8_t addr);
void smd_pulse_width_pos_mode(uint8_t addr, uint16_t topw_max,
                              uint16_t topw_min, int32_t top_pos,
                              int32_t down_pos);
void smd_pulse_width_ma_mode(uint8_t addr, uint16_t topw_max, uint16_t topw_min,
                             int32_t top_ma, int32_t down_ma);
void smd_pulse_width_speed_mode(uint8_t addr, uint16_t topw_max,
                                uint16_t topw_min, int32_t top_speed,
                                int32_t down_speed);
void smd_ol_speed_mode(uint8_t addr, uint8_t dir, uint8_t acc, float speed);
void smd_ol_pos_mode(uint8_t addr, uint8_t dir, uint8_t acc, uint16_t speed,
                     uint32_t pulses);
void smd_ol_pos_rel_mode(uint8_t addr, uint8_t dir, uint8_t acc, uint16_t speed,
                         uint32_t pulses);
void smd_ol_pulse_mode(uint8_t addr);
void smd_io_run_ctrl(uint8_t addr, uint8_t dir, uint8_t acc, float speed);

void smd_angle_to_zero(uint8_t addr);
void smd_remove_clog_protect(uint8_t addr);
void smd_motor_enable(uint8_t addr, uint8_t en);
void smd_clear_sta(uint8_t addr);
void smd_stop_now(uint8_t addr);

uint8_t smd_checksum(const uint8_t *data, uint8_t length);

/* 电机初始化函数 */
uint8_t smd_motor_init(smd_motor_t *motor, uint8_t motor_id);
/* 帧解析函数 */
bool serial_frame_process(uint8_t *buffer, uint8_t len, SERIAL_FRAME *frame);

#endif
