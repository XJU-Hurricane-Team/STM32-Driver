/**
 * @file    as5047.h
 * @author  Deadline039
 * @brief   AS5047芯片驱动
 * @version 1.1
 * @date    2025-11-04
 * 
 ****************************************************************************
 *                     ##### 更新日志 ####
 * 
 *    Date    | Version |   Author    | Version Info
 * -----------+---------+-------------+----------------------------------------
 * 2024-10-20 |   1.0   | Deadline039 | 初版，全场定位码盘设计使用
 * 2025-11-04 |   1.1   | PickingChip | 适配战队项目
 */

#ifndef __AS5047_H
#define __AS5047_H

#include <CSP_Config.h>

/**
 * @brief AS5047位置记录
 */
typedef struct {
    uint16_t init_position;    /*!< 上电位置 */
    uint16_t current_position; /*!< 当前位置 */
    uint16_t last_position;    /*!< 上次位置 */
    int32_t round_cnt;         /*!< 圈数计数 */
    int64_t total_position;    /*!< 总角度 */
} as5047_position_t;

extern as5047_position_t g_as5047_position[];

/* 编码器分辨率 */
#define AS5047_RESOLUTION             14
/* 编码器一圈的计数 */
#define AS5047_CIRCLE_CNT             (1 << AS5047_RESOLUTION)
/* 编码器最大数注册数量 */
#define AS5047_MAX_NUM                2


#define AS5047_COMMAND_READ           0x4000

#define AS5047_NOP_VOL_REG_ADD        0x0000
#define AS5047_ERRFL_VOL_REG_ADD      0x0001
#define AS5047_PROG_VOL_REG_ADD       0x0003
#define AS5047_DIAAGC_VOL_REG_ADD     0x3ffc
#define AS5047_MAG_VOL_REG_ADD        0x3ffd
#define AS5047_ANGLEUNC_VOL_REG_ADD   0x3ffe
#define AS5047_ANGLECOM_VOL_REG_ADD   0x3fff
#define AS5047_ZPOSM_nVOL_REG_ADD     0x0016
#define AS5047_ZPOSL_nVOL_REG_ADD     0x0017
#define AS5047_SETTINGS1_nVOL_REG_ADD 0x0018
#define AS5047_SETTINGS2_nVOL_REG_ADD 0x0019
#define AS5047_RED_VOL_nREG_ADD       0x001a

void as5047_init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *port, uint16_t pin);
void as5047_reset(int devidx);

void as5047_set_zero(int devidx);
uint16_t as5047_get_zero(int devidx);
uint16_t as5047_get_absolute_position(int devidx);
void as5047_get_relative_position(int devidx);

uint16_t as5047_write_data(int devidx, uint16_t addr, uint16_t data);
uint16_t as5047_read_data(int devidx, uint16_t addr);

#endif /* __AS5047_H */
