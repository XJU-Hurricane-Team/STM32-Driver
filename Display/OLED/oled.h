/**
 * @file    oled.h
 * @author  Deadline039
 * @brief   OLED屏驱动代码
 * @version 0.1
 * @date    2023-11-05
 */
#ifndef __OLED_H
#define __OLED_H

#include "./core/core_delay.h"
#include "./iic/iic.h"
#include "math.h"
#include "CSP_Config.h"

#define OLED_0_91
#define MAX_COLUMN 128

#define OLED_CMD   0 /* 写命令 */
#define OLED_DATA  1 /* 写数据 */

void oled_init(void);
void oled_wr_byte(unsigned dat, unsigned cmd);
void oled_display_on(void);
void oled_display_off(void);
void oled_clear(void);
void oled_show_char(uint8_t x, uint8_t y, uint8_t chr, uint8_t size);
void oled_show_num(uint8_t x, uint8_t y, uint32_t num, uint8_t len,
                   uint8_t size);
void oled_show_string(uint8_t x, uint8_t y, const char *chars, uint8_t size);
void oled_set_position(uint8_t x, uint8_t y);

#endif /* __OLED_H */
