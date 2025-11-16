/**
 * @file    oled.h
 * @author  江协科技 (jxkj)
 * @brief   OLED 屏驱动代码
 * @version 2.1
 * @date    2023-11-22
 
 *****************************************************************************
 * Change Logs:
 * Date         Version     Author      Notes
 * 2023-11-22   1.0         jxkj        首次发布, 快速上手视频使用的版本
 * 2023-12-08   1.1         jxkj        初始化后, 加入了清屏的代码, 防止初始化后花屏; 更改了 oled_show_float 的执行逻辑, 修复了浮点数显示的 Bug
 * 2024-04-24   1.2         jxkj        将 x, y 坐标的类型由原来的 uint8_t 改成了 int16_t, 函数内部也对负数坐标做了相应的处理. 使用负数坐标, 可以实现显示内容平滑移入和移出屏幕的效果
 * 2024-10-20   2.0         jxkj        删除了 OLED_ShowChinese. 函数 oled_show_string 和 oled_printf 支持中英文混写
 * 2025-02-14   2.1         Deadline039 支持 \t \n \r 和自动换行
 */
#ifndef __OLED_H
#define __OLED_H

#include <CSP_Config.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* 定义 OLED 尺寸, 0.91/0.96/1.30 */
#define OLED_0_96
/* 定义通讯方式, I2C 或者 SPI */
#define OLED_USE_SPI

#if (defined(OLED_USE_SPI))
/* 片选引脚 */
#define OLED_CS_GPIO_PORT          GPIOB
#define OLED_CS_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOA_CLK_ENABLE()
#define OLED_CS_GPIO_PIN           GPIO_PIN_12
/* 数据 / 命令选择引脚 */
#define OLED_DC_GPIO_PORT          GPIOB
#define OLED_DC_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOB_CLK_ENABLE()
#define OLED_DC_GPIO_PIN           GPIO_PIN_13
/* 复位引脚 */
#define OLED_RES_GPIO_PORT         GPIOB
#define OLED_RES_GPIO_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()
#define OLED_RES_GPIO_PIN          GPIO_PIN_14
#endif /* OLED_USE_SPI */

#if (defined(OLED_0_91))
#define OLED_MAX_LINE 32
#else /* OLED_SIZE */
#define OLED_MAX_LINE 64
#endif /* OLED_SIZE */

#define OLED_MAX_COLUMN 128
#define OLED_MAX_PAGE   (OLED_MAX_LINE >> 3)

#define OLED_CMD        0    /* 写命令 */
#define OLED_DATA       1    /* 写数据 */
#define OLED_ADDRESS    0x78 /* OLED I2C 地址 */

/* FontSize 参数取值
 * 此参数值不仅用于判断, 而且用于计算横向字符偏移, 默认值为字体像素宽度 */
#define OLED_8X16       8
#define OLED_6X8        6

/* 制表符占的字符长度 */
#define OLED_TAB_SIZE   4

/* IsFilled 参数数值 */
#define OLED_UNFILLED   0
#define OLED_FILLED     1

/* 公开图片数据 */
extern const uint8_t gc_image[];

void oled_init(void);
void oled_on(void);
void oled_off(void);

void oled_update(void);
void oled_update_area(int16_t x, int16_t y, uint8_t width, uint8_t height);

void oled_clear(void);
void oled_clear_area(int16_t x, int16_t y, uint8_t width, uint8_t height);
void oled_reserve(void);
void oled_reserve_area(int16_t x, int16_t y, uint8_t width, uint8_t height);

void oled_show_char(int16_t x, int16_t y, char ch, uint8_t font_size);
void oled_show_string(int16_t x, int16_t y, char *str, uint8_t font_size);
void oled_show_number(int16_t x, int16_t y, uint32_t number, uint8_t length,
                      uint8_t font_size);
void oled_show_signed_number(int16_t x, int16_t y, int32_t number,
                             uint8_t length, uint8_t font_size);
void oled_show_hex(int16_t x, int16_t y, uint32_t number, uint8_t length,
                   uint8_t font_size);
void oled_show_bin(int16_t x, int16_t y, uint32_t number, uint8_t length,
                   uint8_t font_size);
void oled_show_float(int16_t x, int16_t y, double number, uint8_t int_length,
                     uint8_t float_length, uint8_t font_size);
void oled_show_image(int16_t x, int16_t y, uint8_t width, uint8_t height,
                     const uint8_t *image);
void oled_printf(int16_t x, int16_t y, uint8_t font_size, char *format, ...);

void oled_draw_point(int16_t x, int16_t y);
uint8_t oled_get_point(int16_t x, int16_t y);
void oled_draw_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1);
void oled_draw_rectangle(int16_t x, int16_t y, uint8_t width, uint8_t height,
                         uint8_t is_filled);
void oled_draw_tritangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1,
                         int16_t x2, int16_t y2, uint8_t is_filled);
void oled_draw_circle(int16_t x, int16_t y, uint8_t radius, uint8_t is_filled);
void oled_draw_ellipse(int16_t x, int16_t y, uint8_t a, uint8_t b,
                       uint8_t is_filled);
void oled_draw_arc(int16_t x, int16_t y, uint8_t radius, int16_t start_angle,
                   int16_t end_angle, uint8_t is_filled);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __OLED_H */
