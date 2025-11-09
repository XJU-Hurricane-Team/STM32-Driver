/**
 * @file st7735.h
 * @author PickingChip
 * @brief 
 * @version 0.1
 * @date 2025-11-09
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef _ST7735_H_
#define _ST7735_H_

#include "bsp.h"

typedef struct {
    const uint8_t width;
    const uint8_t height;
    const uint32_t *data;
} FontDef;

typedef struct {
    uint16_t width;  /* 屏幕的宽度，随显示方向改变 */
    uint16_t height; /* 屏幕的高度，随显示方向改变 */
    uint8_t dir;     /* 显示的方向，取值0~3，对应屏幕逆时针旋转0°~270° */
} lcd_dev_t;

extern const FontDef Font_7x10;
extern const FontDef Font_11x18;
extern const FontDef Font_Custom;

/*************************************************************************************************/
/* 引脚定义 */
#define ST7735_RST_Pin       GPIO_PIN_14 /*!<硬件复位引脚 */
#define ST7735_RST_GPIO_Port GPIOB
#define ST7735_DC_Pin        GPIO_PIN_13 /*!<数据(1)/命令(0)选择引脚 */
#define ST7735_DC_GPIO_Port  GPIOB
#define ST7735_CS_Pin        GPIO_PIN_12 /*!<片选引脚 */
#define ST7735_CS_GPIO_Port  GPIOB

#define ST7735_SPI_INSTANCE  spi1_handle

/*************************************************************************************************/
/* 显示设置 */
#define ST7735_XSTART        0 /*!<显示区域设置 */
#define ST7735_YSTART        0
#define ST7735_WIDTH         130 /* 横向分辨率 */
#define ST7735_HEIGHT        162 /* 纵向分辨率 */
#define ST7735_MAX_WIDTH                                                       \
    ((ST7735_WIDTH) > (ST7735_HEIGHT) ? (ST7735_WIDTH) : (ST7735_HEIGHT))

#define ST7735_ROTATION    1 /*!<屏幕显示方式*/

#define ST7735_MADCTL_RGB  0x00 /*!<Color Mode: RGB or BGR */
#define ST7735_MADCTL_BGR  0x08
#define ST7735_MADCTL_MODE ST7735_MADCTL_RGB

#define ST7735_INVERSE     0 /*!< Color Inverse: 0=NO, 1=YES */

/*************************************************************************************************/
/* 颜色定义 */
#define ST7735_BLACK       0x0000
#define ST7735_BLUE        0x001F
#define ST7735_RED         0xF800
#define ST7735_GREEN       0x07E0
#define ST7735_CYAN        0x07FF
#define ST7735_MAGENTA     0xF81F
#define ST7735_YELLOW      0xFFE0
#define ST7735_WHITE       0xFFFF
#define ST7735_COLOR565(r, g, b)                                               \
    (((r & 0xF8) << 8) | ((g & 0xFC) << 3) | ((b & 0xF8) >> 3))

/*************************************************************************************************/
/* IC指令 */
#define ST7735_SLPOUT    0x11 /* 退出睡眠 */
#define ST7735_FRMCTR1   0xB1
#define ST7735_FRMCTR2   0xB2
#define ST7735_FRMCTR3   0xB3
#define ST7735_INVCTR    0xB4
#define ST7735_PWCTR1    0xC0
#define ST7735_PWCTR2    0xC1
#define ST7735_PWCTR3    0xC2
#define ST7735_PWCTR4    0xC3
#define ST7735_PWCTR5    0xC4
#define ST7735_VMCTR1    0xC5
#define ST7735_COLMOD    0x3A /* 颜色格式设置 */
#define ST7735_GMCTRP1   0xE0
#define ST7735_GMCTRN1   0xE1
#define ST7735_NORON     0x13
#define ST7735_DISPON    0x29
#define ST7735_CASET     0x2A /* 设置列窗口 */
#define ST7735_RASET     0x2B /* 设置行窗口 */
#define ST7735_RAMWR     0x2C
#define ST7735_INVOFF    0x20
#define ST7735_INVON     0x21
#define ST7735_MADCTL    0x36
#define ST7735_MADCTL_MX 0x40
#define ST7735_MADCTL_MY 0x80
#define ST7735_MADCTL_MV 0x20
/*************************************************************************************************/
/* 函数声明 */
void ST7735_Init(uint8_t dir);
void ST7735_DrawRectangle(uint16_t x, uint16_t y, uint16_t width,
                          uint16_t height, uint16_t color);
void ST7735_DrawString(uint16_t x, uint16_t y, const char *str, uint16_t color,
                       uint16_t bgColor, const FontDef *font);
void ST7735_FillScreen(uint16_t color);
void ST7735_DrawImage(uint16_t x, uint16_t y, uint16_t width, uint16_t height,
                      const uint8_t *image);

#endif /* _ST7735_H_ */