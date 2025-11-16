/**
 * @file st77xx.h
 * @author PickingChip
 * @brief 
 * @version 0.2
 * @date 2025-11-09
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef _ST77xx_H_
#define _ST77xx_H_

#include "bsp.h"

typedef enum {
    ST7735,
    ST7789,
} ic_type_t;

typedef struct {
    const uint8_t width;
    const uint8_t height;
    const uint32_t *data;
} FontDef;

typedef struct {
    ic_type_t id;    /* IC类型 */
    uint16_t width;  /* 屏幕的宽度，随显示方向改变 */
    uint16_t height; /* 屏幕的高度，随显示方向改变 */
    uint8_t dir;     /* 显示的方向，取值0~3，对应屏幕逆时针旋转0°~270° */
} lcd_dev_t;

extern const FontDef Font_7x10;
extern const FontDef Font_11x18;
extern const FontDef Font_Custom;

/*************************************************************************************************/
/* 引脚定义 */

#define ST77xx_CS_Pin        LL_GPIO_PIN_0 /*!<片选引脚 */
#define ST77xx_CS_GPIO_Port  GPIOE
#define ST77xx_RST_Pin       LL_GPIO_PIN_1 /*!<硬件复位引脚 */
#define ST77xx_RST_GPIO_Port GPIOE
#define ST77xx_DC_Pin        LL_GPIO_PIN_2 /*!<数据(1)/命令(0)选择引脚 */
#define ST77xx_DC_GPIO_Port  GPIOE
#define ST77xx_BL_Pin        LL_GPIO_PIN_3 /*!<背光引脚 */
#define ST77xx_BL_GPIO_Port  GPIOE
#define ST77xx_SPI_INSTANCE  spi1_handle

/*************************************************************************************************/
/* 显示设置 */

#define ST77xx_XSTART        0 /*!<显示区域设置 */
#define ST77xx_YSTART        0
#define ST77xx_WIDTH         130 /* 横向分辨率 */
#define ST77xx_HEIGHT        162 /* 纵向分辨率 */
#define ST77xx_MAX_WIDTH                                                       \
    ((ST77xx_WIDTH) > (ST77xx_HEIGHT) ? (ST77xx_WIDTH) : (ST77xx_HEIGHT))

#define ST77xx_MADCTL_RGB  0x00 /*!<Color Mode: RGB or BGR */
#define ST77xx_MADCTL_BGR  0x08
#define ST77xx_MADCTL_MODE ST77xx_MADCTL_RGB

#define ST7735_INVERSE     0 /*!< Color Inverse: 0=NO, 1=YES */

/*************************************************************************************************/
/* 颜色定义 */

#define ST77xx_BLACK       0x0000
#define ST77xx_BLUE        0x001F
#define ST77xx_RED         0xF800
#define ST77xx_GREEN       0x07E0
#define ST77xx_CYAN        0x07FF
#define ST77xx_MAGENTA     0xF81F
#define ST77xx_YELLOW      0xFFE0
#define ST77xx_WHITE       0xFFFF
#define ST77xx_COLOR565(r, g, b)                                               \
    (((r & 0xF8) << 8) | ((g & 0xFC) << 3) | ((b & 0xF8) >> 3))

/*************************************************************************************************/
/* IC指令 */

/* ST7735  */
#define ST7735_SLPOUT  0x11 /* 退出睡眠，唤醒控制器，需延时等待内部电源稳定 */
#define ST7735_FRMCTR1 0xB1 /* 帧率/时序设置（通常跟3字节） */
#define ST7735_FRMCTR2 0xB2 /* 空闲模式帧率设置 */
#define ST7735_FRMCTR3 0xB3 /* 更完整的帧率/时序设置 */
#define ST7735_INVCTR  0xB4 /* 反相相关时序设置 */
#define ST7735_PWCTR1                                                          \
    0xC0 /* Power Control 1：电源/泵参数设置（影响 VGH/VGL 等） */
#define ST7735_PWCTR2        0xC1 /* Power Control 2：电源相关参数 */
#define ST7735_PWCTR3        0xC2 /* Power Control 3：电源/参考电压设置 */
#define ST7735_PWCTR4        0xC3 /* Power Control 4：电源补充参数 */
#define ST7735_PWCTR5        0xC4 /* Power Control 5：电源补充参数 */
#define ST7735_VMCTR1        0xC5 /* VCOM 电压设置，影响黑场和对比 */
#define ST7735_COLMOD        0x3A /* 像素格式（0x05=16-bit RGB565） */
#define ST7735_GMCTRP1       0xE0 /* 正向 Gamma 表（参数数组） */
#define ST7735_GMCTRN1       0xE1 /* 负向 Gamma 表 */
#define ST7735_NORON         0x13 /* 设置为正常显示模式 */
#define ST7735_DISPON        0x29 /* 打开显示 */
#define ST7735_CASET         0x2A /* 列地址起止（4字节：xs/xe） */
#define ST7735_RASET         0x2B /* 行地址起止（4字节：ys/ye） */
#define ST7735_RAMWR         0x2C /* 写像素数据（需与 COLMOD 相匹配） */
#define ST7735_INVOFF        0x20 /* 关闭显示反相 */
#define ST7735_INVON         0x21 /* 开启显示反相 */
#define ST7735_MADCTL        0x36 /* 行列交换/镜像/RGB-BGR 控制 */
#define ST7735_MADCTL_MX     0x40 /* MX 水平镜像（左右翻转） */
#define ST7735_MADCTL_MY     0x80 /* MY 垂直镜像（上下翻转） */
#define ST7735_MADCTL_MV     0x20 /* MV 行列交换（用于 90° 旋转） */

/* ST7789 */
#define ST7789_NOP           0x00 /* NOP */
#define ST7789_SWRESET       0x01 /* Software Reset */
#define ST7789_SLPIN         0x10 /* Sleep In */
#define ST7789_SLPOUT        0x11 /* Sleep Out */
#define ST7789_PTLON         0x12 /* Partial Mode On (optional) */
#define ST7789_NORON         0x13 /* Normal Display On */
#define ST7789_INVOFF        0x20 /* Inversion Off */
#define ST7789_INVON         0x21 /* Inversion On */
#define ST7789_DISPOFF       0x28 /* Display Off */
#define ST7789_DISPON        0x29 /* Display On */
#define ST7789_CASET         0x2A /* Column Address Set */
#define ST7789_RASET         0x2B /* Row Address Set */
#define ST7789_RAMWR         0x2C /* Memory Write */
#define ST7789_COLMOD        0x3A /* Interface Pixel Format (0x05 = RGB565) */
#define ST7789_MADCTL        0x36 /* MADCTL (Memory Data Access Control) */

#define ST7789_CMD_PORCH     0xB2 /* Porch Setting / Frame Setting */
#define ST7789_CMD_GATECTRL  0xB7 /* Gate Control */
#define ST7789_CMD_VCOM      0xBB /* VCOM Setting */
#define ST7789_CMD_VRHS      0xC0 /* VRH Set (driver related) */
#define ST7789_CMD_VDVVRHEN  0xC2 /* VDV and VRH Command Enable */
#define ST7789_CMD_VRHS_SET  0xC3 /* GVDD / VRH 等（变体） */
#define ST7789_CMD_VDV_SET   0xC4 /* VDV Set */
#define ST7789_CMD_FRAMERATE 0xC6 /* Frame Rate Control (e.g., 60Hz) */
#define ST7789_CMD_PV_GM     0xD0 /* Power control (Positive voltage control) */
#define ST7789_GMCTRP1       0xE0 /* Gamma Positive */
#define ST7789_GMCTRN1       0xE1 /* Gamma Negative */
/*************************************************************************************************/
/* 函数声明 */
void ST77xx_Init(uint8_t dir, ic_type_t st77xx);
void ST77xx_DrawRectangle(uint16_t x, uint16_t y, uint16_t width,
                          uint16_t height, uint16_t color);
void ST77xx_DrawString(uint16_t x, uint16_t y, const char *str, uint16_t color,
                       uint16_t bgColor, const FontDef *font);
void ST77xx_FillScreen(uint16_t color);
void ST77xx_DrawImage(uint16_t x, uint16_t y, uint16_t width, uint16_t height,
                      const uint8_t *image);

#endif /* _ST77xx_H_ */