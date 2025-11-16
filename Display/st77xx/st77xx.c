/**
 * @file st77xx.c
 * @author PickingChip
 * @brief 
 * @version 0.2
 * @date 2025-11-09
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "st77xx.h"
#include "font.h"
#include "../core/core_delay.h"
#include "stm32g4xx_ll_gpio.h"

lcd_dev_t lcd_dev;
void ST77xx_SetSPISpeed(uint32_t prescaler) {
    HAL_SPI_DeInit(&ST77xx_SPI_INSTANCE);
    ST77xx_SPI_INSTANCE.Init.BaudRatePrescaler = prescaler;
    HAL_SPI_Init(&ST77xx_SPI_INSTANCE);
}

void ST77xx_Reset(void) {

    LL_GPIO_ResetOutputPin(ST77xx_RST_GPIO_Port, ST77xx_RST_Pin);
    delay_ms(100);
    LL_GPIO_SetOutputPin(ST77xx_RST_GPIO_Port, ST77xx_RST_Pin);
    delay_ms(100);
}

void ST77xx_WriteCommand(uint8_t cmd) {
    LL_GPIO_ResetOutputPin(ST77xx_DC_GPIO_Port, ST77xx_DC_Pin);
    HAL_SPI_Transmit(&ST77xx_SPI_INSTANCE, &cmd, 1, HAL_MAX_DELAY);
}

void ST77xx_WriteByte(uint8_t data) {
    LL_GPIO_SetOutputPin(ST77xx_DC_GPIO_Port, ST77xx_DC_Pin);
    HAL_SPI_Transmit(&ST77xx_SPI_INSTANCE, &data, 1, HAL_MAX_DELAY);
}

void ST77xx_WriteData(uint8_t *data, size_t data_size) {

    LL_GPIO_SetOutputPin(ST77xx_DC_GPIO_Port, ST77xx_DC_Pin);
    HAL_SPI_Transmit(&ST77xx_SPI_INSTANCE, data, data_size, HAL_MAX_DELAY);
}

/* 设置ST7735显示方向 */
void ST77xx_SetRotation(uint8_t rotation) {
    uint8_t madctl = 0;

    switch (rotation) {
        case 0:
            madctl = ST7735_MADCTL_MX | ST7735_MADCTL_MY |
                     ST77xx_MADCTL_MODE; /* 0XC0 */
            lcd_dev.width = ST77xx_WIDTH;
            lcd_dev.height = ST77xx_HEIGHT;
            break;
        case 1:
            madctl = ST7735_MADCTL_MY | ST7735_MADCTL_MV |
                     ST77xx_MADCTL_MODE; /*!< 0X0A */
            lcd_dev.width = ST77xx_HEIGHT;
            lcd_dev.height = ST77xx_WIDTH;
            break;
        case 2:
            madctl = ST77xx_MADCTL_MODE; /*!< 0X00 */
            lcd_dev.width = ST77xx_WIDTH;
            lcd_dev.height = ST77xx_HEIGHT;
            break;
        case 3:
            madctl = ST7735_MADCTL_MX | ST7735_MADCTL_MV |
                     ST77xx_MADCTL_MODE; /* !< 0x60 */
            lcd_dev.width = ST77xx_HEIGHT;
            lcd_dev.height = ST77xx_WIDTH;
            break;
        default:
            break;
    }

    ST77xx_WriteCommand(ST7735_MADCTL);
    ST77xx_WriteByte(madctl);
}

/**
 * @brief ST7735屏幕初始化函数
 * @note 初始化之前请先初始化使用的SPI，模式：SPI_MODE_MASTER, SPI_CLK_MODE0, SPI_DATASIZE_8BIT, SPI_FIRSTBIT_MSB
 * 
 */
void ST77xx_Init(uint8_t dir, ic_type_t st77xx) {
    /* IO初始化 */

    __HAL_RCC_GPIOE_CLK_ENABLE();
    LL_GPIO_InitTypeDef gpio_init = {.Mode = LL_GPIO_MODE_OUTPUT,
                                     .Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH,
                                     .Pull = LL_GPIO_PULL_UP,
                                     .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
                                     .Pin = ST77xx_CS_Pin | ST77xx_RST_Pin |
                                            ST77xx_DC_Pin};

    LL_GPIO_Init(ST77xx_RST_GPIO_Port, &gpio_init);

    lcd_dev.dir = dir;
    lcd_dev.id = st77xx;
    LL_GPIO_ResetOutputPin(ST77xx_RST_GPIO_Port, ST77xx_CS_Pin);

    if (lcd_dev.id == ST7735) {
        /* 屏幕初始化 */
        ST77xx_Reset();
        ST77xx_WriteCommand(ST7735_SLPOUT);
        delay_ms(120);
        ST77xx_WriteCommand(ST7735_FRMCTR1);
        ST77xx_WriteByte(0x01);
        ST77xx_WriteByte(0x2C);
        ST77xx_WriteByte(0x2D);
        ST77xx_WriteCommand(ST7735_FRMCTR2);
        ST77xx_WriteByte(0x01);
        ST77xx_WriteByte(0x2C);
        ST77xx_WriteByte(0x2D);
        ST77xx_WriteCommand(ST7735_FRMCTR3);
        ST77xx_WriteByte(0x01);
        ST77xx_WriteByte(0x2C);
        ST77xx_WriteByte(0x2D);
        ST77xx_WriteByte(0x01);
        ST77xx_WriteByte(0x2C);
        ST77xx_WriteByte(0x2D);
        ST77xx_WriteCommand(ST7735_INVCTR);
        ST77xx_WriteByte(0x07);
        ST77xx_WriteCommand(ST7735_PWCTR1);
        ST77xx_WriteByte(0xA2);
        ST77xx_WriteByte(0x02);
        ST77xx_WriteByte(0x84);
        ST77xx_WriteCommand(ST7735_PWCTR2);
        ST77xx_WriteByte(0xC5);
        ST77xx_WriteCommand(ST7735_PWCTR3);
        ST77xx_WriteByte(0x0A);
        ST77xx_WriteByte(0x00);
        ST77xx_WriteCommand(ST7735_PWCTR4);
        ST77xx_WriteByte(0x8A);
        ST77xx_WriteByte(0x2A);
        ST77xx_WriteCommand(ST7735_PWCTR5);
        ST77xx_WriteByte(0x8A);
        ST77xx_WriteByte(0xEE);
        ST77xx_WriteCommand(ST7735_VMCTR1);
        ST77xx_WriteByte(0x0E);
        ST77xx_WriteCommand(ST7735_INVERSE ? ST7735_INVON : ST7735_INVOFF);
        ST77xx_WriteCommand(ST7735_COLMOD);
        ST77xx_WriteByte(0x05);
        ST77xx_WriteCommand(ST7735_CASET); /* 设置窗口行地址 */
        ST77xx_WriteByte(0x00);
        ST77xx_WriteByte(0x00);
        ST77xx_WriteByte(0x00);
        ST77xx_WriteByte(0x81);
        ST77xx_WriteCommand(ST7735_RASET); /* 设置窗口列地址 */
        ST77xx_WriteByte(0x00);
        ST77xx_WriteByte(0x00);
        ST77xx_WriteByte(0x00);
        ST77xx_WriteByte(0xA1);
        ST77xx_WriteCommand(ST7735_GMCTRP1);
        ST77xx_WriteByte(0x02);
        ST77xx_WriteByte(0x1C);
        ST77xx_WriteByte(0x07);
        ST77xx_WriteByte(0x12);
        ST77xx_WriteByte(0x37);
        ST77xx_WriteByte(0x32);
        ST77xx_WriteByte(0x29);
        ST77xx_WriteByte(0x2D);
        ST77xx_WriteByte(0x29);
        ST77xx_WriteByte(0x25);
        ST77xx_WriteByte(0x2B);
        ST77xx_WriteByte(0x39);
        ST77xx_WriteByte(0x00);
        ST77xx_WriteByte(0x01);
        ST77xx_WriteByte(0x03);
        ST77xx_WriteByte(0x10);
        ST77xx_WriteCommand(ST7735_GMCTRN1);
        ST77xx_WriteByte(0x03);
        ST77xx_WriteByte(0x1D);
        ST77xx_WriteByte(0x07);
        ST77xx_WriteByte(0x06);
        ST77xx_WriteByte(0x2E);
        ST77xx_WriteByte(0x2C);
        ST77xx_WriteByte(0x29);
        ST77xx_WriteByte(0x2D);
        ST77xx_WriteByte(0x2E);
        ST77xx_WriteByte(0x2E);
        ST77xx_WriteByte(0x37);
        ST77xx_WriteByte(0x3F);
        ST77xx_WriteByte(0x00);
        ST77xx_WriteByte(0x00);
        ST77xx_WriteByte(0x02);
        ST77xx_WriteByte(0x10);
        ST77xx_WriteCommand(ST7735_NORON);
        delay_ms(10);
        ST77xx_WriteCommand(ST7735_DISPON);
        delay_ms(10);

        ST77xx_SetRotation(lcd_dev.dir);
        ST77xx_FillScreen(ST77xx_WHITE);
    } else if (lcd_dev.id == ST7789) {
        ST77xx_WriteCommand(ST7789_SLPOUT);
        delay_ms(120);
        ST77xx_SetRotation(lcd_dev.dir);
        ST77xx_WriteCommand(ST7789_COLMOD);
        ST77xx_WriteByte(0x05);
        ST77xx_WriteCommand(ST7789_CMD_PORCH);
        ST77xx_WriteByte(0x0C);
        ST77xx_WriteByte(0x0C);
        ST77xx_WriteByte(0x00);
        ST77xx_WriteByte(0x33);
        ST77xx_WriteByte(0x33);
        ST77xx_WriteCommand(ST7789_CMD_GATECTRL);
        ST77xx_WriteByte(0x35);
        ST77xx_WriteCommand(ST7789_CMD_VCOM);
        ST77xx_WriteByte(0x32);
        ST77xx_WriteCommand(ST7789_CMD_VDVVRHEN);
        ST77xx_WriteByte(0x01);
        ST77xx_WriteCommand(ST7789_CMD_VRHS_SET);
        ST77xx_WriteByte(0x15); //GVDD=4.8V  颜色深度
        ST77xx_WriteCommand(ST7789_CMD_VDV_SET);
        ST77xx_WriteByte(0x20); //VDV, 0x20:0v
        ST77xx_WriteCommand(ST7789_CMD_FRAMERATE);
        ST77xx_WriteByte(0x0F); //0x0F:60Hz
        ST77xx_WriteCommand(ST7789_CMD_PV_GM);
        ST77xx_WriteByte(0xA4);
        ST77xx_WriteByte(0xA1);
        ST77xx_WriteCommand(ST7789_GMCTRP1);
        ST77xx_WriteByte(0xD0);
        ST77xx_WriteByte(0x08);
        ST77xx_WriteByte(0x0E);
        ST77xx_WriteByte(0x09);
        ST77xx_WriteByte(0x09);
        ST77xx_WriteByte(0x05);
        ST77xx_WriteByte(0x31);
        ST77xx_WriteByte(0x33);
        ST77xx_WriteByte(0x48);
        ST77xx_WriteByte(0x17);
        ST77xx_WriteByte(0x14);
        ST77xx_WriteByte(0x15);
        ST77xx_WriteByte(0x31);
        ST77xx_WriteByte(0x34);
        ST77xx_WriteCommand(ST7789_GMCTRN1);
        ST77xx_WriteByte(0xD0);
        ST77xx_WriteByte(0x08);
        ST77xx_WriteByte(0x0E);
        ST77xx_WriteByte(0x09);
        ST77xx_WriteByte(0x09);
        ST77xx_WriteByte(0x15);
        ST77xx_WriteByte(0x31);
        ST77xx_WriteByte(0x33);
        ST77xx_WriteByte(0x48);
        ST77xx_WriteByte(0x17);
        ST77xx_WriteByte(0x14);
        ST77xx_WriteByte(0x15);
        ST77xx_WriteByte(0x31);
        ST77xx_WriteByte(0x34);
        ST77xx_WriteCommand(ST7789_INVON);
        ST77xx_WriteCommand(ST7789_DISPON);
        delay_ms(10);
        ST77xx_FillScreen(ST77xx_BLACK);
    }
}

/**
 * @brief 
 * 
 * @param x0 
 * @param y0 
 * @param x1 
 * @param y1 
 */
void ST77xx_SetAddressWindow(uint16_t x0, uint16_t y0, uint16_t x1,
                             uint16_t y1) {
    x0 += ST77xx_XSTART;
    y0 += ST77xx_YSTART;

    x1 += ST77xx_XSTART;
    y1 += ST77xx_YSTART;

    ST77xx_WriteCommand(ST7735_CASET);
    uint8_t data[] = {0x00, x0, 0x00, x1};
    ST77xx_WriteData(data, sizeof(data));

    ST77xx_WriteCommand(ST7735_RASET);
    data[1] = y0;
    data[3] = y1;
    ST77xx_WriteData(data, sizeof(data));
}

/**
 * @brief 划线函数
 * 
 * @param x0 
 * @param y0 
 * @param x1 
 * @param y1 
 * @param color 
 */
void ST77xx_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1,
                     uint16_t color) {
    uint16_t t;
    uint8_t buffer[] = {color >> 8, color & 0xFF};
    ;
    int xerr = 0, yerr = 0, delta_x, delta_y, distance;
    int incx, incy, uRow, uCol;
    delta_x = x1 - x0; //计算坐标增量
    delta_y = y1 - y0;
    uRow = x0; //画线起点坐标
    uCol = y0;
    if (delta_x > 0) {
        incx = 1; //设置单步方向
    } else if (delta_x == 0) {
        incx = 0; //垂直线
    } else {
        incx = -1;
        delta_x = -delta_x;
    }
    if (delta_y > 0) {
        incy = 1;
    } else if (delta_y == 0) {
        incy = 0; //水平线
    } else {
        incy = -1;
        delta_y = -delta_y;
    }
    if (delta_x > delta_y) {
        distance = delta_x; //选取基本增量坐标轴
    } else {
        distance = delta_y;
    }
    for (t = 0; t < distance + 1; t++) {
        ST77xx_SetAddressWindow(uRow, uCol, uRow, uCol); //画点
        ST77xx_WriteCommand(ST7735_RAMWR);
        ST77xx_WriteData(buffer, sizeof(buffer));
        xerr += delta_x;
        yerr += delta_y;
        if (xerr > distance) {
            xerr -= distance;
            uRow += incx;
        }
        if (yerr > distance) {
            yerr -= distance;
            uCol += incy;
        }
    }
}

/**
 * @brief 画矩形函数
 * 
 * @param x 
 * @param y 
 * @param width 
 * @param height 
 * @param color 
 */
void ST77xx_DrawRectangle(uint16_t x, uint16_t y, uint16_t width,
                          uint16_t height, uint16_t color) {
    if ((width + x > lcd_dev.width) || (height + y > lcd_dev.height)) {
        return;
    }
    static uint8_t buff[ST77xx_MAX_WIDTH * 2];
    uint16_t i = 0;

    for (i = 0; i < width; i++) {
        buff[i * 2] = color >> 8;
        buff[i * 2 + 1] = color & 0xFF;
    }

    ST77xx_SetAddressWindow(x, y, x + width - 1, y + height - 1);
    ST77xx_WriteCommand(ST7735_RAMWR);
    // Write the color data
    for (i = 0; i < height; i++) {
        ST77xx_WriteData(buff, sizeof(uint16_t) * width);
    }
}

void ST77xx_DrawChar(uint16_t x, uint16_t y, char c, uint16_t color,
                     uint16_t bgColor, const FontDef *font) {
    uint32_t i, b, j;

    ST77xx_SetAddressWindow(x, y, x + font->width - 1, y + font->height - 1);
    ST77xx_WriteCommand(ST7735_RAMWR);

    for (i = 0; i < font->height; i++) {
        b = font->data[(font == &Font_Custom ? (c - 46) : (c - 32)) *
                           font->height +
                       i];
        for (j = 0; j < font->width; j++) {
            if ((b << j) & (font->width > 16 ? 0x80000000 : 0x8000)) {
                uint8_t data[] = {color >> 8, color & 0xFF};
                ST77xx_WriteData(data, sizeof(data));
            } else {
                uint8_t data[] = {bgColor >> 8, bgColor & 0xFF};
                ST77xx_WriteData(data, sizeof(data));
            }
        }
    }
}

void ST77xx_DrawString(uint16_t x, uint16_t y, const char *str, uint16_t color,
                       uint16_t bgColor, const FontDef *font) {
    while (*str) {
        if (x + font->width > lcd_dev.width) {
            x = 0;
            y += font->height;
        }

        if (y + font->height > lcd_dev.height) {
            break;
        }

        ST77xx_DrawChar(x, y, *str, color, bgColor, font);
        x += font->width;
        str++;
    }
}

void ST77xx_FillScreen(uint16_t color) {
    ST77xx_DrawRectangle(0, 0, lcd_dev.width, lcd_dev.height, color);
}

void ST77xx_DrawImage(uint16_t x, uint16_t y, uint16_t width, uint16_t height,
                      const uint8_t *image) {
    ST77xx_SetAddressWindow(x, y, x + width - 1, y + height - 1);

    ST77xx_WriteCommand(ST7735_RAMWR);

    ST77xx_WriteData((uint8_t *)image, sizeof(uint16_t) * width * height);
}
