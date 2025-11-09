/**
 * @file st7735.c
 * @author PickingChip
 * @brief 
 * @version 0.1
 * @date 2025-11-09
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "st7735.h"
#include "font.h"
#include "../core/core_delay.h"

lcd_dev_t lcd_dev;

void ST7735_Reset(void) {
    HAL_GPIO_WritePin(ST7735_RST_GPIO_Port, ST7735_RST_Pin, GPIO_PIN_RESET);
    delay_ms(100);
    HAL_GPIO_WritePin(ST7735_RST_GPIO_Port, ST7735_RST_Pin, GPIO_PIN_SET);
    delay_ms(100);
}

void ST7735_WriteCommand(uint8_t cmd) {
    HAL_GPIO_WritePin(ST7735_DC_GPIO_Port, ST7735_DC_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ST7735_CS_GPIO_Port, ST7735_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&ST7735_SPI_INSTANCE, &cmd, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(ST7735_CS_GPIO_Port, ST7735_CS_Pin, GPIO_PIN_SET);
}

void ST7735_WriteByte(uint8_t data) {
    HAL_GPIO_WritePin(ST7735_DC_GPIO_Port, ST7735_DC_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ST7735_CS_GPIO_Port, ST7735_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&ST7735_SPI_INSTANCE, &data, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(ST7735_CS_GPIO_Port, ST7735_CS_Pin, GPIO_PIN_SET);
}

void ST7735_WriteData(uint8_t *data, size_t data_size) {
    HAL_GPIO_WritePin(ST7735_DC_GPIO_Port, ST7735_DC_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ST7735_CS_GPIO_Port, ST7735_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&ST7735_SPI_INSTANCE, data, data_size, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(ST7735_CS_GPIO_Port, ST7735_CS_Pin, GPIO_PIN_SET);
}

void ST7735_SetRotation(uint8_t rotation) {
    uint8_t madctl = 0;

    switch (rotation) {
        case 0:
            madctl = ST7735_MADCTL_MX | ST7735_MADCTL_MY | ST7735_MADCTL_MODE;
            lcd_dev.width = ST7735_WIDTH;
            lcd_dev.height = ST7735_HEIGHT;
            break;
        case 1:
            madctl = ST7735_MADCTL_MY | ST7735_MADCTL_MV | ST7735_MADCTL_MODE;
            lcd_dev.width = ST7735_HEIGHT;
            lcd_dev.height = ST7735_WIDTH;
            break;
        case 2:
            madctl = ST7735_MADCTL_MODE;
            lcd_dev.width = ST7735_WIDTH;
            lcd_dev.height = ST7735_HEIGHT;
            break;
        case 3:
            madctl = ST7735_MADCTL_MX | ST7735_MADCTL_MV | ST7735_MADCTL_MODE;
            lcd_dev.width = ST7735_HEIGHT;
            lcd_dev.height = ST7735_WIDTH;
            break;
        default:
            break;
    }

    ST7735_WriteCommand(ST7735_MADCTL);
    ST7735_WriteByte(madctl);
}

/**
 * @brief ST7735屏幕初始化函数
 * @note 初始化之前请先初始化使用的SPI，模式：SPI_MODE_MASTER, SPI_CLK_MODE0, SPI_DATASIZE_8BIT, SPI_FIRSTBIT_MSB
 * 
 */
void ST7735_Init(uint8_t dir) {
    /* IO初始化 */
    GPIO_InitTypeDef gpio_init = {.Mode = GPIO_MODE_OUTPUT_PP,
                                  .Speed = GPIO_SPEED_FREQ_HIGH,
                                  .Pull = GPIO_PULLUP,
                                  .Pin = ST7735_RST_Pin | ST7735_DC_Pin |
                                         ST7735_CS_Pin};
    HAL_GPIO_Init(ST7735_RST_GPIO_Port, &gpio_init);

    lcd_dev.dir = dir;

    /* 屏幕初始化 */
    ST7735_Reset();
    ST7735_WriteCommand(ST7735_SLPOUT);
    delay_ms(120);
    ST7735_WriteCommand(ST7735_FRMCTR1);
    ST7735_WriteByte(0x01);
    ST7735_WriteByte(0x2C);
    ST7735_WriteByte(0x2D);
    ST7735_WriteCommand(ST7735_FRMCTR2);
    ST7735_WriteByte(0x01);
    ST7735_WriteByte(0x2C);
    ST7735_WriteByte(0x2D);
    ST7735_WriteCommand(ST7735_FRMCTR3);
    ST7735_WriteByte(0x01);
    ST7735_WriteByte(0x2C);
    ST7735_WriteByte(0x2D);
    ST7735_WriteByte(0x01);
    ST7735_WriteByte(0x2C);
    ST7735_WriteByte(0x2D);
    ST7735_WriteCommand(ST7735_INVCTR);
    ST7735_WriteByte(0x07);
    ST7735_WriteCommand(ST7735_PWCTR1);
    ST7735_WriteByte(0xA2);
    ST7735_WriteByte(0x02);
    ST7735_WriteByte(0x84);
    ST7735_WriteCommand(ST7735_PWCTR2);
    ST7735_WriteByte(0xC5);
    ST7735_WriteCommand(ST7735_PWCTR3);
    ST7735_WriteByte(0x0A);
    ST7735_WriteByte(0x00);
    ST7735_WriteCommand(ST7735_PWCTR4);
    ST7735_WriteByte(0x8A);
    ST7735_WriteByte(0x2A);
    ST7735_WriteCommand(ST7735_PWCTR5);
    ST7735_WriteByte(0x8A);
    ST7735_WriteByte(0xEE);
    ST7735_WriteCommand(ST7735_VMCTR1);
    ST7735_WriteByte(0x0E);
    ST7735_WriteCommand(ST7735_INVERSE ? ST7735_INVON : ST7735_INVOFF);
    ST7735_WriteCommand(ST7735_COLMOD);
    ST7735_WriteByte(0x05);
    ST7735_WriteCommand(ST7735_CASET);
    ST7735_WriteByte(0x00);
    ST7735_WriteByte(0x00);
    ST7735_WriteByte(0x00);
    ST7735_WriteByte(0x81);
    ST7735_WriteCommand(ST7735_RASET);
    ST7735_WriteByte(0x00);
    ST7735_WriteByte(0x00);
    ST7735_WriteByte(0x00);
    ST7735_WriteByte(0xA1);
    ST7735_WriteCommand(ST7735_GMCTRP1);
    ST7735_WriteByte(0x02);
    ST7735_WriteByte(0x1C);
    ST7735_WriteByte(0x07);
    ST7735_WriteByte(0x12);
    ST7735_WriteByte(0x37);
    ST7735_WriteByte(0x32);
    ST7735_WriteByte(0x29);
    ST7735_WriteByte(0x2D);
    ST7735_WriteByte(0x29);
    ST7735_WriteByte(0x25);
    ST7735_WriteByte(0x2B);
    ST7735_WriteByte(0x39);
    ST7735_WriteByte(0x00);
    ST7735_WriteByte(0x01);
    ST7735_WriteByte(0x03);
    ST7735_WriteByte(0x10);
    ST7735_WriteCommand(ST7735_GMCTRN1);
    ST7735_WriteByte(0x03);
    ST7735_WriteByte(0x1D);
    ST7735_WriteByte(0x07);
    ST7735_WriteByte(0x06);
    ST7735_WriteByte(0x2E);
    ST7735_WriteByte(0x2C);
    ST7735_WriteByte(0x29);
    ST7735_WriteByte(0x2D);
    ST7735_WriteByte(0x2E);
    ST7735_WriteByte(0x2E);
    ST7735_WriteByte(0x37);
    ST7735_WriteByte(0x3F);
    ST7735_WriteByte(0x00);
    ST7735_WriteByte(0x00);
    ST7735_WriteByte(0x02);
    ST7735_WriteByte(0x10);
    ST7735_WriteCommand(ST7735_NORON);
    delay_ms(10);
    ST7735_WriteCommand(ST7735_DISPON);
    delay_ms(10);

    ST7735_SetRotation(lcd_dev.dir);
    ST7735_FillScreen(ST7735_BLACK);
}

void ST7735_SetAddressWindow(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1) {
    x0 += ST7735_XSTART;
    y0 += ST7735_YSTART;

    x1 += ST7735_XSTART;
    y1 += ST7735_YSTART;

    ST7735_WriteCommand(ST7735_CASET);
    uint8_t data[] = {0x00, x0, 0x00, x1};
    ST7735_WriteData(data, sizeof(data));

    ST7735_WriteCommand(ST7735_RASET);
    data[1] = y0;
    data[3] = y1;
    ST7735_WriteData(data, sizeof(data));
}

void ST7735_DrawRectangle(uint16_t x, uint16_t y, uint16_t width,
                          uint16_t height, uint16_t color) {
    if ((width + x > lcd_dev.width) ||(height + y > lcd_dev.height)) {
        return;
    }
    static uint8_t buff[ST7735_MAX_WIDTH * 2];
    uint16_t i = 0;

    for (i = 0; i < width; i++) {
        buff[i * 2] = color >> 8;
        buff[i * 2 + 1] = color & 0xFF;
    }

    ST7735_SetAddressWindow(x, y, x + width - 1, y + height - 1);
    ST7735_WriteCommand(ST7735_RAMWR);
    // Write the color data
    for (i = 0; i < height; i++) {
        ST7735_WriteData(buff, sizeof(uint16_t) * width);
    }
}

void ST7735_DrawChar(uint16_t x, uint16_t y, char c, uint16_t color,
                     uint16_t bgColor, const FontDef *font) {
    uint32_t i, b, j;

    ST7735_SetAddressWindow(x, y, x + font->width - 1, y + font->height - 1);
    ST7735_WriteCommand(0x2C);

    for (i = 0; i < font->height; i++) {
        b = font->data[(font == &Font_Custom ? (c - 46) : (c - 32)) *
                           font->height +
                       i];
        for (j = 0; j < font->width; j++) {
            if ((b << j) & (font->width > 16 ? 0x80000000 : 0x8000)) {
                uint8_t data[] = {color >> 8, color & 0xFF};
                ST7735_WriteData(data, sizeof(data));
            } else {
                uint8_t data[] = {bgColor >> 8, bgColor & 0xFF};
                ST7735_WriteData(data, sizeof(data));
            }
        }
    }
}

void ST7735_DrawString(uint16_t x, uint16_t y, const char *str, uint16_t color,
                       uint16_t bgColor, const FontDef *font) {
    while (*str) {
        if (x + font->width > lcd_dev.width) {
            x = 0;
            y += font->height;
        }

        if (y + font->height > lcd_dev.height) {
            break;
        }

        ST7735_DrawChar(x, y, *str, color, bgColor, font);
        x += font->width;
        str++;
    }
}

void ST7735_FillScreen(uint16_t color) {
    ST7735_DrawRectangle(0, 0, lcd_dev.width, lcd_dev.height, color);
}

void ST7735_DrawImage(uint16_t x, uint16_t y, uint16_t width, uint16_t height,
                      const uint8_t *image) {
    ST7735_SetAddressWindow(x, y, x + width - 1, y + height - 1);

    ST7735_WriteCommand(ST7735_RAMWR);

    ST7735_WriteData((uint8_t *)image, sizeof(uint16_t) * width * height);
}
