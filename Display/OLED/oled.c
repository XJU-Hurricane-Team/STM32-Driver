/**
 * @file    oled.c
 * @author  Deadline039
 * @brief   OLED屏驱动代码
 * @version 0.1
 * @date    2023-11-05
 */

#include "oled.h"
#include "oledfont.h"

/**
  * @brief OLED初始化
  * @note 函数内已经调用iic_init(), 无需再次调用
  *
  */
void oled_init(void) {
    iic_init();
    delay_ms(200);

#if (defined(OLED_0_96))          /* 0.96寸OLED */
    oled_wr_byte(0xAE, OLED_CMD); /* 关闭显示 */
    oled_wr_byte(0xD5, OLED_CMD); /* 设置显示时钟分频比/振荡器频率 */
    oled_wr_byte(0x80, OLED_CMD);

    oled_wr_byte(0xA8, OLED_CMD); /* 设置多路复用率 */
    oled_wr_byte(0x3F, OLED_CMD);

    oled_wr_byte(0xD3, OLED_CMD); /* 设置显示偏移 */
    oled_wr_byte(0x00, OLED_CMD);

    oled_wr_byte(0x40, OLED_CMD); /* 设置显示开始行 */
    oled_wr_byte(0xA1, OLED_CMD); /* 设置左右方向，0xA1正常 0xA0左右反置 */
    oled_wr_byte(0xC8, OLED_CMD); /* 设置上下方向，0xC8正常 0xC0上下反置 */
    oled_wr_byte(0xDA, OLED_CMD); /* 设置COM引脚硬件配置 */
    oled_wr_byte(0x12, OLED_CMD);

    oled_wr_byte(0x81, OLED_CMD); /* 设置对比度控制 */
    oled_wr_byte(0xCF, OLED_CMD);

    oled_wr_byte(0xD9, OLED_CMD); /* 设置预充电周期 */
    oled_wr_byte(0xF1, OLED_CMD);

    oled_wr_byte(0xDB, OLED_CMD); /* 设置VCOMH取消选择级别 */
    oled_wr_byte(0x30, OLED_CMD);

    oled_wr_byte(0xA4, OLED_CMD); /* 设置整个显示打开/关闭 */
    oled_wr_byte(0xA6, OLED_CMD); /* 设置正常/倒转显示 */
    oled_wr_byte(0x8D, OLED_CMD); /* 设置充电泵 */
    oled_wr_byte(0x14, OLED_CMD);

    oled_wr_byte(0xAF, OLED_CMD); /* 开启显示 */
#elif (defined(OLED_0_91))        /* 0.91寸OLED */
    oled_wr_byte(0xAE, OLED_CMD); /*  关闭显示 */

    oled_wr_byte(0x40, OLED_CMD); /* ---set low column address */
    oled_wr_byte(0xB0, OLED_CMD); /* ---set high column address */

    oled_wr_byte(0xC8, OLED_CMD); /* -not offset */

    oled_wr_byte(0x81, OLED_CMD); /*  设置对比度 */
    oled_wr_byte(0xff, OLED_CMD);

    oled_wr_byte(0xa1, OLED_CMD); /*  段重定向设置 */

    oled_wr_byte(0xa6, OLED_CMD);

    oled_wr_byte(0xa8, OLED_CMD); /*  设置驱动路数 */
    oled_wr_byte(0x1f, OLED_CMD);

    oled_wr_byte(0xd3, OLED_CMD);
    oled_wr_byte(0x00, OLED_CMD);

    oled_wr_byte(0xd5, OLED_CMD);
    oled_wr_byte(0xf0, OLED_CMD);

    oled_wr_byte(0xd9, OLED_CMD);
    oled_wr_byte(0x22, OLED_CMD);

    oled_wr_byte(0xda, OLED_CMD);
    oled_wr_byte(0x02, OLED_CMD);

    oled_wr_byte(0xdb, OLED_CMD);
    oled_wr_byte(0x49, OLED_CMD);

    oled_wr_byte(0x8d, OLED_CMD);
    oled_wr_byte(0x14, OLED_CMD);

    oled_wr_byte(0xaf, OLED_CMD);

#endif /* OLED */
    oled_clear();
}

/**
  * @brief OLED写数据
  *
  * @param dat 数据
  * @param cmd 命令还是数据
  */
void oled_wr_byte(unsigned dat, unsigned cmd) {
    if (cmd) {
        iic_write_data(dat);
    } else {
        iic_write_commandd(dat);
    }
}

/**
  * @brief 开启OLED显示
  *
  */
void oled_display_on(void) {
    oled_wr_byte(0X8D, OLED_CMD);
    oled_wr_byte(0X14, OLED_CMD);
    oled_wr_byte(0XAF, OLED_CMD);
}

/**
  * @brief 关闭OLED显示
  *
  */
void oled_display_off(void) {
    oled_wr_byte(0X8D, OLED_CMD);
    oled_wr_byte(0X10, OLED_CMD);
    oled_wr_byte(0XAE, OLED_CMD);
}

/**
  * @brief OLED清屏函数
  *
  */
void oled_clear(void) {
    for (uint8_t i = 0; i < 8; i++) {
        oled_wr_byte(0xb0 + i, OLED_CMD);
        oled_wr_byte(0x00, OLED_CMD);
        oled_wr_byte(0x10, OLED_CMD);
        for (uint8_t n = 0; n < 128; n++) {
            oled_wr_byte(0, OLED_DATA);
        }
    }
}

/**
  * @brief 在指定位置显示一个字符,包括部分字符
  *
  * @param x x坐标
  * @param y y坐标
  * @param chr 字符
  * @param size 字符大小, 8或16
  */
void oled_show_char(uint8_t x, uint8_t y, uint8_t chr, uint8_t size) {
    uint8_t ch = 0;
    ch = chr - ' '; /*  得到偏移后的值 */
    if (x > MAX_COLUMN - 1) {
        x = 0;
        y = y + 2;
    }
    if (size == 16) {
        oled_set_position(x, y);
        for (uint8_t i = 0; i < 8; i++) {
            oled_wr_byte(F8X16[ch * 16 + i], OLED_DATA);
        }
        oled_set_position(x, y + 1);
        for (uint8_t i = 0; i < 8; i++) {
            oled_wr_byte(F8X16[ch * 16 + i + 8], OLED_DATA);
        }
    } else {
        oled_set_position(x, y);
        for (uint8_t i = 0; i < 6; i++) {
            oled_wr_byte(F6x8[ch][i], OLED_DATA);
        }
    }
}

/**
  * @brief OLED显示数字
  *
  * @param x x坐标
  * @param y y坐标
  * @param num 显示的数字
  * @param len 数字位数
  * @param size 字符大小, 8或16
  */
void oled_show_num(uint8_t x, uint8_t y, uint32_t num, uint8_t len,
                   uint8_t size) {
    uint8_t temp;
    uint8_t enshow = 0;
    for (uint8_t t = 0; t < len; t++) {
        temp = (int)(num / pow(10, len - t - 1)) % 10;
        if (enshow == 0 && t < (len - 1)) {
            if (temp == 0) {
                oled_show_char(x + (size / 2) * t, y, ' ', size);
                continue;
            } else {
                enshow = 1;
            }
        }
        oled_show_char(x + (size / 2) * t, y, temp + '0', size);
    }
}

/**
  * @brief OLED显示字符串
  *
  * @param x x坐标
  * @param y y坐标
  * @param chars 字符串
  * @param size 字符大小, 8或16
  */
void oled_show_string(uint8_t x, uint8_t y, const char *chars, uint8_t size) {
    uint8_t i = 0;
    while (chars[i] != '\0') {
        oled_show_char(x, y, chars[i], size);
        x += 8;
        if (x > 120) {
            x = 0;
            y += 2;
        }
        i++;
    }
}

/**
  * @brief OLED设置坐标
  *
  * @param x x坐标
  * @param y y坐标
  */
void oled_set_position(uint8_t x, uint8_t y) {
    oled_wr_byte(0xb0 + y, OLED_CMD);
    oled_wr_byte(((x & 0xf0) >> 4) | 0x10, OLED_CMD);
    oled_wr_byte((x & 0x0f), OLED_CMD);
}
