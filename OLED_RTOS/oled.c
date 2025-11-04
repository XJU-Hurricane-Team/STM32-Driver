/**
 * @file    oled.c
 * @author  江协科技 (jxkj)
 * @brief   OLED 屏驱动代码
 * @version 2.1
 * @date    2023-11-22
 */

#include "oled.h"
#include "oledfont.h"

#if (defined(OLED_USE_SPI))
#define OLED_CS_GPIO_WRITE(X)                                                  \
    HAL_GPIO_WritePin(OLED_CS_GPIO_PORT, OLED_CS_GPIO_PIN, X)
#define OLED_DC_GPIO_WRITE(X)                                                  \
    HAL_GPIO_WritePin(OLED_DC_GPIO_PORT, OLED_DC_GPIO_PIN, X)
#define OLED_RES_GPIO_WRITE(X)                                                 \
    HAL_GPIO_WritePin(OLED_RES_GPIO_PORT, OLED_RES_GPIO_PIN, X)
#endif /* OLED_USE_SPI */

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/**
 * 数据存储格式:
 * 纵向 8 点, 高位在下, 先从左到右, 再从上到下
 * 每一个 Bit 对应一个像素点
 *
 *      B0 B0                  B0 B0
 *      B1 B1                  B1 B1
 *      B2 B2                  B2 B2
 *      B3 B3  ------------->  B3 B3 --
 *      B4 B4                  B4 B4  |
 *      B5 B5                  B5 B5  |
 *      B6 B6                  B6 B6  |
 *      B7 B7                  B7 B7  |
 *                                    |
 *  -----------------------------------
 *  |
 *  |   B0 B0                  B0 B0
 *  |   B1 B1                  B1 B1
 *  |   B2 B2                  B2 B2
 *  --> B3 B3  ------------->  B3 B3
 *      B4 B4                  B4 B4
 *      B5 B5                  B5 B5
 *      B6 B6                  B6 B6
 *      B7 B7                  B7 B7
 *
 * 坐标轴定义:
 * 左上角为 (0, 0) 点
 * 横向向右为 X 轴, 取值范围: 0 ~ OLED_MAX_COLUMN - 1
 * 纵向向下为 Y 轴, 取值范围: 0 ~ 31/63
 *
 *       0             X 轴           127
 *      .------------------------------->
 *    0 |
 *      |
 *      |
 *      |
 *  Y   |
 *      |
 *      |
 *      |
 * 31/63|
 *      v
 *
 */

/**
 * OLED 显存数组
 * 所有的显示函数, 都只是对此显存数组进行读写
 * 随后调用 oled_update 函数或 oled_update_area 函数
 * 才会将显存数组的数据发送到 OLED 硬件, 进行显示
 */
static uint8_t oled_display_buf[OLED_MAX_PAGE][OLED_MAX_COLUMN];

/* OLED 是否打开 */
static uint8_t oled_is_open;

#include "FreeRTOS.h"
#include "semphr.h"

/* I2C 硬件发送信号量, 等待硬件发送完毕, 由硬件释放 */
static SemaphoreHandle_t i2c_semp;
/* 显示互斥信号量, oled_printf 做线程安全处理使用 */
static SemaphoreHandle_t show_semp;

/**
 * @brief OLED 写入命令
 *
 * @param data 写入的命令
 */
static void oled_write_command(uint8_t data) {
#if (defined(OLED_USE_I2C))
    if (xSemaphoreTake(i2c_semp, portMAX_DELAY) == pdTRUE) {
        HAL_I2C_Mem_Write_DMA(&i2c1_handle, OLED_ADDRESS, 0x00,
                              I2C_MEMADD_SIZE_8BIT, &data, 1);
    }
#elif (defined(OLED_USE_SPI))
    OLED_CS_GPIO_WRITE(GPIO_PIN_RESET);
    OLED_DC_GPIO_WRITE(GPIO_PIN_RESET);
    HAL_SPI_Transmit(&spi1_handle, &data, 1, 100);
    OLED_CS_GPIO_WRITE(GPIO_PIN_SET);
#endif /* OLED_INTERFACE */
}

/**
 * @brief OLED 写入数据
 *
 * @param data 数据
 * @param count 数据长度
 */
static void oled_write_data(uint8_t *data, uint8_t count) {
#if (defined(OLED_USE_I2C))
    if (xSemaphoreTake(i2c_semp, portMAX_DELAY) == pdTRUE) {
        HAL_I2C_Mem_Write_DMA(&i2c1_handle, OLED_ADDRESS, 0x40,
                              I2C_MEMADD_SIZE_8BIT, data, count);
    }
#elif (defined(OLED_USE_SPI))
    OLED_CS_GPIO_WRITE(GPIO_PIN_RESET);
    OLED_DC_GPIO_WRITE(GPIO_PIN_SET);
    HAL_SPI_Transmit(&spi1_handle, data, count, 100);
    OLED_CS_GPIO_WRITE(GPIO_PIN_SET);
#endif /* OLED_INTERFACE */
}

/**
 * @brief I2C 发送完毕, 释放信号量
 *
 * @param hi2c I2C 句柄
 */
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance != I2C1) {
        return;
    }

    xSemaphoreGiveFromISR(i2c_semp, NULL);
}

/**
 * @brief 幂函数
 *
 * @param x 底数
 * @param y 指数
 * @return x 的 y 次方
 */
static uint32_t oled_pow(uint32_t x, uint32_t y) {
    uint32_t result = 1;
    while (y--) {
        result *= x;
    }
    return result;
}

/**
 * @brief OLED 初始化
 *
 */
void oled_init(void) {
#if (defined(OLED_USE_SPI))
    GPIO_InitTypeDef gpio_init_struct;
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init_struct.Pull = GPIO_PULLUP;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;

    OLED_CS_GPIO_CLK_ENABLE();
    gpio_init_struct.Pin = OLED_CS_GPIO_PIN;
    HAL_GPIO_Init(OLED_CS_GPIO_PORT, &gpio_init_struct);

    OLED_DC_GPIO_CLK_ENABLE();
    gpio_init_struct.Pin = OLED_DC_GPIO_PIN;
    HAL_GPIO_Init(OLED_DC_GPIO_PORT, &gpio_init_struct);

    OLED_RES_GPIO_CLK_ENABLE();
    gpio_init_struct.Pin = OLED_RES_GPIO_PIN;
    HAL_GPIO_Init(OLED_RES_GPIO_PORT, &gpio_init_struct);

    OLED_CS_GPIO_WRITE(GPIO_PIN_SET);
    OLED_DC_GPIO_WRITE(GPIO_PIN_SET);
    OLED_RES_GPIO_WRITE(GPIO_PIN_SET);

#endif /* OLED_USE_SPI */

    i2c_semp = xSemaphoreCreateBinary();
    xSemaphoreGive(i2c_semp);
    show_semp = xSemaphoreCreateMutex();

#if (defined(OLED_0_96) || defined(OLED_1_30)) /* 0.96/1.3 寸 OLED */

    oled_write_command(0xAE); /* 关闭显示 */
    oled_write_command(0xD5); /* 设置显示时钟分频比 / 振荡器频率 */
    oled_write_command(0x80);

    oled_write_command(0xA8); /* 设置多路复用率 */
    oled_write_command(0x3F);

    oled_write_command(0xD3); /* 设置显示偏移 */
    oled_write_command(0x00);

    oled_write_command(0x40); /* 设置显示开始行 */
    oled_write_command(0xA1); /* 设置左右方向, 0xA1 正常 0xA0 左右反置 */
    oled_write_command(0xC8); /* 设置上下方向, 0xC8 正常 0xC0 上下反置 */
    oled_write_command(0xDA); /* 设置 COM 引脚硬件配置 */
    oled_write_command(0x12);

    oled_write_command(0x81); /* 设置对比度控制 */
    oled_write_command(0xCF);

    oled_write_command(0xD9); /* 设置预充电周期 */
    oled_write_command(0xF1);

    oled_write_command(0xDB); /* 设置 VCOMH 取消选择级别 */
    oled_write_command(0x30);

    oled_write_command(0xA4); /* 设置整个显示打开 / 关闭 */
    oled_write_command(0xA6); /* 设置正常 / 倒转显示 */
    oled_write_command(0x8D); /* 设置充电泵 */
    oled_write_command(0x14);

    oled_write_command(0xAF); /* 开启显示 */

#elif (defined(OLED_0_91)) /* 0.91 寸 OLED */
    oled_write_command(0xAE); /*  关闭显示 */

    oled_write_command(0x40); /* ---set low column address */
    oled_write_command(0xB0); /* ---set high column address */

    oled_write_command(0xC8); /* -not offset */

    oled_write_command(0x81); /*  设置对比度 */
    oled_write_command(0xFF);

    oled_write_command(0xA1); /*  段重定向设置 */

    oled_write_command(0xA6);

    oled_write_command(0xA8); /*  设置驱动路数 */
    oled_write_command(0x1F);

    oled_write_command(0xD3);
    oled_write_command(0x00);

    oled_write_command(0xD5);
    oled_write_command(0xF0);

    oled_write_command(0xD9);
    oled_write_command(0x22);

    oled_write_command(0xDA);
    oled_write_command(0x02);

    oled_write_command(0xDB);
    oled_write_command(0x49);

    oled_write_command(0x8D);
    oled_write_command(0x14);

    oled_write_command(0xAF);

#else /* OLED_SIZE */
#error Unknow OLED size. you should define OLED_0_91,  OLED_0_96 or OLED_1_30.
#endif /* OLED_SIZE */

    oled_is_open = 1;

    oled_clear();
    oled_update();
}

/**
 * @brief 打开 OLED 显示
 *
 */
void oled_on(void) {
    oled_write_command(0x8D);
    oled_write_command(0x14);
    oled_write_command(0xAF);
    oled_is_open = 1;
}

/**
 * @brief 关闭 OLED 显示
 *
 */
void oled_off(void) {
    oled_is_open = 0;
    oled_write_command(0x8D);
    oled_write_command(0x10);
    oled_write_command(0xAE);
}

/**
 * @brief OLED 设置显示光标位置
 *
 * @param page 指定光标所在的页, 范围: 0 ~ OLED_MAX_PAGE
 * @param x 指定光标所在的 X 轴坐标, 范围: 0 ~ OLED_MAX_COLUMN - 1
 * @note OLED 默认的 y 轴, 只能 8 个 Bit 为一组写入, 即 1 页等于 8 个 y 轴坐标
 */
void oled_set_cursor(uint8_t page, uint8_t x) {
#if (defined(OLED_1_30))
    /* 1.3 寸的 OLED 驱动芯片 (SH1106) 有 132 列, 屏幕的起始列接在了第 2
     * 列, 而不是第 0 列,  所以需要将 x 加 2, 才能正常显示 */
    x += 2;
#endif /* defined (OLED_1_30) */

    /* 通过指令设置页地址和列地址 */
    oled_write_command(0xB0 | page);              /* 设置页位置 */
    oled_write_command(0x10 | ((x & 0xF0) >> 4)); /* 设置 X 位置高 4 位 */
    oled_write_command(0x00 | (x & 0x0F));        /* 设置 X 位置低 4 位 */
}

/**
 * @brief 判断指定点是否在指定多边形内部
 *
 * @param nvert 多边形的顶点数
 * @param vertx 包含多边形顶点的 x 坐标的数组
 * @param verty 包含多边形顶点的 y 坐标的数组
 * @param testx 测试点的 x 坐标
 * @param testy 测试点的 y 坐标
 * @return 指定点是否在指定多边形内部, 1: 在内部, 0: 不在内部
 */
uint8_t oled_pnpoly(uint8_t nvert, int16_t *vertx, int16_t *verty,
                    int16_t testx, int16_t testy) {
    int16_t i, j, c = 0;

    /* 此算法由 W. Randolph Franklin 提出
     * 参考链接:https://wrfranklin.org/Research/Short_Notes/pnpoly.html */
    for (i = 0, j = nvert - 1; i < nvert; j = i++) {
        if (((verty[i] > testy) != (verty[j] > testy)) &&
            (testx < (vertx[j] - vertx[i]) * (testy - verty[i]) /
                             (verty[j] - verty[i]) +
                         vertx[i])) {
            c = !c;
        }
    }
    return c;
}

/**
 * @brief 判断指定点是否在指定角度内部
 *
 * @param x 指定点的 x 坐标
 * @param y 指定点的 y 坐标
 * @param start_angle 起始角度, 范围: -180 ~ 180
 * @param end_angle 终止角度, 范围: -180 ~ 180
 * @return 指定点是否在指定角度内部, 1: 在内部, 0: 不在内部
 * @note 水平向右为 0 度, 水平向左为 180 度或 - 180 度, 下方为正数, 上方为负数,
 *       顺时针旋转
 */
uint8_t oled_is_in_angle(int16_t x, int16_t y, int16_t start_angle,
                         int16_t end_angle) {
    int16_t point_angle;

    /* 计算指定点的弧度, 并转换为角度表示 */
    point_angle = (int16_t)(atan2(y, x) / 3.14 * 180.0);
    if (start_angle < end_angle) {
        /* 起始角度小于终止角度的情况 */
        /* 如果指定角度在起始终止角度之间, 则判定指定点在指定角度 */
        if (point_angle >= start_angle && point_angle <= end_angle) {
            return 1;
        }
    } else {
        /* 起始角度大于于终止角度的情况 */
        /* 如果指定角度大于起始角度或者小于终止角度, 则判定指定点在指定角度 */
        if (point_angle >= start_angle || point_angle <= end_angle) {
            return 1;
        }
    }
    return 0; /* 不满足以上条件, 则判断判定指定点不在指定角度 */
}

/**
 * @brief 将 OLED 显存数组更新到 OLED 屏幕
 *
 * @note    所有的显示函数, 都只是对 OLED 显存数组进行读写
 *          随后调用 oled_update 函数或 oled_update_area 函数
 *          才会将显存数组的数据发送到 OLED 硬件, 进行显示
 *          故调用显示函数后, 要想真正地呈现在屏幕上, 还需调用更新函数
 */
void oled_update(void) {
    uint8_t j;

    if (oled_is_open == 0) {
        return;
    }

    /* 遍历每一页 */
    for (j = 0; j < OLED_MAX_PAGE; j++) {
        /* 设置光标位置为每一页的第一列 */
        oled_set_cursor(j, 0);
        /* 连续写入 128 个数据, 将显存数组的数据写入到 OLED 硬件 */
        oled_write_data(oled_display_buf[j], OLED_MAX_COLUMN);
    }
}

/**
 * @brief 将 OLED 显存数组部分更新到 OLED 屏幕
 *
 * @param x 指定区域左上角的横坐标, 范围: -32768 ~ 32767,
 *          屏幕区域: 0 ~ OLED_MAX_COLUMN - 1
 * @param y 指定区域左上角的纵坐标, 范围: -32768 ~ 32767,
 *          屏幕区域: 0 ~ OLED_MAX_LINE - 1
 * @param width 指定区域的宽度, 范围: 0 ~ OLED_MAX_COLUMN
 * @param height 指定区域的高度, 范围: 0 ~ OLED_MAX_LINE
 * @note    此函数会至少更新参数指定的区域
 *          如果更新区域 y 轴只包含部分页, 则同一页的剩余部分会跟随一起更新
 *          所有的显示函数, 都只是对 OLED 显存数组进行读写
 *          随后调用 oled_update 函数或 oled_update_area 函数
 *          才会将显存数组的数据发送到 OLED 硬件, 进行显示
 *          故调用显示函数后, 要想真正地呈现在屏幕上, 还需调用更新函数
 */
void oled_update_area(int16_t x, int16_t y, uint8_t width, uint8_t height) {
    int16_t j;
    int16_t page, page1;

    if (oled_is_open == 0) {
        return;
    }

    /* 负数坐标在计算页地址时需要加一个偏移 *
     * (y + height - 1) / 8 + 1 的目的是 (y + height) / 8 并向上取整 */
    page = y / 8;
    page1 = (y + height - 1) / 8 + 1;
    if (y < 0) {
        page -= 1;
        page1 -= 1;
    }

    /* 遍历指定区域涉及的相关页 */
    for (j = page; j < page1; j++) {
        if (x >= 0 && x < OLED_MAX_COLUMN && j >= 0 && j < OLED_MAX_PAGE) {
            /* 超出屏幕的内容不显示 *
             * 设置光标位置为相关页的指定列 */
            oled_set_cursor(j, x);
            /* 连续写入 width 个数据, 将显存数组的数据写入到 OLED 硬件 */
            oled_write_data(&oled_display_buf[j][x], width);
        }
    }
}

/**
 * @brief 将 OLED 显存数组全部清零
 *
 * @note 调用此函数后, 要想真正地呈现在屏幕上, 还需调用更新函数
 */
void oled_clear(void) {
    uint8_t i, j;
    for (j = 0; j < OLED_MAX_PAGE; j++) {
        for (i = 0; i < OLED_MAX_COLUMN; i++) {
            oled_display_buf[j][i] = 0x00;
        }
    }
}

/**
 * @brief 将 OLED 显存数组部分清零
 *
 * @param x 指定区域左上角的横坐标, 范围: -32768 ~ 32767,
 *          屏幕区域: 0 ~ OLED_MAX_COLUMN - 1
 * @param y 指定区域左上角的纵坐标, 范围: -32768 ~ 32767,
 *          屏幕区域: 0 ~ OLED_MAX_LINE - 1
 * @param width 指定区域的宽度, 范围: 0 ~ OLED_MAX_COLUMN
 * @param height 指定区域的高度, 范围: 0 ~ OLED_MAX_LINE
 * @note 调用此函数后, 要想真正地呈现在屏幕上, 还需调用更新函数
 */
void oled_clear_area(int16_t x, int16_t y, uint8_t width, uint8_t height) {
    int16_t i, j;

    for (j = y; j < y + height; j++) {
        for (i = x; i < x + width; i++) {
            if (i >= 0 && i < OLED_MAX_COLUMN && j >= 0 && j < OLED_MAX_LINE) {
                /* 超出屏幕的内容不显示 *
                 * 将显存数组指定数据清零 */
                oled_display_buf[j >> 3][i] &= ~(0x01 << (j & 0x7));
            }
        }
    }
}

/**
 * @brief 将 OLED 显存数组全部取反
 *
 * @note 调用此函数后, 要想真正地呈现在屏幕上, 还需调用更新函数
 */
void oled_reserve(void) {
    uint8_t i, j;
    for (j = 0; j < OLED_MAX_PAGE; j++) {
        for (i = 0; i < OLED_MAX_COLUMN; i++) {
            oled_display_buf[j][i] ^= 0xFF; /* 将显存数组数据全部取反 */
        }
    }
}

/**
 * @brief 将 OLED 显存数组部分取反
 *
 * @param x 指定区域左上角的横坐标, 范围: -32768 ~ 32767,
 *          屏幕区域: 0 ~ OLED_MAX_COLUMN - 1
 * @param y 指定区域左上角的纵坐标, 范围: -32768 ~ 32767,
 *          屏幕区域: 0 ~ OLED_MAX_LINE - 1
 * @param width 指定区域的宽度, 范围: 0 ~ OLED_MAX_COLUMN
 * @param height 指定区域的高度, 范围: 0 ~ OLED_MAX_LINE
 * @note 调用此函数后, 要想真正地呈现在屏幕上, 还需调用更新函数
 */
void oled_reserve_area(int16_t x, int16_t y, uint8_t width, uint8_t height) {
    int16_t i, j;

    for (j = y; j < y + height; j++) {
        for (i = x; i < x + width; i++) {
            if (i >= 0 && i < OLED_MAX_COLUMN && j >= 0 && j < OLED_MAX_LINE) {
                /* 超出屏幕的内容不显示
                 * 将显存数组指定数据取反 */
                oled_display_buf[j >> 3][i] ^= 0x01 << (j & 0x7);
            }
        }
    }
}

/**
 * @brief OLED 显示一个字符
 *
 * @param x 指定字符左上角的横坐标, 范围: -32768 ~ 32767,
 *          屏幕区域: 0 ~ OLED_MAX_COLUMN - 1
 * @param y 指定字符左上角的纵坐标, 范围: -32768 ~ 32767,
 *          屏幕区域: 0 ~ OLED_MAX_LINE - 1
 * @param ch 指定要显示的字符, 范围: ASCII 码可见字符
 * @param font_size 指定字体大小
 *  @arg OLED_6X8  6x8 像素
 *  @arg OLED_8X16 8x16 像素
 * @note 调用此函数后, 要想真正地呈现在屏幕上, 还需调用更新函数
 */
void oled_show_char(int16_t x, int16_t y, char ch, uint8_t font_size) {
    if (font_size == OLED_8X16) {
        /* 将 ASCII 字模库 OLED_F8x16 的指定数据以 8*16 的图像格式显示 */
        oled_show_image(x, y, 8, 16, OLED_F8x16[ch - ' ']);
    } else if (font_size == OLED_6X8) {
        /* 将 ASCII 字模库 OLED_F6x8 的指定数据以 6*8 的图像格式显示 */
        oled_show_image(x, y, 6, 8, OLED_F6x8[ch - ' ']);
    }
}

/**
 * @brief OLED 显示字符串
 *
 * @param x 指定字符串左上角的横坐标, 范围: -32768 ~ 32767,
 *          屏幕区域: 0 ~ OLED_MAX_COLUMN - 1
 * @param y 指定字符串左上角的纵坐标, 范围: -32768 ~ 32767,
 *          屏幕区域: 0 ~ OLED_MAX_LINE - 1
 * @param str 指定要显示的字符串, 范围: ASCII 码可见字符组成的字符串
 * @param font_size 指定字体大小
 *  @arg OLED_6X8  6x8 像素
 *  @arg OLED_8X16 8x16 像素
 * @note 调用此函数后, 要想真正地呈现在屏幕上, 还需调用更新函数
 */
void oled_show_string(int16_t x, int16_t y, char *str, uint8_t font_size) {
    uint16_t i = 0;
    char single_char[5];
    uint8_t char_length = 0;
    int16_t x_offset = 0;
    uint16_t y_offset = 0;
    uint16_t char_index;

    while (str[i] != '\0') {

        if (str[i] == '\t') {
            x_offset += font_size * (OLED_TAB_SIZE -
                                     ((x_offset / font_size) % OLED_TAB_SIZE));
            ++i;
            continue;
        }

        if (str[i] == '\n') {
            /* 新的一行 */
            y_offset += (font_size == OLED_6X8) ? 8 : 16;
            if (x_offset > 0) {
                x_offset = 0;
            }
            ++i;
            continue;
        }

        if (str[i] == '\r') {
            /* 到行首, 直接到屏幕 x 轴的起始位置 */
            x_offset = -x;
            ++i;
            continue;
        }

        if (y + y_offset > OLED_MAX_LINE) {
            /* 显示不下 */
            return;
        }

#if defined(OLED_CHARSET_UTF8)
        /* 提取UTF8字符串中的一个字符, 转存到 single_char 子字符串中 */

        /* 判断UTF8编码第一个字节的标志位 */
        if ((str[i] & 0x80) == 0x00) {
            /* 第一个字节为 0xxxxxxx */
            char_length = 1;
            single_char[0] = str[i++];
            single_char[1] = '\0';
        } else if ((str[i] & 0xE0) == 0xC0) {
            /* 第一个字节为 110xxxxx */
            char_length = 2;
            single_char[0] = str[i++];
            if (str[i] == '\0') {
                /* 意外情况, 跳出循环, 结束显示 */
                break;
            }
            single_char[1] = str[i++];
            single_char[2] = '\0';
        } else if ((str[i] & 0xF0) == 0xE0) {
            /* 第一个字节为 1110xxxx */
            char_length = 3;
            single_char[0] = str[i++];
            if (str[i] == '\0') {
                break;
            }
            single_char[1] = str[i++];
            if (str[i] == '\0') {
                break;
            }
            single_char[2] = str[i++];
            single_char[3] = '\0';
        } else if ((str[i] & 0xF8) == 0xF0) {
            /* 第一个字节为 11110xxx */
            char_length = 4;
            single_char[0] = str[i++];
            if (str[i] == '\0') {
                break;
            }
            single_char[1] = str[i++];
            if (str[i] == '\0') {
                break;
            }
            single_char[2] = str[i++];
            if (str[i] == '\0') {
                break;
            }
            single_char[3] = str[i++];
            single_char[4] = '\0';
        } else {
            i++; /* 意外情况, i指向下一个字节, 忽略此字节, 继续判断下一个字节 */
            continue;
        }
#elif defined(OLED_CHARSET_GB2312)
        /* 提取GB2312字符串中的一个字符, 转存到 single_char 子字符串中 */

        /*  判断GB2312字节的最高位标志位 */
        if ((str[i] & 0x80) == 0x00) {
            char_length = 1;
            single_char[0] = str[i++];
            single_char[1] = '\0';
        } else {
            char_length = 2;
            single_char[0] = str[i++];
            if (str[i] == '\0') {
                /* 意外情况, 跳出循环, 结束显示 */
                break;
            }
            single_char[1] = str[i++];
            single_char[2] = '\0';
        }
#endif /* OLED_CHARSET */

        if (x + x_offset + font_size * (char_length > 1 ? 2 : 1) >
            OLED_MAX_COLUMN) {
            /* 超出显示区域, 从头开始 */
            x_offset = 0;
            y_offset += (font_size == OLED_6X8) ? 8 : 16;
        }

        /* 显示上述代码提取到的 single_char */
        if (char_length == 1) {
            oled_show_char(x + x_offset, y + y_offset, single_char[0],
                           font_size);
            x_offset += font_size;
        } else {
            for (char_index = 0;
                 strcmp(OLED_CF16x16[char_index].index, "") != 0;
                 char_index++) {
                /* 找到匹配的字符 */
                if (strcmp(OLED_CF16x16[char_index].index, single_char) == 0) {
                    break;
                }
            }

            if (font_size == OLED_8X16) {
                /* 将字模库 OLED_CF16x16 的指定数据以 16*16 的图像格式显示 */
                oled_show_image(x + x_offset, y + y_offset, 16, 16,
                                OLED_CF16x16[char_index].data);
                x_offset += 16;
            } else if (font_size == OLED_6X8) {
                /* 空间不足, 此位置显示 '?' */
                oled_show_char(x + x_offset, y + y_offset, '?', OLED_6X8);
                x_offset += OLED_6X8;
            }
        }
    }
}

/**
 * @brief OLED 显示数字 (十进制, 正整数)
 *
 * @param x 指定数字左上角的横坐标, 范围: -32768 ~ 32767,
 *          屏幕区域: 0 ~ OLED_MAX_COLUMN - 1
 * @param y 指定数字左上角的纵坐标, 范围: -32768 ~ 32767,
 *          屏幕区域: 0 ~ OLED_MAX_LINE - 1
 * @param number 指定要显示的数字, 范围: 0 ~ 4294967295
 * @param length 指定数字的长度, 范围: 0 ~ 10
 * @param font_size 指定字体大小
 *  @arg OLED_6X8  6x8 像素
 *  @arg OLED_8X16 8x16 像素
 * @note 调用此函数后, 要想真正地呈现在屏幕上, 还需调用更新函数
 */
void oled_show_number(int16_t x, int16_t y, uint32_t number, uint8_t length,
                      uint8_t font_size) {
    uint8_t i;
    for (i = 0; i < length; i++) {
        /* 调用 OLED_Showch 函数, 依次显示每个数字 */
        oled_show_char(x + i * font_size, y,
                       number / oled_pow(10, length - i - 1) % 10 + '0',
                       font_size);
    }
}

/**
 * @brief OLED 显示有符号数字 (十进制, 整数)
 *
 * @param x 指定数字左上角的横坐标, 范围: -32768 ~ 32767,
 *          屏幕区域: 0 ~ OLED_MAX_COLUMN - 1
 * @param y 指定数字左上角的纵坐标, 范围: -32768 ~ 32767,
 *          屏幕区域: 0 ~ OLED_MAX_LINE - 1
 * @param number 指定要显示的数字, 范围: -2147483648 ~ 2147483647
 * @param length 指定数字的长度, 范围: 0 ~ 10
 * @param font_size 指定字体大小
 *  @arg OLED_6X8  6x8 像素
 *  @arg OLED_8X16 8x16 像素
 * @note 调用此函数后, 要想真正地呈现在屏幕上, 还需调用更新函数
 */
void oled_show_signed_number(int16_t x, int16_t y, int32_t number,
                             uint8_t length, uint8_t font_size) {
    uint8_t i;
    uint32_t show_num;

    if (number > 0) {
        oled_show_char(x, y, '+', font_size); /* 显示 + 号 */
        show_num = number;
    } else if (number < 0) {
        oled_show_char(x, y, '-', font_size); /* 显示 - 号 */
        show_num = -number;
    } else {
        show_num = 0;
    }

    for (i = 0; i < length; i++) {
        /* show_num / oled_pow(10, length - i - 1) % 10 *
         * 可以十进制提取数字的每一位 + '0' 可将数字转换为字符格式 */
        oled_show_char(x + (i + 1) * font_size, y,
                       show_num / oled_pow(10, length - i - 1) % 10 + '0',
                       font_size);
    }
}

/**
 * @brief OLED 显示十六进制数字 (十六进制, 正整数)
 *
 * @param x 指定数字左上角的横坐标, 范围: -32768 ~ 32767,
 *          屏幕区域: 0 ~ OLED_MAX_COLUMN - 1
 * @param y 指定数字左上角的纵坐标, 范围: -32768 ~ 32767,
 *          屏幕区域: 0 ~ OLED_MAX_LINE - 1
 * @param number 指定要显示的数字, 范围: 0x00000000 ~ 0xFFFFFFFF
 * @param length 指定数字的长度, 范围: 0 ~ OLED_MAX_PAGE
 * @param font_size 指定字体大小
 *  @arg OLED_6X8  6x8 像素
 *  @arg OLED_8X16 8x16 像素
 * @note 调用此函数后, 要想真正地呈现在屏幕上, 还需调用更新函数
 */
void oled_show_hex(int16_t x, int16_t y, uint32_t number, uint8_t length,
                   uint8_t font_size) {
    uint8_t i, single_number;
    for (i = 0; i < length; i++) {
        /* 以十六进制提取数字的每一位 */
        single_number = number / oled_pow(16, length - i - 1) % 16;

        if (single_number < 10) {
            oled_show_char(x + i * font_size, y, single_number + '0',
                           font_size);
        } else {
            oled_show_char(x + i * font_size, y, single_number - 10 + 'A',
                           font_size);
        }
    }
}

/**
 * @brief OLED 显示二进制数字 (二进制, 正整数)
 *
 * @param x 指定数字左上角的横坐标, 范围: -32768 ~ 32767,
 *          屏幕区域: 0 ~ OLED_MAX_COLUMN - 1
 * @param y 指定数字左上角的纵坐标, 范围: -32768 ~ 32767,
 *          屏幕区域: 0 ~ OLED_MAX_LINE - 1
 * @param number 指定要显示的数字, 范围: 0x00000000 ~ 0xFFFFFFFF
 * @param length 指定数字的长度, 范围: 0 ~ 16
 * @param font_size 指定字体大小
 *  @arg OLED_6X8  6x8 像素
 *  @arg OLED_8X16 8x16 像素
 * @note 调用此函数后, 要想真正地呈现在屏幕上, 还需调用更新函数
 */
void oled_show_bin(int16_t x, int16_t y, uint32_t number, uint8_t length,
                   uint8_t font_size) {
    uint8_t i;
    for (i = 0; i < length; i++) {
        oled_show_char(x + i * font_size, y,
                       number / oled_pow(2, length - i - 1) % 2 + '0',
                       font_size);
    }
}

/**
 * @brief OLED 显示浮点数字 (十进制, 小数)
 *
 * @param x 指定数字左上角的横坐标, 范围: -32768 ~ 32767,
 *          屏幕区域: 0 ~ OLED_MAX_COLUMN - 1
 * @param y 指定数字左上角的纵坐标, 范围: -32768 ~ 32767,
 *          屏幕区域: 0 ~ OLED_MAX_LINE - 1
 * @param number 指定要显示的数字, 范围: -4294967295.0 ~ 4294967295.0
 * @param int_length 指定数字的整数位长度, 范围: 0 ~ 10
 * @param float_length 指定数字的小数位长度, 范围: 0 ~ 9, 小数进行四舍五入显示
 *  @arg OLED_6X8  6x8 像素
 *  @arg OLED_8X16 8x16 像素
 * @note 调用此函数后, 要想真正地呈现在屏幕上, 还需调用更新函数
 */
void oled_show_float(int16_t x, int16_t y, double number, uint8_t int_length,
                     uint8_t float_length, uint8_t font_size) {
    uint32_t pow_number, int_number, float_number;

    if (number > 0) {
        oled_show_char(x, y, '+', font_size); /* 显示 + 号 */
    } else {
        oled_show_char(x, y, '-', font_size); /* 显示 - 号 */
        number = -number;                     /* 取负 */
    }

    /* 提取整数部分和小数部分 */
    int_number = (int)number; /* 直接赋值给整型变量, 提取整数 */
    /* 将 number 的整数减掉, 防止之后将小数乘到整数时因数过大造成错误 */
    number -= int_number;
    pow_number = oled_pow(10, float_length); /* 根据指定小数的位数, 确定乘数 */
    /* 将小数乘到整数, 同时四舍五入, 避免显示误差 */
    float_number = (int)round(number * pow_number);
    /* 若四舍五入造成了进位, 则需要再加给整数 */
    int_number += float_number / pow_number;

    /* 显示整数部分 */
    oled_show_number(x + font_size, y, int_number, int_length, font_size);

    /* 显示小数点 */
    oled_show_char(x + (int_length + 1) * font_size, y, '.', font_size);

    /* 显示小数部分 */
    oled_show_number(x + (int_length + 2) * font_size, y, float_number,
                     float_length, font_size);
}

/**
 * @brief OLED 显示图像
 *
 * @param x 指定图像左上角的横坐标, 范围: -32768 ~ 32767,
 *          屏幕区域: 0 ~ OLED_MAX_COLUMN - 1
 * @param y 指定图像左上角的纵坐标, 范围: -32768 ~ 32767,
 *          屏幕区域: 0 ~ OLED_MAX_LINE - 1
 * @param width 指定图像的宽度, 范围: 0 ~ OLED_MAX_COLUMN
 * @param height 指定图像的高度, 范围: 0 ~ OLED_MAX_LINE
 * @param image 指定要显示的图像
 * @note 调用此函数后, 要想真正地呈现在屏幕上, 还需调用更新函数
 */
void oled_show_image(int16_t x, int16_t y, uint8_t width, uint8_t height,
                     const uint8_t *image) {
    uint8_t i = 0, j = 0;
    int16_t page, shift;

    /* 将图像所在区域清空 */
    oled_clear_area(x, y, width, height);

    /* 遍历指定图像涉及的相关页 *
     * (height - 1) / 8 + 1 的目的是 height / 8 并向上取整 */
    for (j = 0; j < (height - 1) / 8 + 1; j++) {
        /* 遍历指定图像涉及的相关列 */
        for (i = 0; i < width; i++) {
            if (x + i >= 0 && x + i < OLED_MAX_COLUMN) {
                /* 超出屏幕的内容不显示 *
                 * 负数坐标在计算页地址和移位时需要加一个偏移 */
                page = y >> 3;
                shift = y & 0x7;
                if (y < 0) {
                    page -= 1;
                    shift += 8;
                }

                if (page + j >= 0 && page + j < OLED_MAX_PAGE) {
                    /* 超出屏幕的内容不显示
                     * 显示图像在当前页的内容 */
                    oled_display_buf[page + j][x + i] |= image[j * width + i]
                                                         << (shift);
                }

                if (page + j + 1 >= 0 && page + j + 1 < OLED_MAX_PAGE) {
                    /* 超出屏幕的内容不显示 *
                     * 显示图像在下一页的内容 */
                    oled_display_buf[page + j + 1][x + i] |=
                        image[j * width + i] >> (8 - shift);
                }
            }
        }
    }
}

/**
 * @brief OLED 使用 printf 函数打印格式化字符串
 *
 * @param x 指定格式化字符串左上角的横坐标, 范围: -32768 ~ 32767,
 *          屏幕区域: 0 ~ OLED_MAX_COLUMN - 1
 * @param y 指定格式化字符串左上角的纵坐标, 范围: -32768 ~ 32767,
 *          屏幕区域: 0 ~ OLED_MAX_LINE - 1
 * @param font_size 字体大小
 *  @arg OLED_6X8  6x8 像素
 *  @arg OLED_8X16 8x16 像素
 * @param format 指定要显示的格式化字符串, 范围: ASCII 码可见字符组成的字符串
 * @note 调用此函数后, 要想真正地呈现在屏幕上, 还需调用更新函数
 */
void oled_printf(int16_t x, int16_t y, uint8_t font_size, char *format, ...) {
    /* 此处是一个 static 的变量, 需要做线程安全处理 */
    static char str[128];

    xSemaphoreTake(show_semp, portMAX_DELAY);

    va_list arg;
    va_start(arg, format);

    vsnprintf(str, sizeof(str), format, arg);
    va_end(arg);
    oled_show_string(x, y, str, font_size);

    xSemaphoreGive(show_semp);
}

/**
 * @brief OLED 在指定位置画一个点
 *
 * @param x 指定点的横坐标, 范围: -32768 ~ 32767,
 *          屏幕区域: 0 ~ OLED_MAX_COLUMN - 1
 * @param y 指定点的纵坐标, 范围: -32768 ~ 32767,
 *          屏幕区域: 0 ~ OLED_MAX_LINE - 1
 * @note 调用此函数后, 要想真正地呈现在屏幕上, 还需调用更新函数
 */
void oled_draw_point(int16_t x, int16_t y) {
    if (x >= 0 && x < OLED_MAX_COLUMN && y >= 0 && y < OLED_MAX_LINE) {
        /* 超出屏幕的内容不显示
         * 将显存数组指定位置的一个 Bit 数据置 1 */
        oled_display_buf[y >> 3][x] |= 0x01 << (y & 0x7);
    }
}

/**
 * @brief OLED 获取指定位置点的值
 *
 * @param x 指定点的横坐标, 范围: -32768 ~ 32767,
 *          屏幕区域: 0 ~ OLED_MAX_COLUMN - 1
 * @param y 指定点的纵坐标, 范围: -32768 ~ 32767,
 *          屏幕区域: 0 ~ OLED_MAX_LINE - 1
 * @return 指定位置点是否处于点亮状态, 1: 点亮, 0: 熄灭
 */
uint8_t oled_get_point(int16_t x, int16_t y) {
    if (x >= 0 && x < OLED_MAX_COLUMN && y >= 0 && y < OLED_MAX_LINE) {
        /* 超出屏幕的内容不读取
         * 判断指定位置的数据 */
        if (oled_display_buf[y >> 3][x] & 0x01 << (y & 0x7)) {
            return 1;
        }
    }

    return 0;
}

/**
 * @brief OLED 画线
 *
 * @param x0 指定一个端点的横坐标, 范围: -32768 ~ 32767,
 *          屏幕区域: 0 ~ OLED_MAX_COLUMN - 1
 * @param y0 指定一个端点的纵坐标, 范围: -32768 ~ 32767,
 *           屏幕区域: 0 ~ OLED_MAX_LINE - 1
 * @param x1 指定另一个端点的横坐标, 范围: -32768 ~ 32767,
 *           屏幕区域: 0 ~ OLED_MAX_COLUMN - 1
 * @param y1 指定另一个端点的纵坐标, 范围: -32768 ~ 32767,
 *           屏幕区域: 0 ~ OLED_MAX_LINE - 1
 * @note 调用此函数后, 要想真正地呈现在屏幕上, 还需调用更新函数
 */
void oled_draw_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1) {
    int16_t x, y, dx, dy, d, incrE, incrNE, temp;
    uint8_t yflag = 0, xyflag = 0;

    if (y0 == y1) {
        /* 横线单独处理 */

        /* 0 号点 X 坐标大于 1 号点 X 坐标, 则交换两点 X 坐标 */
        if (x0 > x1) {
            temp = x0;
            x0 = x1;
            x1 = temp;
        }

        /* 遍历 X 坐标 */
        for (x = x0; x <= x1; x++) {
            oled_draw_point(x, y0); /* 依次画点 */
        }
    } else if (x0 == x1) {
        /* 竖线单独处理 */

        /* 0 号点 y 坐标大于 1 号点 y 坐标, 则交换两点 y 坐标 */
        if (y0 > y1) {
            temp = y0;
            y0 = y1;
            y1 = temp;
        }

        /* 遍历 y 坐标 */
        for (y = y0; y <= y1; y++) {
            oled_draw_point(x0, y); /* 依次画点 */
        }
    } else {
        /* 使用 Bresenham 算法画直线, 可以避免耗时的浮点运算, 效率更高
         * 参考文档:https://www.cs.montana.edu/courses/spring2009/425/dslectures/Bresenham.pdf
         * 参考教程:https://www.bilibili.com/video/BV1364y1d7Lo */

        if (x0 > x1) {
            /* 交换两点坐标, 交换后不影响画线,
             * 但是画线方向由第一, 二, 三, 四象限变为第一, 四象限 */
            temp = x0;
            x0 = x1;
            x1 = temp;
            temp = y0;
            y0 = y1;
            y1 = temp;
        }

        if (y0 > y1) {
            /* 将 y 坐标取负, 取负后影响画线, 但是画线方向由第一,
             * 四象限变为第一象限 */
            y0 = -y0;
            y1 = -y1;

            /* 置标志位 yflag, 记住当前变换, 在后续实际画线时, 再将坐标换回来 */
            yflag = 1;
        }

        if (y1 - y0 > x1 - x0) {
            /* 将 X 坐标与 y 坐标互换 */
            /* 互换后影响画线,
             * 但是画线方向由第一象限 0 ~ 90 度范围变为第一象限 0 ~ 45 度范围 */
            temp = x0;
            x0 = y0;
            y0 = temp;
            temp = x1;
            x1 = y1;
            y1 = temp;

            /* 置标志位 xyflag, 记住当前变换, 在后续实际画线时, 再将坐标换回来
             */
            xyflag = 1;
        }

        /* 以下为 Bresenham 算法画直线 */
        /* 算法要求, 画线方向必须为第一象限 0 ~ 45 度范围 */
        dx = x1 - x0;
        dy = y1 - y0;
        incrE = 2 * dy;
        incrNE = 2 * (dy - dx);
        d = 2 * dy - dx;
        x = x0;
        y = y0;

        /* 画起始点, 同时判断标志位, 将坐标换回来 */
        if (yflag && xyflag) {
            oled_draw_point(y, -x);
        } else if (yflag) {
            oled_draw_point(x, -y);
        } else if (xyflag) {
            oled_draw_point(y, x);
        } else {
            oled_draw_point(x, y);
        }

        while (x < x1) {
            /* 遍历 X 轴的每个点 */
            x++;
            if (d < 0) {
                /* 下一个点在当前点东方 */
                d += incrE;
            } else {
                /* 下一个点在当前点东北方 */
                y++;
                d += incrNE;
            }

            /* 画每一个点, 同时判断标志位, 将坐标换回来 */
            if (yflag && xyflag) {
                oled_draw_point(y, -x);
            } else if (yflag) {
                oled_draw_point(x, -y);
            } else if (xyflag) {
                oled_draw_point(y, x);
            } else {
                oled_draw_point(x, y);
            }
        }
    }
}

/**
 * @brief OLED 矩形
 *
 * @param x 指定矩形左上角的横坐标, 范围: -32768 ~ 32767,
 *          屏幕区域: 0 ~ OLED_MAX_COLUMN - 1
 * @param y 指定矩形左上角的纵坐标, 范围: -32768 ~ 32767,
 *          屏幕区域: 0 ~ OLED_MAX_LINE - 1
 * @param width 指定矩形的宽度, 范围: 0 ~ OLED_MAX_COLUMN
 * @param height 指定矩形的高度, 范围: 0 ~ OLED_MAX_LINE
 * @param is_filled 指定矩形是否填充
 *  @arg OLED_UNFILLED 不填充
 *  @arg OLED_FILLED 填充
 * @note 调用此函数后, 要想真正地呈现在屏幕上, 还需调用更新函数
 */
void oled_draw_rectangle(int16_t x, int16_t y, uint8_t width, uint8_t height,
                         uint8_t is_filled) {
    int16_t i, j;
    if (!is_filled) {
        /* 指定矩形不填充 */

        /* 遍历上下 X 坐标, 画矩形上下两条线 */
        for (i = x; i < x + width; i++) {
            oled_draw_point(i, y);
            oled_draw_point(i, y + height - 1);
        }
        /* 遍历左右 y 坐标, 画矩形左右两条线 */
        for (i = y; i < y + height; i++) {
            oled_draw_point(x, i);
            oled_draw_point(x + width - 1, i);
        }
    } else {
        /* 指定矩形填充 */

        /* 遍历 X 坐标 */
        for (i = x; i < x + width; i++) {
            /* 遍历 y 坐标 */
            for (j = y; j < y + height; j++) {
                /* 在指定区域画点, 填充满矩形 */
                oled_draw_point(i, j);
            }
        }
    }
}

/**
 * @brief OLED 三角形
 *
 * @param x0 指定第一个端点的横坐标, 范围: -32768 ~ 32767,
 *           屏幕区域: 0 ~ OLED_MAX_COLUMN - 1
 * @param y0 指定第一个端点的纵坐标, 范围: -32768 ~ 32767,
 *           屏幕区域: 0 ~ OLED_MAX_LINE - 1
 * @param x1 指定第二个端点的横坐标, 范围: -32768 ~ 32767,
 *           屏幕区域: 0 ~ OLED_MAX_COLUMN - 1
 * @param y1 指定第二个端点的纵坐标, 范围: -32768 ~ 32767,
 *           屏幕区域: 0 ~ OLED_MAX_LINE - 1
 * @param x2 指定第三个端点的横坐标, 范围: -32768 ~ 32767,
 *           屏幕区域: 0 ~ OLED_MAX_COLUMN - 1
 * @param y2 指定第三个端点的纵坐标, 范围: -32768 ~ 32767,
 *           屏幕区域: 0 ~ OLED_MAX_LINE - 1
 * @param is_filled 指定三角形是否填充
 *  @arg OLED_UNFILLED 不填充
 *  @arg OLED_FILLED 填充
 * @note 调用此函数后, 要想真正地呈现在屏幕上, 还需调用更新函数
 */
void oled_draw_tritangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1,
                         int16_t x2, int16_t y2, uint8_t is_filled) {
    int16_t minx = x0, miny = y0, maxx = x0, maxy = y0;
    int16_t i, j;
    int16_t vx[] = {x0, x1, x2};
    int16_t vy[] = {y0, y1, y2};

    if (!is_filled) {
        /* 指定三角形不填充 */

        /* 调用画线函数, 将三个点用直线连接 */
        oled_draw_line(x0, y0, x1, y1);
        oled_draw_line(x0, y0, x2, y2);
        oled_draw_line(x1, y1, x2, y2);
    } else {
        /* 指定三角形填充 */

        /* 找到三个点最小的 x, y 坐标 */
        if (x1 < minx) {
            minx = x1;
        }
        if (x2 < minx) {
            minx = x2;
        }
        if (y1 < miny) {
            miny = y1;
        }
        if (y2 < miny) {
            miny = y2;
        }

        /* 找到三个点最大的 x, y 坐标 */
        if (x1 > maxx) {
            maxx = x1;
        }
        if (x2 > maxx) {
            maxx = x2;
        }
        if (y1 > maxy) {
            maxy = y1;
        }
        if (y2 > maxy) {
            maxy = y2;
        }

        /* 最小最大坐标之间的矩形为可能需要填充的区域 遍历此区域中所有的点 */

        /* 遍历 x 坐标 */
        for (i = minx; i <= maxx; i++) {
            /* 遍历 y 坐标 */
            for (j = miny; j <= maxy; j++) {
                /* 调用 oled_pnpoly, 判断指定点是否在指定三角形之中 */

                /* 如果在, 则画点, 如果不在, 则不做处理 */
                if (oled_pnpoly(3, vx, vy, i, j)) {
                    oled_draw_point(i, j);
                }
            }
        }
    }
}

/**
 * @brief OLED 画圆
 *
 * @param x 指定圆的圆心横坐标, 范围: -32768 ~ 32767,
 *          屏幕区域: 0 ~ OLED_MAX_COLUMN - 1
 * @param y 指定圆的圆心纵坐标, 范围: -32768 ~ 32767,
 *          屏幕区域: 0 ~ OLED_MAX_LINE - 1
 * @param radius 指定圆的半径, 范围: 0 ~ 255
 * @param is_filled 指定圆是否填充
 *  @arg OLED_UNFILLED 不填充
 *  @arg OLED_FILLED 填充
 * @note 调用此函数后, 要想真正地呈现在屏幕上, 还需调用更新函数
 */
void oled_draw_circle(int16_t x, int16_t y, uint8_t radius, uint8_t is_filled) {

    /* 使用 Bresenham 算法画圆, 可以避免耗时的浮点运算, 效率更高
     * 参考文档:https://www.cs.montana.edu/courses/spring2009/425/dslectures/Bresenham.pdf
     * 参考教程:https://www.bilibili.com/video/BV1VM4y1u7wJ */
    int16_t px, py, d, j;

    d = 1 - radius;
    px = 0;
    py = radius;

    /* 画每个八分之一圆弧的起始点 */
    oled_draw_point(x + px, y + py);
    oled_draw_point(x - px, y - py);
    oled_draw_point(x + py, y + px);
    oled_draw_point(x - py, y - px);

    if (is_filled) {
        /* 遍历起始点 Y 坐标 */
        for (j = -py; j < py; j++) {
            /* 在指定区域画点, 填充部分圆 */
            oled_draw_point(x, y + j);
        }
    }

    while (px < py) {
        px++;
        if (d < 0) {
            /* 下一个点在当前点东方 */
            d += 2 * px + 1;
        } else {
            /* 下一个点在当前点东南方 */
            py--;
            d += 2 * (px - py) + 1;
        }

        /* 画每个八分之一圆弧的点 */
        oled_draw_point(x + px, y + py);
        oled_draw_point(x + py, y + px);
        oled_draw_point(x - px, y - py);
        oled_draw_point(x - py, y - px);
        oled_draw_point(x + px, y - py);
        oled_draw_point(x + py, y - px);
        oled_draw_point(x - px, y + py);
        oled_draw_point(x - py, y + px);

        if (is_filled) {
            /* 遍历中间部分 */
            for (j = -py; j < py; j++) {
                /* 在指定区域画点, 填充部分圆 */
                oled_draw_point(x + px, y + j);
                oled_draw_point(x - px, y + j);
            }

            /* 遍历两侧部分 */
            for (j = -px; j < px; j++) {
                /* 在指定区域画点, 填充部分圆 */
                oled_draw_point(x - py, y + j);
                oled_draw_point(x + py, y + j);
            }
        }
    }
}

/**
 * @brief OLED 画椭圆
 *
 * @param x 指定椭圆的圆心横坐标, 范围: -32768 ~ 32767,
 *          屏幕区域: 0 ~ OLED_MAX_COLUMN - 1
 * @param y 指定椭圆的圆心纵坐标, 范围: -32768 ~ 32767,
 *          屏幕区域: 0 ~ OLED_MAX_LINE - 1
 * @param a 指定椭圆的横向半轴长度, 范围: 0 ~ 255
 * @param b 指定椭圆的纵向半轴长度, 范围: 0 ~ 255
 * @param is_filled 指定椭圆是否填充
 *  @arg OLED_UNFILLED 不填充
 *  @arg OLED_FILLED 填充
 * @note 调用此函数后, 要想真正地呈现在屏幕上, 还需调用更新函数
 */
void oled_draw_ellipse(int16_t x, int16_t y, uint8_t a, uint8_t b,
                       uint8_t is_filled) {

    /* 使用 Bresenham 算法画椭圆, 可以避免部分耗时的浮点运算, 效率更高
     * 参考链接:https://blog.csdn.net/myf_666/article/details/128167392 */

    int16_t px, py, j;
    float d1, d2;

    px = 0;
    py = b;
    d1 = b * b + a * a * (-b + 0.5);

    if (is_filled) {
        /* 遍历起始点 y 坐标 */
        for (j = -py; j < py; j++) {
            /* 在指定区域画点, 填充部分椭圆 */
            oled_draw_point(x, y + j);
            oled_draw_point(x, y + j);
        }
    }

    /* 画椭圆弧的起始点 */
    oled_draw_point(x + px, y + py);
    oled_draw_point(x - px, y - py);
    oled_draw_point(x - px, y + py);
    oled_draw_point(x + px, y - py);

    /* 画椭圆中间部分 */
    while (b * b * (px + 1) < a * a * (py - 0.5)) {
        if (d1 <= 0) {
            /* 下一个点在当前点东方 */
            d1 += b * b * (2 * px + 3);
        } else {
            /* 下一个点在当前点东南方 */
            d1 += b * b * (2 * px + 3) + a * a * (-2 * py + 2);
            py--;
        }
        px++;

        if (is_filled) {
            /* 遍历中间部分 */
            for (j = -py; j < py; j++) {
                /* 在指定区域画点, 填充部分椭圆 */
                oled_draw_point(x + px, y + j);
                oled_draw_point(x - px, y + j);
            }
        }

        /* 画椭圆中间部分圆弧 */
        oled_draw_point(x + px, y + py);
        oled_draw_point(x - px, y - py);
        oled_draw_point(x - px, y + py);
        oled_draw_point(x + px, y - py);
    }

    /* 画椭圆两侧部分 */
    d2 = b * b * (px + 0.5) * (px + 0.5) + a * a * (py - 1) * (py - 1) -
         a * a * b * b;

    while (py > 0) {
        if (d2 <= 0) {
            /* 下一个点在当前点东方 */
            d2 += b * b * (2 * px + 2) + a * a * (-2 * py + 3);
            px++;

        } else {
            /* 下一个点在当前点东南方 */
            d2 += a * a * (-2 * py + 3);
        }
        py--;

        if (is_filled) {
            /* 遍历两侧部分 */
            for (j = -py; j < py; j++) {
                /* 在指定区域画点, 填充部分椭圆 */
                oled_draw_point(x + px, y + j);
                oled_draw_point(x - px, y + j);
            }
        }

        /* 画椭圆两侧部分圆弧 */
        oled_draw_point(x + px, y + py);
        oled_draw_point(x - px, y - py);
        oled_draw_point(x - px, y + py);
        oled_draw_point(x + px, y - py);
    }
}

/**
 * @brief OLED 画圆弧
 * @param x 指定圆弧的圆心横坐标, 范围: -32768 ~ 32767,
 *          屏幕区域: 0 ~ OLED_MAX_COLUMN - 1
 * @param y 指定圆弧的圆心纵坐标, 范围: -32768 ~ 32767,
 *          屏幕区域: 0 ~ OLED_MAX_LINE - 1
 * @param radius 指定圆弧的半径, 范围: 0 ~ 255
 * @param start_angle 指定圆弧的起始角度, 范围: -180 ~ 180
 * @param end_angle 指定圆弧的终止角度, 范围: -180 ~ 180
 * @param is_filled 指定圆弧是否填充, 填充后为扇形
 *  @arg OLED_UNFILLED 不填充
 *  @arg OLED_FILLED 填充
 * @note 水平向右为 0 度, 水平向左为 180 度或 - 180 度, 下方为正数, 上方为负数,
 *       顺时针旋转.
 *       调用此函数后, 要想真正地呈现在屏幕上, 还需调用更新函数
 */
void oled_draw_arc(int16_t x, int16_t y, uint8_t radius, int16_t start_angle,
                   int16_t end_angle, uint8_t is_filled) {
    int16_t px, py, d, j;

    /* 此函数借用 Bresenham 算法画圆的方法 */

    d = 1 - radius;
    px = 0;
    py = radius;

    /* 在画圆的每个点时, 判断指定点是否在指定角度内,
     * 在, 则画点, 不在, 则不做处理 */
    if (oled_is_in_angle(px, py, start_angle, end_angle)) {
        oled_draw_point(x + px, y + py);
    }
    if (oled_is_in_angle(-px, -py, start_angle, end_angle)) {
        oled_draw_point(x - px, y - py);
    }
    if (oled_is_in_angle(py, px, start_angle, end_angle)) {
        oled_draw_point(x + py, y + px);
    }
    if (oled_is_in_angle(-py, -px, start_angle, end_angle)) {
        oled_draw_point(x - py, y - px);
    }

    if (is_filled) {
        /* 遍历起始点 Y 坐标 */
        for (j = -py; j < py; j++) {
            /* 在填充圆的每个点时, 判断指定点是否在指定角度内,
             * 在, 则画点, 不在, 则不做处理 */
            if (oled_is_in_angle(0, j, start_angle, end_angle)) {
                oled_draw_point(x, y + j);
            }
        }
    }

    while (px < py) {
        /* 遍历 X 轴的每个点 */
        px++;
        if (d < 0) {
            /* 下一个点在当前点东方 */
            d += 2 * px + 1;
        } else {
            /* 下一个点在当前点东南方 */
            py--;
            d += 2 * (px - py) + 1;
        }

        /* 在画圆的每个点时, 判断指定点是否在指定角度内,
         * 在, 则画点, 不在, 则不做处理 */
        if (oled_is_in_angle(px, py, start_angle, end_angle)) {
            oled_draw_point(x + px, y + py);
        }
        if (oled_is_in_angle(py, px, start_angle, end_angle)) {
            oled_draw_point(x + py, y + px);
        }
        if (oled_is_in_angle(-px, -py, start_angle, end_angle)) {
            oled_draw_point(x - px, y - py);
        }
        if (oled_is_in_angle(-py, -px, start_angle, end_angle)) {
            oled_draw_point(x - py, y - px);
        }
        if (oled_is_in_angle(px, -py, start_angle, end_angle)) {
            oled_draw_point(x + px, y - py);
        }
        if (oled_is_in_angle(py, -px, start_angle, end_angle)) {
            oled_draw_point(x + py, y - px);
        }
        if (oled_is_in_angle(-px, py, start_angle, end_angle)) {
            oled_draw_point(x - px, y + py);
        }
        if (oled_is_in_angle(-py, px, start_angle, end_angle)) {
            oled_draw_point(x - py, y + px);
        }

        if (is_filled) {
            /* 遍历中间部分 */
            for (j = -py; j < py; j++) {
                /* 在填充圆的每个点时, 判断指定点是否在指定角度内,
                 * 在, 则画点, 不在, 则不做处理 */
                if (oled_is_in_angle(px, j, start_angle, end_angle)) {
                    oled_draw_point(x + px, y + j);
                }
                if (oled_is_in_angle(-px, j, start_angle, end_angle)) {
                    oled_draw_point(x - px, y + j);
                }
            }

            /* 遍历两侧部分 */
            for (j = -px; j < px; j++) {
                /* 在填充圆的每个点时, 判断指定点是否在指定角度内,
                 * 在, 则画点, 不在, 则不做处理 */
                if (oled_is_in_angle(-py, j, start_angle, end_angle)) {
                    oled_draw_point(x - py, y + j);
                }
                if (oled_is_in_angle(py, j, start_angle, end_angle)) {
                    oled_draw_point(x + py, y + j);
                }
            }
        }
    }
}
