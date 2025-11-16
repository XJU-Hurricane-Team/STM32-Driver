/**
 * @file    as5047.c
 * @author  Deadline039
 * @brief   AS5047p驱动
 * @version 1.1
 * @date     2025-11-04
 * 
 */

#include "as5047.h"
#include <assert.h>

/**
 * @brief AS5047配置项目
 */
static struct {
    SPI_HandleTypeDef *hspi; /*!< SPI句柄 */
    GPIO_TypeDef *port;      /*!< 端口 */
    uint16_t pin;            /*!< 引脚 */
} devices[AS5047_MAX_NUM];

as5047_position_t g_as5047_position[AS5047_MAX_NUM]; /*!< 编码器位置信息 */

static uint8_t as5047_num = 0; /*!< 已注册编码器数量 */

/**
 * @brief 检查是否是有效的设备ID
 *
 * @param __devidx 设备ID
 */
#define AS5047_IS_VALID_ID(__devidx)                                           \
    (!(__devidx < 0 && (__devidx > as5047_num)))

/**
 * @brief CS引脚拉高
 *
 * @param __devidx 设备ID
 */
#define CS_HIGH(__devidx)                                                      \
    HAL_GPIO_WritePin(devices[__devidx].port, devices[__devidx].pin,           \
                      GPIO_PIN_SET)

/**
 * @brief CS引脚拉低
 *
 * @param __devidx 设备ID
 */
#define CS_LOW(__devidx)                                                       \
    HAL_GPIO_WritePin(devices[__devidx].port, devices[__devidx].pin,           \
                      GPIO_PIN_RESET)

/**
 * @brief 打开GPIO时钟
 *
 *@param PORT GPIO端口
 */
#define GPIO_CLK_ENABLE(PORT)                                                  \
    do {                                                                       \
        if (PORT == GPIOA) {                                                   \
            __HAL_RCC_GPIOA_CLK_ENABLE();                                      \
        } else if (PORT == GPIOB) {                                            \
            __HAL_RCC_GPIOB_CLK_ENABLE();                                      \
        } else if (PORT == GPIOC) {                                            \
            __HAL_RCC_GPIOC_CLK_ENABLE();                                      \
        } else if (PORT == GPIOD) {                                            \
            __HAL_RCC_GPIOD_CLK_ENABLE();                                      \
        }                                                                      \
    } while (0)

/**
 * @brief 偶校验
 *
 * @param data 待校验的数据
 * @return 校验结果
 */
static uint16_t parity_bit_calculate(uint16_t data) {
    uint16_t parity_bit_value = 0;
    while (data != 0) {
        parity_bit_value ^= data;
        data >>= 1;
    }
    return (parity_bit_value & 0x1);
}

/**
 * @brief SPI读写 1 byte 数据
 *
 * @param devidx 设备ID
 * @param _txdata 发送的数据
 * @return 收到的数据
 */
static uint16_t spi_rw_byte(int devidx, uint16_t _txdata) {
    uint16_t rxdata;
#ifdef DEBUG
    assert(AS5047_IS_VALID_ID(devidx));
#endif /* DEBUG */

    CS_LOW(devidx);
    if (HAL_SPI_TransmitReceive(devices[devidx].hspi, (uint8_t *)&_txdata,
                                (uint8_t *)&rxdata, 1, 100) != HAL_OK) {
        rxdata = 0;
    }
    CS_HIGH(devidx);

    return rxdata;
}

/**
 * @brief 发送数据
 *
 * @param devidx 设备ID
 * @param addr 地址
 * @param data 数据
 * @return 收到的数据
 */
uint16_t as5047_write_data(int devidx, uint16_t addr, uint16_t data) {
    if (parity_bit_calculate(addr & 0x3fff)) {
        addr |= 0x8000;
    }

    CS_LOW(devidx);
    spi_rw_byte(devidx, addr);
    CS_HIGH(devidx);

    if (parity_bit_calculate(data & 0x3fff)) {
        data |= 0x8000;
    }

    uint16_t feedback;
    CS_LOW(devidx);
    feedback = spi_rw_byte(devidx, data);

    CS_HIGH(devidx);
    return feedback;
}

/**
 * @brief 读取数据
 *
 * @param devidx 设备ID
 * @param addr 地址
 * @return 数据
 */
uint16_t as5047_read_data(int devidx, uint16_t addr) {
    uint16_t data;
    if (parity_bit_calculate(addr) == 0) {
        addr |= 0x8000;
    }

    addr |= AS5047_COMMAND_READ;
    spi_rw_byte(devidx, addr);
    data = spi_rw_byte(devidx, AS5047_NOP_VOL_REG_ADD);
    data &= 0x3fff;

    data &= 0x3fff;
    return data;
}

/**
 * @brief AS5047初始化
 * 
 * @param hspi spi句柄
 * @param port 片选GPIO
 * @param pin 片选引脚
 * 
 * @note 请先初始化对应硬件spi，时钟模式为"SPI_CLK_MODE1"，CSP中设置片选引脚"Not Used"，
 *        data_size设置为"SPI_DATASIZE_16BIT"，MSB。
 */
void as5047_init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *port, uint16_t pin) {

    if (as5047_num >= AS5047_MAX_NUM) {
        return;
    }

    devices[as5047_num].hspi = hspi;
    devices[as5047_num].port = port;
    devices[as5047_num].pin = pin;

    GPIO_InitTypeDef gpio_init_struct;
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init_struct.Pull = GPIO_PULLUP;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;

    /* 初始化片选引脚 */
    gpio_init_struct.Pin = devices[as5047_num].pin;
    GPIO_CLK_ENABLE(devices[as5047_num].port);
    HAL_GPIO_Init(devices[as5047_num].port, &gpio_init_struct);

    CS_HIGH(as5047_num);
    as5047_reset(as5047_num);
    ++as5047_num;
}

/**
 * @brief 重置编码器计数
 *
 * @param devidx 设备编号
 */
void as5047_reset(int devidx) { /* 获取初始角度 */
    g_as5047_position[devidx].init_position =
        as5047_read_data(devidx, AS5047_ANGLEUNC_VOL_REG_ADD);
    g_as5047_position[devidx].current_position =
        g_as5047_position[devidx].init_position;
    g_as5047_position[devidx].round_cnt = 0;
    g_as5047_position[devidx].total_position = 0;
}

/**
 * @brief 设置零点
 *
 * @param devidx 设备编号
 */
void as5047_set_zero(int devidx) {
    as5047_read_data(devidx, AS5047_DIAAGC_VOL_REG_ADD);
    uint16_t ANGLEUNC = as5047_read_data(devidx, AS5047_ANGLEUNC_VOL_REG_ADD);
    as5047_write_data(devidx, AS5047_ZPOSM_nVOL_REG_ADD,
                      (ANGLEUNC >> 6) & 0x00ff);
    as5047_write_data(devidx, AS5047_ZPOSL_nVOL_REG_ADD, ANGLEUNC & 0x003f);
}

/**
 * @brief 获取零点
 *
 * @param devidx 设备编号
 * @return 零点位置
 */
uint16_t as5047_get_zero(int devidx) {
    uint16_t ZPOSM = as5047_read_data(devidx, AS5047_ZPOSM_nVOL_REG_ADD);
    uint16_t ZPOSL = as5047_read_data(devidx, AS5047_ZPOSL_nVOL_REG_ADD);
    return (ZPOSM << 6) & (ZPOSL & 0x003f);
}

/**
 * @brief 获取当前的绝对位置
 *
 * @param devidx 设备编号
 * @return 当前的位置
 */
uint16_t as5047_get_absolute_position(int devidx) {
    g_as5047_position[devidx].current_position =
        as5047_read_data(devidx, AS5047_ANGLEUNC_VOL_REG_ADD);
    return g_as5047_position[devidx].current_position;
}

/**
 * @brief 获取从上电到现在的相对位置
 *
 * @param devidx 设备编号
 * @note 调用后会更新`g_as5047_position`变量
 */
void as5047_get_relative_position(int devidx) {
    g_as5047_position[devidx].last_position =
        g_as5047_position[devidx].current_position;

    as5047_get_absolute_position(devidx);

    if (g_as5047_position[devidx].current_position -
            g_as5047_position[devidx].last_position >
        (AS5047_CIRCLE_CNT >> 1)) {
        --g_as5047_position[devidx].round_cnt;
    } else if (g_as5047_position[devidx].current_position -
                   g_as5047_position[devidx].last_position <
               -(AS5047_CIRCLE_CNT >> 1)) {
        ++g_as5047_position[devidx].round_cnt;
    }

    g_as5047_position[devidx].total_position =
        g_as5047_position[devidx].round_cnt * AS5047_CIRCLE_CNT +
        g_as5047_position[devidx].current_position -
        g_as5047_position[devidx].init_position;
}
