/**
 * @file    iic.c
 * @author  Deadline039
 * @brief   I2C初始化以及驱动代码
 * @version 0.2
 * @date    2023-11-05
 */

#include "./core/core_delay.h"
#include "iic.h"

/**
 * @brief IIC设备初始化
 *
 */
void iic_init(void) {
    GPIO_InitTypeDef gpio_initure = {0};

    IIC_SCL_GPIO_ENABLE();
    IIC_SDA_GPIO_ENABLE();

    gpio_initure.Pin = IIC_SCL_GPIO_PIN;
    gpio_initure.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_initure.Pull = GPIO_PULLUP;
    gpio_initure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio_initure);

    gpio_initure.Pin = IIC_SDA_GPIO_PIN;
    gpio_initure.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_initure.Pull = GPIO_PULLUP;
    gpio_initure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio_initure);

    iic_stop();
}

/**
 * @brief 产生IIC开始信号
 *
 */
void iic_start(void) {
    IIC_SDA(1);
    IIC_SCL(1);
    delay_us(WAIT_TIME);
    IIC_SDA(0);
    IIC_SCL(0);
    delay_us(WAIT_TIME);
}

/**
 * @brief 产生IIC停止信号
 *
 */
void iic_stop(void) {
    IIC_SCL(1);
    IIC_SDA(0);
    delay_us(WAIT_TIME);
    IIC_SCL(1);
}

/**
 * @brief IIC发送ACK信号
 *
 */
void iic_ack(void) {
    IIC_SDA(0); /* SCL 0 -> 1 时 SDA = 0,表示应答 */
    delay_us(WAIT_TIME);
    IIC_SCL(1); /* 产生一个时钟 */
    delay_us(WAIT_TIME);
    IIC_SCL(0);
    delay_us(WAIT_TIME);
    IIC_SDA(1); /* 主机释放SDA线 */
    delay_us(WAIT_TIME);
}

/**
 * @brief IIC不发送ACK信号
 *
 */
void iic_nack(void) {
    IIC_SDA(1); /* SCL 0 -> 1  时 SDA = 1,表示不应答 */
    delay_us(WAIT_TIME);
    IIC_SCL(1); /* 产生一个时钟 */
    delay_us(WAIT_TIME);
    IIC_SCL(0);
    delay_us(WAIT_TIME);
}

/**
 * @brief IIC等待ACK信号
 *
 */
void iic_wait_ack(void) {
    IIC_SCL(1);
    delay_us(WAIT_TIME);
    IIC_SCL(0);
    delay_us(WAIT_TIME);
}

/**
 * @brief IIC发送一字节数据
 *
 * @param data 要发送的数据
 */
void iic_send_byte(uint8_t data) {
    uint8_t m;
    for (uint8_t i = 0; i < 8; i++) {
        m = data;
        m &= 0x80;
        if (m == 0x80) {
            IIC_SDA(1);
        } else {
            IIC_SDA(0);
        }
        data <<= 1;
        delay_us(WAIT_TIME);
        IIC_SCL(1);
        delay_us(WAIT_TIME);
        IIC_SCL(0);
    }
}

/**
 * @brief IIC读取一字节数据
 *
 * @param ack 是否发送ACK
 *  @arg 1-发送ack, 0-不发送ack
 * @return uint8_t
 */
uint8_t iic_read_byte(uint8_t ack) {
    uint8_t receive = 0;
    for (uint8_t i; i < 8; i++) {
        /* 接收1个字节数据 */
        receive <<= 1; /* 高位先输出,所以先收到的数据位要左移 */
        IIC_SCL(1);
        delay_us(WAIT_TIME);

        if (IIC_READ_SDA) {
            receive++;
        }

        IIC_SCL(0);
        delay_us(WAIT_TIME);
    }

    if (!ack) {
        iic_nack(); /* 发送nACK */
    } else {
        iic_ack(); /* 发送ACK */
    }
    return receive;
}

/**
 * @brief IIC发送命令
 *
 * @param cmd 命令内容
 */
void iic_write_commandd(uint8_t cmd) {
    iic_start();
    iic_send_byte(0x78);
    iic_wait_ack();
    iic_send_byte(0x00);
    iic_wait_ack();
    iic_send_byte(cmd);
    iic_wait_ack();
    iic_stop();
}

/**
 * @brief IIC发送数据
 *
 * @param data 数据内容
 */
void iic_write_data(uint8_t data) {
    iic_start();
    iic_send_byte(0x78);
    iic_wait_ack();
    iic_send_byte(0x40);
    iic_wait_ack();
    iic_send_byte(data);
    iic_wait_ack();
    iic_stop();
}
