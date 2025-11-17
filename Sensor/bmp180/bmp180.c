/**
 * @file    bmp180.c
 * @author  Deadline039
 * @brief   BMP180气压传感器
 * @version 1.0
 * @date    2024-05-05
 */

#include "bmp180.h"
#include "math.h"

/* BMP180获取到的数据 */
bmp_data_t g_bmp_data;

/**
 * @brief The raw data of the bmp180.
 */
static struct {
    short AC1;
    short AC2;
    short AC3;
    unsigned short AC4;
    unsigned short AC5;
    unsigned short AC6;
    short B1;
    short B2;
    short MB;
    short MC;
    short MD;

    long UT;
    long UP;
    long X1;
    long X2;
    long X3;
    long B3;
    unsigned long B4;
    long B5;
    long B6;
    long B7;
} bmp180_raw_data;

/**
 * @brief Write one byte from the bmp180 register.
 *
 * @param iic_number IIC device number
 * @param addr Address
 * @param data Data
 */
void bmp180_write_one_byte(uint8_t addr, uint8_t data) {
    HAL_I2C_Mem_Write(&I2C_HANDLE_1, 0x77 << 1, addr, I2C_MEMADD_SIZE_8BIT, &data,
                      1, 1000);
}

/**
 * @brief Read one byte from the bmp180 register.
 *
 * @param iic_number IIC device number
 * @param addr Address
 * @return Data
 */
uint8_t bmp180_read_one_byte(uint8_t addr) {
    uint8_t data = 0;

    HAL_I2C_Mem_Read(&I2C_HANDLE_1, 0x77 << 1, addr, I2C_MEMADD_SIZE_8BIT, &data,
                     1, 1000);
    return data;
}

/**
 * @brief Read two byte from the bmp180 register.
 *
 * @param iic_number IIC device number
 * @param addr Address
 * @return Data
 */
uint16_t bmp180_read_two_byte(uint8_t addr) {
    uint16_t data;
    uint8_t tmp[2];

    // msb = iic_read_byte(iic_number, 1);
    // lsb = iic_read_byte(iic_number, 0);
    // data = (msb << 8) | lsb;

    HAL_I2C_Mem_Read(&I2C_HANDLE_1, 0x77 << 1, addr, I2C_MEMADD_SIZE_8BIT, tmp,
                     2, 1000);
    data = (tmp[0] << 8) | tmp[1];
    return data;
}

/**
 * @brief Read Calibration data from the E2PROM of the BMP180.
 *
 * @param iic_number IIC device number
 */
void bmp180_get_cal_param(void) {
    bmp180_raw_data.AC1 = bmp180_read_two_byte(0xAA);
    bmp180_raw_data.AC2 = bmp180_read_two_byte(0xAC);
    bmp180_raw_data.AC3 = bmp180_read_two_byte(0xAE);
    bmp180_raw_data.AC4 = bmp180_read_two_byte(0xB0);
    bmp180_raw_data.AC5 = bmp180_read_two_byte(0xB2);
    bmp180_raw_data.AC6 = bmp180_read_two_byte(0xB4);
    bmp180_raw_data.B1 = bmp180_read_two_byte(0xB6);
    bmp180_raw_data.B2 = bmp180_read_two_byte(0xB8);
    bmp180_raw_data.MB = bmp180_read_two_byte(0xBA);
    bmp180_raw_data.MC = bmp180_read_two_byte(0xBC);
    bmp180_raw_data.MD = bmp180_read_two_byte(0xBE);
}

/**
 * @brief Read uncompensated temperature value.
 *
 * @param iic_number IIC device number
 * @return Uncompensated temperature value.
 */
long bmp180_get_ut(void) {
    long temp = 0;
    bmp180_write_one_byte(0xF4, 0x2E);

    HAL_Delay(5);
    temp = (long)bmp180_read_two_byte(0xF6);
    return temp;
}

/**
 * @brief Read uncompensated pressure value.
 *
 * @param iic_number IIC device number
 * @return Uncompensated pressure value.
 */
long bmp180_get_up(void) {
    long pressure = 0;

    bmp180_write_one_byte(0xF4, 0x34);
    HAL_Delay(5);

    pressure = (long)bmp180_read_two_byte(0xF6);
    // pressure = pressure + bmp180_read_one_byte(iic_number, 0xf8);
    pressure &= 0x0000FFFF;

    return pressure;
}

/**
 * @brief Calculate true temperature.
 *
 */
void bmp180_get_temperature(void) {
    long temperature;

    bmp180_raw_data.X1 =
        ((bmp180_raw_data.UT - bmp180_raw_data.AC6) * bmp180_raw_data.AC5) >>
        15;
    bmp180_raw_data.X2 = (((long)bmp180_raw_data.MC) << 11) /
                         (bmp180_raw_data.X1 + bmp180_raw_data.MD);
    bmp180_raw_data.B5 = bmp180_raw_data.X1 + bmp180_raw_data.X2;
    temperature = (bmp180_raw_data.B5 + 8) >> 4;

    g_bmp_data.tempature = temperature / 10.0f;
}

/**
 * @brief Calculate true pressure.
 *
 */
void bmp180_get_pressure(void) {
    bmp180_raw_data.B6 = bmp180_raw_data.B5 - 4000;
    bmp180_raw_data.X1 = ((long)bmp180_raw_data.B2 *
                          (bmp180_raw_data.B6 * bmp180_raw_data.B6 >> 12)) >>
                         11;
    bmp180_raw_data.X2 = ((long)bmp180_raw_data.AC2) * bmp180_raw_data.B6 >> 11;
    bmp180_raw_data.X3 = bmp180_raw_data.X1 + bmp180_raw_data.X2;

    bmp180_raw_data.B3 =
        ((((long)bmp180_raw_data.AC1) * 4 + bmp180_raw_data.X3) + 2) / 4;
    bmp180_raw_data.X1 = ((long)bmp180_raw_data.AC3) * bmp180_raw_data.B6 >> 13;
    bmp180_raw_data.X2 = (((long)bmp180_raw_data.B1) *
                          (bmp180_raw_data.B6 * bmp180_raw_data.B6 >> 12)) >>
                         16;
    bmp180_raw_data.X3 = ((bmp180_raw_data.X1 + bmp180_raw_data.X2) + 2) >> 2;
    bmp180_raw_data.B4 = ((long)bmp180_raw_data.AC4) *
                             (unsigned long)(bmp180_raw_data.X3 + 32768) >>
                         15;
    bmp180_raw_data.B7 =
        ((unsigned long)bmp180_raw_data.UP - bmp180_raw_data.B3) * 50000;

    if (bmp180_raw_data.B7 < 0x80000000) {
        g_bmp_data.pressure = (bmp180_raw_data.B7 * 2) / bmp180_raw_data.B4;
    } else {
        g_bmp_data.pressure = (bmp180_raw_data.B7 / bmp180_raw_data.B4) * 2;
    }

    bmp180_raw_data.X1 =
        (g_bmp_data.pressure >> 8) * (g_bmp_data.pressure >> 8);
    bmp180_raw_data.X1 = (((long)bmp180_raw_data.X1) * 3038) >> 16;
    bmp180_raw_data.X2 = (-7357 * g_bmp_data.pressure) >> 16;

    g_bmp_data.pressure =
        g_bmp_data.pressure +
        ((bmp180_raw_data.X1 + bmp180_raw_data.X2 + 3791) >> 4);
}

/**
 * @brief BMP180初始化
 *
 */
void bmp180_init(void) {
    bmp180_get_cal_param();
    g_bmp_data.pressure = 0;
    g_bmp_data.altitude = 0;
    g_bmp_data.tempature = 0.0f;
}

/**
 * @brief 用获取的参数对温度和大气压进行修正, 并计算海拔
 *
 * @note 获取到数据后会更新`g_bmp_data`变量, 读此变量内容即可.
 */
void bmp180_get_data(void) {
    bmp180_raw_data.UT = bmp180_get_ut(); /* 第一次数据修正 */
    bmp180_raw_data.UT = bmp180_get_ut(); /* 进行第二次数据修正 */
    bmp180_raw_data.UP = bmp180_get_up();
    bmp180_get_temperature();
    bmp180_get_pressure();
    g_bmp_data.altitude =
        44330 * (1 - pow(((g_bmp_data.pressure) / 101325.0), (1.0 / 5.255)));
}