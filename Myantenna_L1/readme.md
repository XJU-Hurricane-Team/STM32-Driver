# Myantenna_L1 激光模块
- 文档:http://www.imyantenna.com/list-8.html
- 激光模块上有微信二维码，可以扫码联系技术人员.
- ASCII协议建议在上位机上使用,不建议在代码中进行解析.
- 实验室大部分激光模式为[ascii通信]+[上电连续快速测量]+[串口波特率115200],如果需要设定参数,需先**使用iHALT停止测量再进行参数设定**.
- **仅建议连接TTL时使用ASCII模式**
- 如果需要高精度的参数测量,请勿使用[上电连续快速测量].
- 注:
    1. 使用ASCII发送命令后,反馈帧同为ASCII数据.
    2. 可以通过上位机设置协议格式,需要重新上电更新协议.
    3. `myantenna_laser_init`中的填入的协议格式须确认与激光设置的相同.
    4. 使用modbus协议时,务必人为(在comtool中)设定地址和协议格式.
    5. modbus协议解析：目前仅支持解析寄存器地址0x000F下的数据,即测量距离.其它暂时不支持.

# BUG
1. ASCII会出现数据处理未完成而丢包的现象.(解决办法:采用DMA进行数据的处理).
2. ASCII单次收发可能收不到,部分不会触发串口中断,巨抽象.

# 已测试
- MODBUS
    - [上电连续快速测量] + [数据解析]
    - [单次测量]+[数据解析]
- HEX 
    - [上电连续快速测量] + [数据解析]
    - [上电连续快速测量] -> [停止测量] -> [单次测量] -> [连续快速测量] + [数据解析]
    - [单次测量] + [数据解析]
- ASCII
    - [上电连续快速测量] + [数据解析]大部分正常,但不建议使用
    - [上电连续快速测量] -> [停止测量]
    - [单次测量] + [数据解析]**有BUG**不要用,建议使用**HEX协议格式**或**MODBUS**

# 依赖
- uart

# API
- `myantenna_laser_handle_t` 激光句柄
- `myantenna_laser_init` 初始化激光

# 使用方法
1. 使用`myantenna_laser_handle_t`创建激光句柄.
2. 使用`myantenna_laser_init`初始化激光句柄，输入通信协议+地址+串口句柄.
3. 创建串口中断回调函数和串口接收处理数组,使用`myantenna_laser_handle_t`中`myantenna_data_parse`处理激光数据.
4. 注册串口中断回调函数
## 注意事项
- 串口中断接收数组**rx_buf的长度**,**请在`myantenna_uart_rx_buf_length`中按照对应协议格式进行选择**.
- `myantenna_laser_init`中adress仅与modbus解析相关.
## Example
```
#include "./Myantenna_L1/myantenna_L1.h"

myantenna_laser_handle_t laser_test;                               /* 激光句柄 */
uint8_t laser_test_rx_buf[MYANTENNA_MODBUS_UART_RX_BUF_LEN];       /* 串口rx数组 */


void callback(UART_HandleTypeDef *huart, uint16_t Pos)；

/**
 *@brief 激光初始化
 *
 */
void laser_init(void) {
    /* 初始化laser_test激光句柄 */
     myantenna_laser_init(&laser_test, MYANTENNA_MODBUS, 0x01, &usart2_handle);

    /* 开启串口空闲中断回调 */
    HAL_UARTEx_ReceiveToIdle_IT(laser_test.uart_handle, laser_test_rx_buf,
                                sizeof(laser_test_rx_buf));
    /* 注册串口中断回调函数 */
    HAL_UART_RegisterRxEventCallback(laser_test.uart_handle, callback);
}

/**
 *@brief 串口中断回调函数
 *
 */
void callback(UART_HandleTypeDef *huart, uint16_t Pos) {
    UNUSED(Pos);
    if (huart == laser_test.uart_handle) {
        HAL_UARTEx_ReceiveToIdle_IT(laser_test.uart_handle, laser_test_rx_buf,
                                    sizeof(laser_test_rx_buf));
        /* 数据解析 */
        laser_test.myantenna_data_parse(&laser_test, laser_test_rx_buf,
                                    sizeof(laser_test_rx_buf));
        memset(laser_test_rx_buf, 0, sizeof(laser_test_rx_buf));
    }
}
```
 