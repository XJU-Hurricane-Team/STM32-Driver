# IO_Link_Master

基于 **STM32F103C8T6 + L6360 + SICK DT35** 的 IO-Link 主站测距方案：通过 IO-Link 接口周期读取 DT35 激光距离传感器的距离值，经串口（USART2）以二进制大端格式上报给主板。

- 芯片：STM32F103C8T6（Keil MDK-ARM 工程：`MDK-ARM/IO_Link_Master.uvprojx`）
- 传感器：SICK DT35（IO-Link 从站，COM2 = 38.4 kbit/s）
- 收发器：ST L6360（IO-Link 主站 PHY，寄存器经 I2C1 配置，C/Q 数据经 USART1 收发）

---

## 1. 数据链路：DT35 距离值 → STM32 → 串口发送

```
┌──────┐  IO-Link C/Q线   ┌───────┐  USART1  ┌─────────────┐
│ DT35 │ ◄──────────────► │ L6360 │ ◄──────► │ STM32F103C8T6│
│(从站) │  M-sequence      │(PHY)  │  字节收发 │  数据链路层   │
└──────┘                  └───────┘          └──────┬──────┘
                                                    │ USART2
                                              ┌─────▼──────┐
                                              │  主板     │
                                              └────────────┘
```

### 完整数据流

| 步骤 | 位置 | 说明 |
|------|------|------|
| ① 测距 | DT35 | 激光测距，距离值作为 IO-Link **Process Data（2 字节）**周期输出 |
| ② 物理层收发 | L6360 | C/Q 线 UART 数据经 L6360 与 MCU USART1 透明传输（`L6360_Send_IN_CQ_Data` / `L6360_Recv_OUT_CQ_Data`） |
| ③ 数据链路层 | `iolink_master_dl.c` | 组帧 M-sequence **Type 2_2**（MC+CKT+OD+PD0+PD1+CKS，含校验） |
| ④ 轮询读取 | `main.c` while 循环 | `dl_read_type_2_2(&m_sequence2, &processData, NULL, Idle_1)`，将 2 字节 PD 拼成 `uint16_t processData` |
| ⑤ 周期上报 | TIM3 更新中断（500 Hz） | `HAL_TIM_PeriodElapsedCallback` 中调用 `uart_send_message(&huart2, processData)` |
| ⑥ 串口发送 | `main.c` `uart_send_message` | 16 位距离值**大端拆为 2 字节**，`HAL_UART_Transmit` 发送，详见下文协议 |

### 上报串口协议（USART2）

```c
// main.c
buffer[0] = (uint8_t)(data >> 8);   // 高字节先发（大端）
buffer[1] = (uint8_t)(data & 0xFF); // 低字节
HAL_UART_Transmit(huart, buffer, 2, 100);
```

- 每帧 **2 字节**，无帧头/校验，**高字节在前**
- **发送频率 500 Hz**：由 TIM3 更新中断驱动，调用一次 `uart_send_message` 发送一帧
- 帧间隔靠 UART 空闲线划分（接收端可用空闲中断分帧）
- 距离分辨率由 `DT35_Set_Resolution()` 决定：0.1 mm 或 1 mm，数值单位即为计数

---

## 2. LED 灯状态说明

LED 为 L6360 板载 LED1/LED2，经 I2C 寄存器（`L6360_LED_x_MSB/LSB`）驱动，模式见 `LED_Mode_t`：

| LED1 | LED2 | 含义 |
|------|------|------|
| 常亮 | 慢闪 | **正常运行**：唤醒成功、通信已建立，暂无有效距离数据（`processData == 0`） |
| 常亮 | 常亮 | **数据有效**：收到非零距离值（`processData != 0`） |
| 快闪 | 快闪 | **通信故障**：错误计数超限（≥10），正在执行重连 `dl_reconnection_handler()` |

状态迁移逻辑位于 `main.c`：

- 唤醒成功后 `L6360_Set_Led(1, LED_ON)`（通信建立）
- `processData` 从 0 → 非 0：LED2 慢闪转常亮；非 0 → 0：转回慢闪
- `HAL_TIM_PeriodElapsedCallback`（TIM2）中错误计数 ≥10 置 `reWakeUpFlag`，双灯快闪并重连

---

## 3. 相关 API

### 3.1 IO-Link 数据链路层 `iolink_master_dl.h`

| 函数 | 说明 |
|------|------|
| `Dl_Status_t dl_wake_up(void)` | 向从站发送唤醒脉冲（COMX 速率），建立通信 |
| `Dl_Status_t dl_read_type_2_2(M_Sequence_Type_2_2 *m, uint16_t *data_out, OD_t *od_out, uint8_t addr)` | 读 M-seq Type 2_2，**获取 2 字节过程数据（距离值）** |
| `Dl_Status_t dl_write_type_2_2(M_Sequence_Type_2_2 *m, OD_t data_in, uint16_t *data_out, uint8_t addr)` | 写 OD 并读取过程数据 |
| `Dl_Status_t dl_write_type_0(M_Sequence_Type_0 *m, uint8_t data_in, uint8_t addr)` | M-seq Type 0 单字节写（如 0x99 起始命令） |
| `Dl_Status_t dl_write_ISDU_uint8(uint8_t index, uint8_t data)` | **ISDU 参数写**（DT35 全部配置经此下发） |
| `Dl_Status_t dl_reconnection_handler(void)` | 通信故障后重连 |

### 3.2 DT35 配置 `DT35.h`（均基于 ISDU 写）

| 函数 | ISDU Index | 参数 |
|------|-----------|------|
| `DT35_Set_Resolution(mode)` | 0x69 | 分辨率：0=0.1 mm（量程 6.5 m）/ 1=1 mm（量程 65 m）/ 2=10 mm |
| `DT35_Set_Laser(mode)` | 0x44 | 激光：0=关 / 1=开 |
| `DT35_Set_Response_Time(mode)` | 0x67 | 响应时间：0=Expert ~ 5=Super-fast |
| `DT35_Set_Output_Rate(mode)` | 0x40 | 输出速率：2^n × 2 ms，0~8 |
| `DT35_Set_Filter_Depth(mode)` | 0x43 | 滤波深度：1/2/4/8/16 |
| `DT35_Reset_Parameter(void)` | 0x02 | 恢复出厂参数（写 130） |

### 3.3 L6360 收发器驱动 `L6360.h`

| 函数 | 说明 |
|------|------|
| `void L6360_Init(void)` | 初始化（EN_CQ / L+ / 复位时序，I2C 配置寄存器） |
| `uint8_t L6360_Set_Buadrate(IO_Link_Buadrate_t b)` | IO-Link 速率：COM1=4800 / COM2=38400 / **COM3=230400** / COMX=唤醒 |
| `HAL_StatusTypeDef L6360_Send_IN_CQ_Data(uint8_t *pdata, uint8_t len)` | 经 USART1 向 C/Q 线发数据 |
| `HAL_StatusTypeDef L6360_Recv_OUT_CQ_Data(uint8_t *pdata, uint8_t len)` | 从 C/Q 线收数据 |
| `uint8_t L6360_Write_Register(uint8_t reg, uint8_t data)` | 写 L6360 寄存器（I2C1，地址 0x60） |
| `uint8_t L6360_Read_Register(uint8_t reg)` / `uint16_t L6360_Current_Read_Register(void)` | 读寄存器 / 读取当前状态（错误诊断） |
| `uint8_t L6360_Set_Led(uint8_t id, LED_Mode_t mode)` | 控制 LED1/LED2：ON / OFF / SLOW_BLINK / FAST_BLINK |
| `uint8_t L6360_Set_EN_CQ / Set_EN_LPlus / Set_RST(uint8_t)`、`void L6360_Reset(void)` | C/Q 通道、L+ 电源、复位控制 |

### 3.4 串口发送与扩展 `main.c` 

| 函数 | 说明 |
|------|------|
| `HAL_StatusTypeDef uart_send_message(UART_HandleTypeDef *huart, uint16_t data)` | **距离值上报**：16 bit 大端拆 2 字节发送（main.c） |

---

## 4. 错误监测与自动重连

1. L6360 故障引脚（IRQ）接 EXTI15_10，中断中置 `l6360_irq_pending`，主循环读取 L6360 状态寄存器诊断
2. `dl_read_type_2_2` 的错误次数累计到 `errorCounter`
3. TIM2 周期检查：`errorCounter ≥ 10` → 置 `reWakeUpFlag`，双灯快闪
4. 主循环执行 `dl_reconnection_handler()` 重连，成功后恢复 LED 状态

---

## 5. 目录结构

```
├── Core/                     # CubeMX 生成：main.c / 中断 / HAL MSP
├── Drivers/
│   ├── BSP/                  # L6360、can_bsp
│   ├── STM32F1xx_HAL_Driver/ # HAL 库
│   └── CMSIS/
├── MDK-ARM/
│   ├── IO_Link_Master.uvprojx  # Keil 工程
│   ├── io-link-master-dl/      # IO-Link 数据链路层
│   └── DT35/                   # DT35 ISDU 配置驱动
└── IO_Link_Master.ioc      # CubeMX 配置
```
