数据手册：https://ww1.microchip.com/downloads/cn/DeviceDoc/21801D_CN.pdf

源驱动：[[eziya/STM32_SPI_MCP2515: STM32 + MCP2515 CAN Example](https://github.com/eziya/STM32_SPI_MCP2515)](https://github.com/eziya/STM32_SPI_MCP2515)

本目录为 **BSP（Board Support Package）层** 的 MCP2515 SPI CAN 驱动实现，包含两部分：

1. **`SPICAN/` 目录（MCP2515 + CAN 接口封装）**
2. **`spican_list/` 目录（基于哈希表的消息回调分发）**

---

##  目录结构

```
Drivers_Achieve/
  ├─ SPICAN/
  │    ├─ MCP2515.h
  │    ├─ MCP2515.c
  │    ├─ CANSPI.h
  │    └─ CANSPI.c
  └─ spican_list/
  │    ├─ spican_list.h
  │    └─ spican_list.c
  │ 
  └─ README.md    
```

> **提示**：`SPICAN` 与 `spican_list` 属于同一功能域（MCP2515 CAN 驱动），但是逻辑上分为：
>
> - `SPICAN`：底层 SPI 读写 + CAN 报文发送/接收。
> - `spican_list`：上层消息分发、回调管理，支持 FreeRTOS 异步处理。

---

##  核心模块说明

### 1) `SPICAN` 目录（MCP2515 SPI 驱动 + CAN 报文封装）

#### **`MCP2515.h / MCP2515.c`**
- 定义 MCP2515 SPI 指令与寄存器地址（如 `MCP2515_READ`, `MCP2515_CANCTRL` 等）。
- 实现 SPI 读写、寄存器访问、缓冲区读写：
  - `MCP2515_ReadByte_Ext`：读取一字节
  - `MCP2515_WriteByte_Ext`：写入一字节
  - `MCP2515_ReadRxSequence_Ext`：读取一系列寄存器的值
  - `MCP2515_LoadTxSequence_Ext`：往一系列寄存器写入值
- 支持 MCP2515 模式切换：
  - `MCP2515_SetConfigMode_Ext`：配置模式
  - `MCP2515_SetNormalMode_Ext`：正常模式
  - `MCP2515_SetSleepMode_Ext`：睡眠模式
- 支持多设备（`MCP2515_DevId` + `MCP2515_DEVx_ENABLE`），以及中断配置（`MCP2515_INT_Init`）。

#### **`CANSPI.h / CANSPI.c`**
- 封装上层 CAN 发送/接收
  - `CANSPI_Initialize_Ext`：初始化 MCP2515 (滤波、波特率、工作模式)
  - `CANSPI_Transmit_Ext`：发送 CAN 报文（自动选择 TX 缓冲区）
  - `CANSPI_Receive_Ext`：接收 CAN 报文，解析标准/扩展 ID
- 提供状态查询
  - `CANSPI_messagesInBuffer_Ext`
  - `CANSPI_isBussOff`, `CANSPI_isRxErrorPassive`, `CANSPI_isTxErrorPassive`
- 处理标准/扩展 ID 编码/解码（`convertCANid2Reg`, `convertReg2StandardCANid`, `convertReg2ExtendedCANid`）

---

## 2) `spican_list` 目录（消息回调分发）

该模块基于 **哈希表 + 回调** 机制组织 CAN 报文处理，适用于复杂系统中按 ID 调度处理逻辑。

### 结构说明
- `spican_list_add_can`：创建 CAN 实例表（支持 STD/EXT 分离）。
- `spican_list_add_new_node`：注册节点 (`id`, `id_mask`, `callback`)。
- `mcp2515_process_msg`：从 MCP2515 读取消息并根据 ID 查找回调执行。

### FreeRTOS 支持（默认开启）
- 宏 `SPICAN_LIST_USE_RTOS`=1 时：
  - 中断处理（`HAL_GPIO_EXTI_Callback`）将事件推入队列。
  - 任务 `spican_list_polling_task` 在后台消费队列并调用 `mcp2515_process_msg`。

---

##  关键配置点

### 1. MCP2515 设备使能与引脚映射（位于 `MCP2515.h`）
通过宏配置启用/禁用设备：

```c
#define MCP2515_DEV1_ENABLE 1
#define MCP2515_DEV2_ENABLE 0
#define MCP2515_DEV3_ENABLE 0
```

对应模块会使用如下宏映射 SPI/CS/INT 等资源：
- `MCP2515_DEVx_SPI_HANDLE`、`MCP2515_DEVx_CS_PIN`、`MCP2515_DEVx_INT_PIN` 等

> 若要支持多个 MCP2515，请确保在硬件上有独立 CS + INT 线，并在 `CSP_Config.h` 中提供相应 HAL 句柄。

### 2. CAN 波特率与滤波（位于 `CANSPI.c`）
- 默认波特率：**1 Mbps**（通过写入 `CNF1/CNF2/CNF3` 实现）。
- 默认滤波：**接受所有报文**（`RXF*` 和 `RXM*` 全清零）。

若要按 ID 过滤，请在 `CANSPI_Initialize_Ext` 调用 `MCP2515_WriteByteSequence_Ext` 时修改 `RXF*`/`RXM*` 的值。

---

##  使用示例

###  基本初始化 + 发送/接收（FreeRTOS）

```c
#include "./SPICAN/CANSPI.h"
#include "./Damiao-Motor_spican/damiao_spican.h"
#include "./spican_list/spican_list.h"
	uint8_t canintf1_reg;
    uint8_t canintf2_reg;
    uint8_t can_tec;
    uint8_t can_rec;
    uint8_t can_elfg;
void task1(void *pvParameters) {
    UNUSED(pvParameters);
    
	/* 注册达妙电机句柄(节点)并填入SPICAN哈希表 */
    dm_motor_init_mcp2515(&g_Damiao_motor_handle, 0x11, 0x01, DM_MODE_MIT, DM_G6220, 12.5, 45, 10, spican1_selected);
    /* 达妙电机使能 */
    dm_motor_enable_mcp2515(&g_Damiao_motor_handle);
    uint8_t cnt = 0;
    uint8_t dir = 1;

    while (1) {        
        dir > 0 ? cnt++ : cnt-- ;
        if(cnt>200) {
            dir = 0;
        } else if(cnt == 0) {
            dir = 1;
        }

        dm_mit_ctrl_mcp2515(&g_Damiao_motor_handle, cnt*0.01, 0, 8.8, 0.1, 0.0f);	
        
        canintf1_reg = MCP2515_ReadByte_Ext(MCP2515_DEV_1, MCP2515_CANINTF);		/* 读取CANINTF寄存器，获取中断标志位 */
        canintf2_reg = MCP2515_ReadByte_Ext(MCP2515_DEV_1, MCP2515_CANINTE);		/* 读取CANINTE寄存器，获取MCP2515使能中断类型 */

        can_tec = MCP2515_ReadByte_Ext(MCP2515_DEV_1, MCP2515_TEC);		/* 读取TEC寄存器，获取CAN报文发送错误计数 */
        can_rec = MCP2515_ReadByte_Ext(MCP2515_DEV_1, MCP2515_REC);		/* 读取REC寄存器，获取CAN报文接收错误计数 */
        can_elfg = MCP2515_ReadByte_Ext(MCP2515_DEV_1, MCP2515_EFLG);	/* 读取ELFG寄存器，根据寄存器标志位判断错误类型 */

        vTaskDelay(5);
    }
}
```

---

##  常见注意点

- MCP2515要求SPI最大时钟频率10Mhz，SPI配置为模式0(CPOL = 0，CPHA = 0)

- 为满足1MBaud通信，MCP2515须接入5V电源

- 防止INT引脚因外界电平不定而导致误进中断，INT引脚须配置上拉电阻

- 波特率配置：根据CNF1、CNF2、CNF3的寄存器值配置，详情见数据手册P42

  CNF1的bit5_0：波特率预分频，CNF2的bit5_3：**PHSEG1**，bit2_0：**PRSEG**，CNF3的bit2_0：**PHSEG2**

  各段实际分配的TQ是**寄存器值+1**，例如CNF2的bit5_3 = 001，实际PHSEG1 = 2Tq

  MCP2515模块的波特率与MCU的CAN计算公式不同，MCP2515的TQ = 2 x (BRP + 1)/FOSC，FOSC是晶振频率，原装是8Mhz，无法满足CAN2.0B通信协议，须联系电路组更换为16Mhz

  所以波特率 = FOSC/**(** 2 x (BRP + 1) x (1+ PRSEG  + PHSEG1 + PHSEG2)**)**	(1MBaud  = 16M/**(**2 x (0 + 1) x (1+ 2 + 3 + 2)**)**

- 如果开启INT引脚，RX0/RX1接收到CAN报文后，一定要清除`CANINTF.RX0IF`和`CANINTF.RX1IF`中断标志位，否则MCP2515的INT引脚一直输出低电平，MCU对应INT引脚无法检测到下降沿信号，进而倒置接收错误、bus-off（TEC >= 255）  （bus-off指该CAN外设完全停止通信，不发也不收，有两种方式可以恢复总线，但在比赛场景肯定不能依赖总线恢复）

- **由于大疆DJI电机的CAN报文回馈频率高达1KHz，而MCP2515模块比较鸡肋，INT引脚无法响应如此快的回馈频率，会导致buf-off，所以用DJI电机不能开启INT引脚，只能用阻塞式收发、非常占用CPU，所以不推荐用MCP2515模块与DJI电机通信**

  但达妙电机的报文回馈方式是MCU发一帧报文，达妙电机才回馈一帧报文，INT引脚完全可以响应，所以对于MCP2515，达妙是理想的通信对象

- 若不使用 FreeRTOS，可将 `SPICAN_LIST_USE_RTOS` 置为 `0` 并自行编写中断处理逻辑

---

