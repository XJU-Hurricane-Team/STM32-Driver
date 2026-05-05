# 概述

`VL53L1`：具有先进的多区域和多目标检测功能的飞行时间`（ToF）`测距传感器。传统的红外传感器是通过测量反射光的**强度**来判断距离的，因此如果遇到黑色物体（吸光）或表面粗糙的物体，测出的距离就会不准。 而 `VL53L1` 使用的是真正的 `ToF` 技术：它测量的是光子从发射到被物体反射并回到传感器所花费的**绝对时间**。因为光速是恒定的，所以它**不受目标物体颜色、反射率或表面材质的影响**。

产品说明：[VL53L1 | Product - STMicroelectronics](https://www.st.com/en/imaging-and-photonics-solutions/vl53l1.html)

本驱动基于 ST 官方 `VL53L1X API` 核心库移植，适配了自定义的软件 IIC 驱动程序。

# 硬件连接

| 模块引脚  | STM32 开发板引脚 | 说明                                       |
| --------- | ---------------- | ------------------------------------------ |
| VCC/AVDD  | 3.3V             | 建议接 3.3V，确保电平匹配                  |
| GND       | GND              | 电源地                                     |
| SCL       | PB10（示例）     | IIC 时钟线                                 |
| SDA       | PB11（示例）     | IIC 数据线                                 |
| XSHUT     | 悬空/3.3V        | 使能引脚。低电平关断，悬空默认开启         |
| GPIO1/INT | 悬空             | 中断输出。本驱动采用软件轮询模式，无需接线 |

# 使用方法

## 加入驱动

1. 建议将驱动中的所有.c文件加入到 `Middlewares` 层。

2. 在`include.h` 中引入

```c
#include "vl53l1_api.h"
```

3. 依赖 `iic.c` 和 `iic.h` 

## 软件 IIC 延时

为了保证通信稳定，`iic.h` 中的 `WAIT_TIME` 建议设置为 `2` 或以上，以确保 IIC 频率在传感器支持的 400kHz 以内。

```c
#define WAIT_TIME    2 
```

## FreeRTOS 堆栈管理 

- **VL53L1_Dev_t 结构体**: 体积较大（约 2-4KB），严禁直接作为任务局部变量定义在栈上。必须使用 `static` 关键字或全局定义。
- **任务栈空间**: 创建任务时，建议分配至少 `512` words (2048 字节) 的栈空间。
- **总堆大小**: 若任务创建失败，请调大 `FreeRTOSConfig.h` 中的 `configTOTAL_HEAP_SIZE`。

## 例程

```c
void vl53l1_task(void *pvParameters) {
    UNUSED(pvParameters);

    // 1. 定义设备句柄和数据结构体
    static VL53L1_Dev_t dev_struct;
    VL53L1_DEV dev = &dev_struct;
    dev->I2cDevAddr = 0x52; // VL53L1X 默认 I2C 地址

    VL53L1_RangingMeasurementData_t RangingData;
    uint8_t dataReady = 0;
    int8_t status = 0;

    printf("VL53L1X Initializing...\r\n");

    // 3. 等待设备启动完成 (加入 vTaskDelay 避免阻塞)
    while (VL53L1_WaitDeviceBooted(dev) != VL53L1_ERROR_NONE) {
        vTaskDelay(pdMS_TO_TICKS(10)); 
    }

    // 4. 初始化传感器
    status = VL53L1_DataInit(dev);
    status = VL53L1_StaticInit(dev);

    if (status == VL53L1_ERROR_NONE) {
        // 5. 配置测距参数
        VL53L1_SetDistanceMode(dev, VL53L1_DISTANCEMODE_SHORT);            // 设置为短距离模式
        VL53L1_SetMeasurementTimingBudgetMicroSeconds(dev, 50000);         // 测量时间预算：50ms
        VL53L1_SetInterMeasurementPeriodMilliSeconds(dev, 20);             // 测量间隔：20ms
        
        // 6. 启动连续测量
        VL53L1_StartMeasurement(dev);
        printf("VL53L1X Start Measurement Success!\r\n");
    } else {
        printf("VL53L1X Init Failed! Error code: %d\r\n", status);
    }

    // 7. 进入主循环读取数据
    while (1) {
        dataReady = 0;
        
        // 查询数据是否准备好
        while (dataReady == 0) {
            VL53L1_GetMeasurementDataReady(dev, &dataReady);
            if (dataReady == 0) {
                // 数据未准备好时，延时 5ms 释放 CPU 给其他任务 (如 task1)
                vTaskDelay(pdMS_TO_TICKS(5)); 
            }
        }
        
        // 读取测距数据
        status = VL53L1_GetRangingMeasurementData(dev, &RangingData);
        
        if (status == VL53L1_ERROR_NONE) {
            // RangeStatus == 0 代表数据有效
            if (RangingData.RangeStatus == 0) { 
                printf("Distance: %d mm\r\n", RangingData.RangeMilliMeter);
            } else {
                // 如果测不到物体、超出范围等，会返回非 0 的状态码
                printf("Range Status Error: %d\r\n", RangingData.RangeStatus);
            }
        }

        // 清除中断状态，触发下一次测量
        VL53L1_ClearInterruptAndStartMeasurement(dev);

        // 测距频率控制，比如每 50ms 打印一次
        vTaskDelay(pdMS_TO_TICKS(50)); 
    }
}
```

## 视野角修改

上述例程的感光区域是默认的，传感器内部有一个 **16x16 的 SPAD（单光子雪崩二极管）接收阵列**。我们可以通过指定这个 16x16 矩阵上的**左上角 和 右下角** 坐标，来框选出想要生效的区域，即修改了视野角，代码示例如下：

```c
// 定义 ROI 结构体
VL53L1_UserRoi_t roiConfig;

// 配置左上角和右下角坐标 (0~15)
roiConfig.TopLeftX = 6;   
roiConfig.TopLeftY = 9;   
roiConfig.BotRightX = 9;  
roiConfig.BotRightY = 6;  

// 将 ROI 配置下发给传感器
VL53L1_SetUserROI(dev, &roiConfig);
```

**注：坐标系规则如下**

**X 轴（列）**：从左到右是 **0 到 15**。所以 `TopLeftX` 必须 **小于或等于** `BotRightX`。  

**Y 轴（行）**：从下到上是 **0 到 15**。这和我们平时屏幕坐标系（Y朝下）相反！所以 `TopLeftY` 必须 **大于或等于** `BotRightY`。  

**最小尺寸限制**：你框选的矩形区域，最小必须是 **4x4**。也就是说，X 的差值和 Y 的差值都必须至少为 3（例如 9 - 6 = 3，包含 6,7,8,9 共 4 个点）。

# 错误码查询

### 测距结果状态码 (`RangingData.RangeStatus`)

这是反映**光学环境、物理距离、目标材质**的状态码。只要 API 没报错，就会返回这个码。

| 状态码 | 英文定义       | 含义与原因                                                   | 解决办法                                                     |
| ------ | -------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| 0      | Range Valid    | 测量有效。信号完美，距离准确。                               | 保持现状。                                                   |
| 1      | Sigma Fail     | 标准差失败（数据波动太大）。通常是因为环境光太强（比如在太阳底下），或者目标太远，导致传感器接收到的光子杂乱无章，无法给出一个确定的距离。 | 避免强光直射，或缩短测量距离，增大测量时间预算 (TimingBudget)。 |
| 2      | Signal Fail    | 信号太弱。传感器发射的红外光被目标吸收了（如黑色、毛绒物体），或者目标太远，导致回波信号低于设定的阈值。 | 更换浅色/反光目标测试；增大测量时间预算 (TimingBudget)。     |
| 3      | Min Range Fail | 最小距离失败。目标离传感器太近了（通常小于 2~4 厘米），导致内部串扰或溢出，无法计算准确距离。 | 将目标移远一点。                                             |
| 4      | Phase Fail     | 相位失败。通常发生在环境光剧烈变化，或者目标物体移动速度极快时，导致内部的相位计算（ToF 核心算法）出错。 | 稳定环境光，确保目标不要高频抖动。                           |
| 5      | Hardware Fail  | 硬件失败。传感器内部的 VCSEL（激光发射器）损坏，或者电源供电极度不稳定。 | 检查 3.3V 供电是否带得动，如果供电正常仍报错，可能是芯片已损坏。 |
| 7      | Wrap Target    | 相位缠绕（多路径干扰）。极其常见于隔着玻璃测距时。玻璃反射了一部分光，真实的物体又反射了一部分光，导致传感器收到两个回波，算法“懵”了。 | 去掉玻璃，或者对玻璃进行串扰校准（Xtalk Calibration）。      |

---

### API 执行状态码 (`status`)

这是反映**代码逻辑、IIC 通信、寄存器读写**的状态码。

| 状态码 | 英文定义                      | 含义与原因                                                   | 解决办法                                                     |
| ------ | ----------------------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| 0      | VL53L1_ERROR_NONE             | 执行成功。                                                   | 继续执行下一步。                                             |
| -7     | VL53L1_ERROR_TIME_OUT         | 超时。通常是 IIC 通信失败，单片机发出的指令传感器没收到，或者轮询状态时死等不到传感器拉高标志位。 | 检查 IIC 的 SCL 和 SDA 连线；检查是否漏写了 iic_init()；检查 IIC 时钟速率是否过快。 |
| -15    | VL53L1_ERROR_DIVISION_BY_ZERO | 除零错误。通常是因为软件 IIC 的底层读写函数（如 iic_read_byte）写错了，导致读回来的校准数据全是 0，底层在做除法时直接崩溃。 | 检查软件 IIC 时序（比如读数据前是否释放了 SDA），检查变量是否漏了初始化（如上文提到的那个 Bug）。 |
| -20    | VL53L1_ERROR_INVALID_PARAMS   | 参数无效。你给 API 传入了不被支持的参数。比如把测距时间预算设置成了 1 毫秒（下限通常是 20 毫秒）。 | 查阅 API 手册，确保传入的值在合理范围内。                    |
| -41    | (平台层错误)                  | IIC 通信错误。有些平台的底层 IIC 读写函数如果返回非 0 值，API 会透传这个错误。 | 拿逻辑分析仪或示波器抓一下 IIC 波形，看看是不是没有收到 ACK 应答。 |