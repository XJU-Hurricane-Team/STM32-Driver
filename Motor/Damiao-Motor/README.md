# Damiao 电机驱动

BSP 层达妙电机驱动，包含 MIT / 位速 / 速度 / 力位混控四种控制模式，以及 0x7FF 帧的寄存器读写和参数持久化。

## 文件

| 文件 | 内容 |
|------|------|
| `damiao.h` | `dm_handle_t` 结构、模式/型号/错误/寄存器枚举、API 原型 |
| `damiao.c` | 协议常量、`can_callback` 分发器、控制函数、寄存器读/写/存参 |

## 寄存器读写协议

发送帧 ID 固定 0x7FF（标准帧），读、写、存参都用它。应答从电调的 `MST_ID` 回来，落进 `can_callback`，按 D[2] 的功能码分流。

### 写寄存器：0x55

```
TX  0x7FF  DLC=8  [ID_L, ID_H&0x07, 0x55, RID, B4, B5, B6, B7]
                                                └──uint32 LE──┘
RX  MST_ID DLC=8  [ID_L, ID_H&0x07, 0x55, RID, B4, B5, B6, B7]
```

写 RAM，不写 Flash。掉电后回到上位机最后保存到 Flash 的值，所以临时改错不会留下永久影响。

### 读寄存器：0x33

```
TX  0x7FF  DLC=4  [ID_L, ID_H&0x07, 0x33, RID]
RX  MST_ID DLC=8  [ID_L, ID_H&0x07, 0x33, RID, B4, B5, B6, B7]
                                                └──uint32 LE──┘
```

### 存参数到 Flash：0xAA + 0x01

```
TX  0x7FF  DLC=4  [ID_L, ID_H&0x07, 0xAA, 0x01]
RX  MST_ID DLC=4  [ID_L, ID_H&0x07, 0x33, RID]      ← 中间状态
RX  MST_ID DLC=4  [ID_L, ID_H&0x07, 0xAA, 0x01]     ← 完成
```

把当前 RAM 里的全部参数一次性写入电调 Flash。调用前先失能；单次擦写最长 30ms，Flash 寿命约 1 万次。

### 字节序

寄存器值是 `uint32` 小端：

```c
uint32_t value = (uint32_t)D[4]
               | ((uint32_t)D[5] << 8)
               | ((uint32_t)D[6] << 16)
               | ((uint32_t)D[7] << 24);
```

## CTRL_MODE 寄存器（RID = 0x0A）

模式编码与 `dm_mode_t` 一一对齐：

| 编码 | `dm_mode_t` | 命令帧 ID 偏移 |
|------|-------------|----------------|
| 1 | `DM_MODE_MIT` | `device_id + 0x000` |
| 2 | `DM_MODE_POS_SPEED` | `device_id + 0x100` |
| 3 | `DM_MODE_SPEED` | `device_id + 0x200` |
| 4 | `DM_MODE_PVT` | `device_id + 0x300` |

## 寄存器一览表

代码里在 `damiao.h` 中定义为 `dm_reg_addr_t`（RW）和 `dm_reg_ro_addr_t`（RO）两个枚举，下面的表是手册的镜像，方便直接查地址、类型、范围。

### 可读写寄存器（RW）

| 地址 | 名称 | 类型 | 范围 | 描述 | 枚举 |
|------|------|------|------|------|------|
| 0x00 | UV_Value | float  | (10.0, fmax]  | 低压保护值 | `DM_REG_UV_VALUE` |
| 0x01 | KT_Value | float  | [0.0, fmax]   | 扭矩系数 | `DM_REG_KT_VALUE` |
| 0x02 | OT_Value | float  | [80.0, 200)   | 过温保护值 | `DM_REG_OT_VALUE` |
| 0x03 | OC_Value | float  | (0.0, 1.0)    | 过流保护值 | `DM_REG_OC_VALUE` |
| 0x04 | ACC      | float  | (0.0, fmax)   | 加速度 | `DM_REG_ACC` |
| 0x05 | DEC      | float  | [-fmax, 0.0)  | 减速度 | `DM_REG_DEC` |
| 0x06 | MAX_SPD  | float  | (0.0, fmax]   | 最大速度 | `DM_REG_MAX_SPD` |
| 0x07 | MST_ID   | uint32 | [0, 0x7FF]    | 反馈 ID | `DM_REG_MST_ID` |
| 0x08 | ESC_ID   | uint32 | [0, 0x7FF]    | 接收 ID | `DM_REG_ESC_ID` |
| 0x09 | TIMEOUT  | uint32 | [0, 2³²−1]    | 超时警报时间 | `DM_REG_TIMEOUT` |
| 0x0A | CTRL_MODE | uint32 | [1, 4]       | 控制模式（见上表） | `DM_REG_CTRL_MODE` |
| 0x15 | PMAX     | float  | (0.0, fmax]   | 位置映射范围 | `DM_REG_PMAX` |
| 0x16 | VMAX     | float  | (0.0, fmax]   | 速度映射范围 | `DM_REG_VMAX` |
| 0x17 | TMAX     | float  | (0.0, fmax]   | 扭矩映射范围 | `DM_REG_TMAX` |
| 0x18 | I_BW     | float  | [100.0, 1e4]  | 电流环控制带宽 | `DM_REG_I_BW` |
| 0x19 | KP_ASR   | float  | [0.0, fmax]   | 速度环 Kp | `DM_REG_KP_ASR` |
| 0x1A | KI_ASR   | float  | [0.0, fmax]   | 速度环 Ki | `DM_REG_KI_ASR` |
| 0x1B | KP_APR   | float  | [0.0, fmax]   | 位置环 Kp | `DM_REG_KP_APR` |
| 0x1C | KI_APR   | float  | [0.0, fmax]   | 位置环 Ki | `DM_REG_KI_APR` |
| 0x1D | OV_Value | float  | TBD           | 过压保护值 | `DM_REG_OV_VALUE` |
| 0x1E | GREF     | float  | (0.0, 1.0]    | 齿轮力矩效率 | `DM_REG_GREF` |
| 0x1F | Deta     | float  | [1.0, 30.0]   | 速度环阻尼系数 | `DM_REG_DETA` |
| 0x20 | V_BW     | float  | (0.0, 500.0)  | 速度环滤波带宽 | `DM_REG_V_BW` |
| 0x21 | IQ_c1    | float  | [100.0, 1e4]  | 电流环增强系数 | `DM_REG_IQ_C1` |
| 0x22 | VL_c1    | float  | (0.0, 1e4]    | 速度环增强系数 | `DM_REG_VL_C1` |
| 0x23 | can_br   | uint32 | [0, 4]        | CAN 波特率代码 | `DM_REG_CAN_BR` |

### 只读寄存器（RO）

电机模型参数、版本号、校准量、实时状态。`dm_read_register` 可读，写 0x55 帧到这些地址电调会拒收。

| 地址 | 名称 | 类型 | 描述 | 枚举 |
|------|------|------|------|------|
| 0x0B | Damp     | float  | 电机粘滞系数 | `DM_REG_RO_DAMP` |
| 0x0C | Inertia  | float  | 电机转动惯量 | `DM_REG_RO_INERTIA` |
| 0x0D | hw_ver   | uint32 | 硬件版本（保留） | `DM_REG_RO_HW_VER` |
| 0x0E | sw_ver   | uint32 | 软件版本号 | `DM_REG_RO_SW_VER` |
| 0x0F | SN       | uint32 | 序列号（保留） | `DM_REG_RO_SN` |
| 0x10 | NPP      | uint32 | 极对数 | `DM_REG_RO_NPP` |
| 0x11 | Rs       | float  | 相电阻 | `DM_REG_RO_RS` |
| 0x12 | Ls       | float  | 相电感 | `DM_REG_RO_LS` |
| 0x13 | Flux     | float  | 磁链值 | `DM_REG_RO_FLUX` |
| 0x14 | Gr       | float  | 齿轮减速比 | `DM_REG_RO_GR` |
| 0x24 | sub_ver  | uint32 | 子版本号 | `DM_REG_RO_SUB_VER` |
| 0x25 | Boot_ver | uint32 | Boot 版本号 | `DM_REG_RO_BOOT_VER` |
| 0x37 | dir      | float  | 方向 | `DM_REG_RO_DIR` |
| 0x38 | m_off    | float  | 电机侧角度偏移 | `DM_REG_RO_M_OFF` |
| 0x3B | Imax     | float  | 驱动板最大电流 | `DM_REG_RO_IMAX` |
| 0x3C | VBus     | float  | 电源电压 | `DM_REG_RO_VBUS` |
| 0x3D | Tpcb     | float  | 驱动板温度 | `DM_REG_RO_TPCB` |
| 0x3E | Tmtr     | float  | 电机温度 | `DM_REG_RO_TMTR` |
| 0x3F | Iu_off   | float  | U 相电流偏置 | `DM_REG_RO_IU_OFF` |
| 0x40 | Iv_off   | float  | V 相电流偏置 | `DM_REG_RO_IV_OFF` |
| 0x41 | Iw_off   | float  | W 相电流偏置 | `DM_REG_RO_IW_OFF` |
| 0x50 | p_m      | float  | 电机当前位置 | `DM_REG_RO_P_M` |
| 0x51 | xout     | float  | 输出轴位置 | `DM_REG_RO_XOUT` |

> `fmax` 指 float 上限（约 3.4×10³⁸）。实际可用范围以达妙官方手册为准。float 寄存器在 CAN 上以 IEEE 754 bit pattern 传输，本驱动收到时按 `uint32` 拼好，调用方用 `union { uint32_t u; float f; }` 或 `memcpy` 转 float。

## 公共 API

### 控制类

| 函数 | 说明 |
|------|------|
| `dm_motor_init / deinit` | 注册到 `can_list` 反馈节点 |
| `dm_motor_enable / disable` | 发使能/失能帧 |
| `dm_clear_error` | 清错误 |
| `dm_save_zero` | 保存当前位置为零点 |
| `dm_mit_ctrl` | MIT 模式（位置 + 速度 + kp + kd + 力矩前馈）|
| `dm_pos_speed_ctrl` | 位置速度模式 |
| `dm_speed_ctrl` | 速度模式 |
| `dm_pvt_ctrl` | 力位混控模式（speed: 0~100，i_des: 0~1）|

### 寄存器类

| 函数 | 说明 |
|------|------|
| `dm_set_control_mode(motor, mode)` | 在线切换控制模式（写 CTRL_MODE）。内部调 `dm_write_register`，写完同步更新 `motor->mode`。`motor->mode == mode` 时返回 `2` 跳过下发，避免清零电调前馈/PID |
| `dm_write_register(motor, reg_addr, value)` | 通用写寄存器（0x55），写 RAM 不写 Flash |
| `dm_read_register(motor, reg_addr)` | 异步读寄存器（0x33），应答经 `can_callback` 写回 `motor->reg_read_*` |
| `dm_save_param(motor)` | 把当前 RAM 全部参数持久化到 Flash（0xAA+0x01）。调用前必须先失能；单次最长 30ms，Flash 寿命约 1 万次 |

读写函数都不阻塞，应答由 `can_callback` 异步写回这三个字段：

```c
volatile uint8_t  reg_read_addr;    /* 应答里的 RID（存参完成时是 0x01）*/
volatile uint32_t reg_read_value;   /* 应答里的 uint32（LE 已解出）；存参为 0 */
volatile uint8_t  reg_read_pending; /* 1=请求已发但应答未到 */
```

调用方等几毫秒读字段，或者轮询 `reg_read_pending`。读/写/存共用这三个字段，混用前确认上一笔已完成或值已取走。

## 安全约束

### 写寄存器之前必须先失能

电调一旦收到 0x55 写帧，会把位置/速度/扭矩前馈、kp、kd 全部清零。在转动或带载状态下切换会出事：

- 带载切换：重力跌落（升降臂、关节最严重）
- 高速切换：0 速命令突变，反电动势冲击直接过流跳闸
- kp/kd 清零后还在发 MIT 帧：等于纯前馈，位置环没刚性

调用顺序：

```c
dm_motor_disable(&m);
vTaskDelay(2);                              /* 让 CAN 把帧发出去 */
dm_set_control_mode(&m, DM_MODE_POS_SPEED);
vTaskDelay(2);
dm_motor_enable(&m);                        /* 重建闭环 */
```

### 读寄存器没副作用

只读不写，闭环运行时也能调，不会动任何命令或 PID。

## 内部实现

### 单回调分发

`dm_motor_init` 在 `can_list` 上为 `master_id` 注册一个 `can_callback`。这个回调要兼顾两种帧：周期上报的状态反馈（D[0] 是 ID+err，D[1..7] 是位速力温的位拼装），以及寄存器/存参应答（D[2] 是 0x33/0x55/0xAA 功能码）。

分流靠 `reg_read_pending`——只有 pending 时才把 D[2] 当功能码看，否则按状态反馈拆。这样状态帧里的 D[2]（即 position 低 8 位）就算碰巧落在 `{0x33, 0x55, 0xAA}` 上也不会误判。

```
master_id 帧 → can_callback
                ├── pending == 1 且 D[2] ∈ {0x33, 0x55, 0xAA}
                │     → 写 reg_read_*, 清 pending, return
                └── 否则 → 按状态反馈解析位置/速度/扭矩/温度
```

存参会回两帧（中间 `0x33+RID`、最终 `0xAA+0x01`），两帧都会清零 pending，调用方等够 50ms 就行。

### 同条 CAN 多电机

每个电机注册各自的 `master_id` 节点，回调里 `node_obj == &motor`，没有共享分发表。前提是每个 motor 的 master_id 不重叠（DM 协议本身就这样要求）。

## 典型用法

切换模式之后读回验证：

```c
dm_motor_disable(&m);
vTaskDelay(2);
dm_set_control_mode(&m, DM_MODE_POS_SPEED);
vTaskDelay(2);

dm_read_register(&m, 0x0A);                  /* 读 CTRL_MODE */
uint32_t t0 = HAL_GetTick();
while (m.reg_read_pending && HAL_GetTick() - t0 < 10) {
    vTaskDelay(1);
}
if (!m.reg_read_pending && m.reg_read_addr == 0x0A) {
    configASSERT(m.reg_read_value == 2);     /* 2 = POS_SPEED */
}

dm_motor_enable(&m);
```

## 已知边界

- 读/写/存参共用 `reg_read_*` 三个字段。同一 motor 不要在 `reg_read_pending` 没清零之前发下一笔，会覆盖前一笔的应答。
- 存参期间电调可能丢几帧状态反馈，调用前先失能、把控制环停掉。
- `dm_motor_deinit` 只摘 `can_list` 上的 `master_id` 节点。当前项目所有电机生命周期跟系统一样长，不会调 deinit，所以没有悬挂引用。
- 当前实现不处理应答与下一笔请求的乱序，并发场景需要上层自己加互斥。task3 是串行轮询，没这个问题。
