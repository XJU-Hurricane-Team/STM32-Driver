# 大疆 M3508, M2006, GM6020 电机驱动

https://www.robomaster.com/zh-CN/products/components/general/GM6020

https://www.robomaster.com/zh-CN/products/components/general/M3508

https://www.robomaster.com/zh-CN/products/components/general/M2006

# 依赖

- `can`
- `can_list`

# 使用

1. 将头文件复制到 `User/Bsp/Inc/` 中，源文件复制到 `User/Bsp/Src` 中
2. 将 `dji_bldc_motor.c` 添加到工程的 `Bsp` 分组中
3. 在 `bsp.h` 中包含 `dji_bldc_motor.h`
4. 如果只使用特定型号的电机，可以在 `dji_bldc_motor.h` 中通过 `DJI_MOTOR_USE_M3508_2006` 和 `DJI_MOTOR_USE_GM6020` 宏来选择是否使用这些型号

# API

## 结构体对象

`dji_motor_handle_t` 包括电机的各种参数，如速度、电流、角度等重要参数。因此使用之前需要定义电机的句柄以接收这些参数。具体包含以下参数，根据需要使用。
```
/**
 * @brief 电机参数
 * @note 只启用部分型号时，对应型号的字段会被宏裁掉。
 */
typedef struct {
    /* 3508/2006 参数 */
    float real_current;    /*!< 转矩电流 Iq（A），3508: ±20 A，2006: ±10 A */

    /* GM6020 参数 */
    int16_t torque_current; /*!< 实际转矩电流（原始值，±16384 对应 ±3 A） */

    /* 共用参数 */
    uint8_t hall; /*!< 保留字段，原 DATA[6]，目前不赋值 */

    bool zero_from_first_frame; /*!< 是否把上电首帧位置当作 0 位 */
    bool got_offset;       /*!< 首帧角度与计圈状态已初始化 */
    uint16_t offset_angle; /*!< 零位对应的编码器计数，范围 0 ~ 8191 */

    uint16_t last_angle; /*!< 上一次回调的转子单圈计数 */
    uint16_t angle;      /*!< 当前回调的转子单圈编码器计数，0 ~ 8191 */
    int32_t total_angle; /*!< 相对零位的转子累计编码器计数 */
    int32_t round_cnt;   /*!< 转子圈数计数 */
    float rotor_degree;  /*!< 输出轴角度
                              3508/2006：相对首帧回调读到数据的累计角，已除减速比；
                              6020：显式 offset 时为相对该零位的单圈角 [0, 360)，
                              省略 offset 时把上电首帧获取的数据当作 0 位并累计 */

    int16_t set_value; /*!< 设置的值（电流或电压），应用层使用 */
    int16_t speed_rpm; /*!< 转子转速（RPM）；3508/2006 需除减速比得输出轴转速 */

    uint8_t temperature;    /*!< 温度（℃），3508/6020 有效，2006 无反馈 */
    dji_can_id_t motor_id;  /*!< 电机 ID */
    dji_motor_model_t motor_model; /*!< 电机型号 */
    can_selected_t can_select;     /*!< 选择 CAN 通信 */
} dji_motor_handle_t;
```

建议将 `set_value` 作为 PID 计算结果接收参数。

## 函数方法

- `dji_motor_init` 初始化电机，需要指定句柄、型号、ID (`dji_can_id_t` 枚举)、CAN1 或者 CAN2
  - 三种型号均可传四个参数，默认把上电首帧位置当作 0 位。6020 上电即有单圈绝对位置，省略 offset 只是把这个首帧读数定义为 0；3508/2006 的编码器在转子侧，输出多圈位置需要这个首帧参考。
  - 第五个参数为 `int32_t offset`，单位为转子编码器计数（一圈 8192），零位设在“编码器原点 + offset”处，即反馈角度减去 offset。传 `0` 使用编码器原点；支持正负值，按单圈归一化。这是软件角度偏移，不会向电机写入或保存原点。
  - 省略第五个参数时由宏补入专用标记 `DJI_OFFSET_AUTO`（`INT32_MIN`，不作为普通偏移值），再调用固定五参数函数 `dji_motor_init_impl`，不使用运行时可变参数。
- `dji_motor_deinit` 反初始化电机
- `dji_motor_set_current` M3508/2006 设置电流
  - `can_select` CAN1 或者 CAN2
  - `can_identify` 控制标识符，`DJI_MOTOR_GROUP1` 或者 `DJI_MOTOR_GROUP2`
- `dji_gm6020_voltage_control` GM6020 电机电压控制
  - `can_select` CAN1 或者 CAN2
  - `can_identify` 控制标识符，`DJI_GM6020_VOLTAGE_GROUP1` 或者 `DJI_GM6020_VOLTAGE_GROUP2`
- `dji_gm6020_current_control` GM6020 电机电流控制
  - `can_select` CAN1 或者 CAN2
  - `can_identify` 控制标识符，`DJI_GM6020_CURRENT_GROUP1` 或者 `DJI_GM6020_CURRENT_GROUP2`

由于大疆的电机的 CAN 报文中没有针对单个电机设置电流，因此不提供单电机控制。可以自行编写。

# 示例

GM6020 初始化示例（CAN 接收表已建立）：

```c
dji_motor_handle_t gm6020 = {0};
dji_motor_init(&gm6020, DJI_GM6020, CAN_GM6020_ID1,
               can1_selected, 2048); /* 编码器 2048 处为零位 */
```

上述 GM6020 在编码器值为 2048 时反馈 0°，4096 时反馈 90°，1024 时反馈 315°。若需要把上电首帧位置当作 0 位并累计角度，省略最后一个参数即可；显式传 `0` 与省略参数含义不同（6020 上电本身知道单圈绝对位置，省略 offset 只是软件上把首帧读数定义为 0）。

这里使用 ARM DSP 库的 pid。

```
int main(void) {
    bsp_init();

    /* 声明大疆电机参数句柄 */
    dji_motor_handle_t m3508 = {0};

    /* 定义 pid 结构体 */
    arm_pid_instance_f32 speed_pid = {
        .Kp = 1.5f,
        .Ki = 0.1f,
        .Kd = 0.0f,
    };

    arm_pid_init_f32(&speed_pid, 1);

    /* 电机初始化，M3508 电机，ID 为 1, 使用 CAN1 通信 */
    dji_motor_init(&m3508, DJI_M3508, CAN_Motor1_ID, can1_selected);

    /* 目标速度 */
    float target_speed = 200.0f;

    while (1) {
        /* ARM DSP 库中的 PID 计算函数。输入为差值，即期望值 - 测量值 */
        m3508.set_value =
            (int16_t)arm_pid_f32(&speed_pid, (target_speed - m3508.speed_rpm));

        dji_motor_set_current(can1_selected, DJI_MOTOR_GROUP1,
                              m3508.set_value, 0, 0, 0);
        HAL_Delay(100);
    }
}
```
