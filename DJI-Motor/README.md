# 大疆 M3508, M2006, GM6020电机驱动

https://www.robomaster.com/zh-CN/products/components/general/GM6020

https://www.robomaster.com/zh-CN/products/components/general/M3508

https://www.robomaster.com/zh-CN/products/components/general/M2006

# 依赖

- `can`

# 使用

1. 将头文件复制到`User/Bsp/Inc/`中，源文件复制到`User/Bsp/Src`中
2. 将`dji_bldc_motor.c`添加到工程的`Bsp`分组中
3. 在`bsp.h`中包含`dji_bldc_motor.h`
4. 由于大疆电机反馈频率较高，链表查找比较耗时。因此采取直接在CAN接收中断回调函数中调用`dji_motor_recv_callback`。CAN接收中断一般为`HAL_CAN_RxFifoxMsgPendingCallback`。
5. 如果只使用特定型号的电机，可以在`dji_bldc_motor.h`中通过`DJI_MOTOR_USE_M3508_2006`和`DJI_MOTOR_USE_GM6020`宏来选择是否使用这些型号
 
接收中断参考以下代码实现：

```
extern void dji_motor_can_recv_callback(can_select_t can_select,
                                        uint32_t can_id, uint8_t *recv_msg);
/**
 * @brief CAN RX FIFO0挂起中断回调
 *
 * @param hcan CAN句柄
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    ...
    if (hcan->Instance == CAN1) {
        if (can_rx_header.IDE == CAN_ID_STD &&
            (can_rx_header.StdId >= 0x201 && can_rx_header.StdId <= 0x20B)) {
            /* 3508, 2006, 6020电机回调 */
            dji_motor_can_recv_callback(can1_selected, can_rx_header.StdId,
                                        recv_msg);
        } else {
            /* 其他回调, 链表查找 */
            can_list_callback(can1_selected, &can_rx_header, recv_msg);
        }
    } else if (hcan->Instance == CAN2) {
        if (can_rx_header.IDE == CAN_ID_STD &&
            (can_rx_header.StdId >= 0x201 && can_rx_header.StdId <= 0x20B)) {
            dji_motor_can_recv_callback(can2_selected, can_rx_header.StdId,
                                        recv_msg);
        } else {
            /* 其他回调, 链表查找 */
            can_list_callback(can2_selected, &can_rx_header, recv_msg);
        }
    }
}

```

# API

## 结构体对象

`dji_motor_handle_t`包括电机的各种参数，如速度、电流、角度等重要参数。因此使用之前需要定义电机的句柄以接收这些参数。具体包含以下参数，根据需要使用。
```
/**
 * @brief 电机参数
 */
typedef struct {
    /* 3508/2006参数 */
    uint8_t hall; /*!< 可能是霍尔传感器值 */

    float real_current;    /*!< 实际电流 */
    int16_t given_current; /*!< 期望电流, M3508电机才会赋值 */

    int32_t total_angle; /*!< 上电以后为0点, 以此为基准的总角度 */

    uint16_t offset_angle; /*!< 上电后角度初始位置 */
    bool got_offset;       /*!< 上电以后获取一次角度偏移 */

    /* GM6020参数 */
    int16_t torque_current; /*!< 实际转矩电流 */
    uint8_t temperature;    /*!< 温度 */

    /* 共用参数 */
    dji_can_id_t motor_id; /*!< 电机ID */
    int16_t set_value;     /*!< 设置的值, 电压或电流值 */
    uint16_t last_angle;   /*!< 上次角度 */
    uint16_t angle;        /*!< 角度, 绝对角度, 一圈为8192 */
    int16_t speed_rpm;     /*!< 速度 */
    int32_t round_cnt;     /*!< 圈数计数 */

    float rotor_degree; /*!< 转子角度
                             对于3508与2006, 是轴的相对位置.
                              上电后为0度, 轴转一圈为360, 0(360)度附近不会跳变.
                              角度会累加, 已经除过减速比;
                             对于6020, 是绝对位置(0 ~ 360).
                              上电后不为0, 角度不会累加, 0(360)度附近会跳变. */

    dji_motor_model_t motor_model; /*!< 电机型号 */
} dji_motor_handle_t;
```

建议将`set_value`作为PID计算结果接收参数。

## 函数方法

- `dji_motor_init` 初始化电机，需要指定句柄、型号、ID(`dji_can_id_t`枚举)、CAN1或者CAN2
- `dji_motor_deinit`反初始化电机
- `dji_motor_set_current`M3508/2006设置电流
  - `can_select`CAN1或者CAN2
  - `can_identify`控制标识符，`DJI_MOTOR_GROUP1`或者`DJI_MOTOR_GROUP2`
- `dji_gm6020_set_voltage`GM6020电机电压控制
  - `can_select`CAN1或者CAN2
  - `can_identify`控制标识符，`DJI_GM6020_VOLTAGE_GROUP1`或者`DJI_GM6020_VOLTAGE_GROUP2`
- `dji_gm6020_set_current`GM6020电机电流控制
  - `can_select`CAN1或者CAN2
  - `can_identify`控制标识符，`DJI_GM6020_CURRENT_GROUP1`或者`DJI_GM6020_CURRENT_GROUP2`

由于大疆的电机的CAN报文中没有针对单个电机设置电流，因此不提供单电机控制。可以自行编写。

# 示例

这里使用ARM DSP库的pid。

```
int main(void) {
    bsp_init();

    /* 声明大疆电机参数句柄 */
    dji_motor_handle_t m3508;

    /* 定义pid结构体 */
    arm_pid_instance_f32 speed_pid = {
        .Kp = 1.5f,
        .Ki = 0.1f,
        .Kd = 0.0f,
    };

    arm_pid_init_f32(&speed_pid, 1);

    /* 电机初始化, M3508电机, ID为1, 使用CAN1通信 */
    dji_motor_init(&m3508, DJI_M3508, CAN_Motor1_ID, can1_selected);

    /* 目标速度 */
    float target_speed = 200.0f;

    while (1) {
        /* ARM DSP库中的PID计算函数. 输入为差值, 即期望值 - 测量值 */
        m3508.set_value =
            (int16_t)arm_pid_f32(&speed_pid, (target_speed - m3508.speed_rpm));

        dji_motor_set_current(can1_selected, DJI_MOTOR_GROUP1,
                              m3508.set_value, 0, 0, 0);
        HAL_Delay(100);
    }
}
```
