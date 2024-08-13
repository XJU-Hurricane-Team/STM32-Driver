# 大疆 M3508 M2006, GM6020电机驱动

1. 将头文件复制到`User/Bsp/Inc/`中，源文件复制到`User/Bsp/Src`中
2. 将`dji_bldc_motor.c`添加到工程的`Bsp`分组中
3. 在`bsp.h`中包含`dji_bldc_motor.h`
4. 由于大疆电机反馈频率较高，链表查找比较耗时。因此采取直接在CAN接收中断回调函数中调用`dji_motor_recv_callback`。CAN接收中断一般为`HAL_CAN_RxFifoxMsgPendingCallback`。参考以下代码实现：

```
extern void dji_motor_can_recv_callback(can_select_t can_selected,
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

`dji_motor_handle_t`包括电机的各种参数，如速度、电流、角度等重要参数。因此使用之前需要定义电机的句柄以接收这些参数。

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

由于大疆的电机的CAN报文中没有针对单个电机设置电流，因此不提供单电机控制。可以自行编写