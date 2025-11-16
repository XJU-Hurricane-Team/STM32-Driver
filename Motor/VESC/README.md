# VESC电调驱动

https://vesc-project.com/sites/default/files/imce/u15301/VESC6_CAN_CommandsTelemetry.pdf

# 依赖

- `can`
- `can_list`
- `utils/buffer_append`

# 使用

1. 将头文件复制到`User/Bsp/Inc/`中，源文件复制到`User/Bsp/Src`中
2. 将`vesc_motor.c`添加到工程的`Bsp`分组中
3. 在`bsp.h`中包含`vesc_motor.h`

# API  


## 结构体对象

`vesc_motor_handle_t`是电机对象句柄，CAN接收回调后会更新参数。包含以下参数
```
/**
 * @brief VESC电机参数
 */
typedef struct {
    uint8_t vesc_id;         /*!< 电机 ID */
    can_selected_t can_select; /*!< 选择CAN1还是CAN2 */

    float input_voltage; /*!< 电机电压 */
    float duty;          /*!< MOSFET占空比 */
    float erpm;          /*!< 转速 */
    float rpm;           /*!< 转子实际转速 */

    float amp_hours;         /*!< 电流时间 */
    float amp_hours_charged; /*!< 电流充电时间 */

    float watt_hours;         /*!< 功率时间 */
    float watt_hours_charged; /*!< 功率充电时间 */

    float motor_current; /*!< 电机电流 */
    float total_current; /*!< 总电流 */

    float mosfet_temperature; /*!< MOSFET温度 */
    float motor_temperature;  /*!< 电机温度 */

    float pid_pos; /*!< 转子位置 */

    int32_t tachometer_value;     /*!< 转速表 */
    vesc_fault_code_t error_code; /*!< 错误码 */
} vesc_motor_handle_t;
```

## 函数方法

- `vesc_motor_init`初始化电机
  - `motor`电机句柄
  - `id`电机ID
  - `can_select`CAN1还是CAN2
- `vesc_motor_deinit`反初始化电机

初始化后使用下面的方法控制电机：

```
void vesc_motor_set_duty(vesc_motor_handle_t *motor, float duty);
void vesc_motor_set_current(vesc_motor_handle_t *motor, float current);
void vesc_motor_set_break_current(vesc_motor_handle_t *motor, float current);
void vesc_motor_set_erpm(vesc_motor_handle_t *motor, float erpm);
void vesc_motor_set_pos(vesc_motor_handle_t *motor, float pos);
void vesc_motor_set_relative_current(vesc_motor_handle_t *motor, float current);
void vesc_motor_set_relative_break_current(vesc_motor_handle_t *motor,
                                           float current);
void vesc_motor_set_current_limit(vesc_motor_handle_t *motor, float min_current,
                                  float max_current, bool store_to_rom);
```

代码注释和文档都有详尽的解释，这里就不多赘述。

# 示例

```
void vesc_demo(void) {
    vesc_motor_handle_t vesc_demo;
    vesc_motor_init(&vesc_demo, 1, can1_selected);

    /* 设置转速 */
    vesc_motor_set_erpm(&vesc_demo, 1000.0f);
    /* 设置角度 */
    vesc_motor_set_pos(&vesc_demo, 30.0f);
    /* 设置占空比 */
    vesc_motor_set_duty(&vesc_demo, 0.2f);
}
```