# CubeMars AK系列电机驱动

https://www.cubemars.com/goods-1151-AK80-8.html

https://www.cubemars.com/images/file/20240611/1718084209493165.pdf

**代码以及文档中的运控模式即MIT，伺服模式即Servo**

# 依赖

- `can`
- `can_list`
- `utils/buffer_append`

# 使用

1. 将头文件复制到`User/Bsp/Inc/`中，源文件复制到`User/Bsp/Src`中
2. 将`ak_motor.c`添加到工程的`Bsp`分组中
3. 在`bsp.h`中包含`ak_motor.h`

# API

## 结构体对象

`ak_motor_handle_t`是电机对象句柄，CAN接收回调后会更新。包含以下内容：
```
/**
 * @brief AK电机句柄
 */
typedef struct {
    can_selected_t can_select; /*!< 选择 CAN */
    uint32_t id;               /*!< CAN ID */

    ak_mode_t mode;              /*!< 电机模式 */
    ak_motor_model_t model;      /*!< 电机型号 */
    float posi;                  /*!< 电机位置 */
    float spd;                   /*!< 电机速度 */
    float current_troq;          /*!< 电机电流，运控模式为扭矩 */
    int8_t motor_temperature;    /*!< 电机温度 */
    ak_motor_error_t error_code; /*!< 电机错误码 */
} ak_motor_handle_t;
```
错误可以通过`ak_motor_error_t`枚举来进行相应的处理。

## 函数方法
- `ak_motor_init`初始化电机
  - `motor`电机句柄
  - `id`电机的ID
  - `model`电机型号
  - `mode` 电机模式
  - `can_select`选择CAN1或CAN2
- `ak_motor_deinit`反初始化电机

初始化电机后使用下面的方法控制电机：

### Servo模式
```
void ak_servo_set_duty(ak_motor_handle_t *motor, float duty);
void ak_servo_set_current(ak_motor_handle_t *motor, float current);
void ak_servo_set_cb(ak_motor_handle_t *motor, float current);
void ak_servo_set_rpm(ak_motor_handle_t *motor, float rpm);
void ak_servo_set_pos(ak_motor_handle_t *motor, float pos);
void ak_servo_set_origin(ak_motor_handle_t *motor,
                         ak_origin_mode_t set_origin_mode);
void ak_servo_set_pos_spd(ak_motor_handle_t *motor, float pos, float spd,
                          float rpa);

```

### MIT模式
```
void ak_mit_enter_motor(ak_motor_handle_t *motor);
void ak_mit_set_origin(ak_motor_handle_t *motor);
void ak_mit_send_data(ak_motor_handle_t *motor, float pos, float spd, float kp,
                      float kd, float torque);
void ak_mit_exit_motor(ak_motor_handle_t *motor);
```

代码注释和文档都有详尽的解释，这里就不多赘述。

# 示例

```
/**
 * @brief 伺服模式示例
 *
 */
void servo_demo(void) {
    ak_motor_handle_t ak80_demo;
    ak_motor_init(&ak80_demo, 104, AK80_8, can1_selected);

    /* 设置速度 */
    ak_servo_set_rpm(&ak80_demo, 1000.0f);
    /* 设置位置 */
    ak_servo_set_pos(&ak80_demo, 100.0f);
    /* 设置电流 */
    ak_servo_set_current(&ak80_demo, 1000.0);
}

/**
 * @brief 运控模式示例
 *
 */
void mit_demo(void) {
    ak_motor_handle_t ak80_demo;
    ak_motor_init(&ak80_demo, 104, AK80_8, can1_selected);

    /* 首先需要进入控制模式 */
    ak_mit_enter_motor(&ak80_demo);

    ak_mit_send_data(&ak80_demo, 100.0f, 200.0f, 10.0f, 0.1f, 5.0f);

    /* 退出控制 */
    ak_mit_exit_motor(&ak80_demo);
}
```