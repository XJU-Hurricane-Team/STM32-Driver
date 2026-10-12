# CAN 驱动

包含 CAN 底层初始化、收发消息等代码。其中接收使用链表，收到消息后根据 ID 在表中找到相应的回调函数并调用。

# 依赖

无

# 用法

1. 将该文件夹复制到 `/Drivers/bsp/` 下
2. 在 `bsp.h` 中使用相对路径包含：
   ``` C
   #include "can_list/can_list.h"
   ```
3. 使用`can_list_add_can`添加一个 CAN 外设
4. 使用`can_list_add_new_node`添加 CAN 外设对应的设备

# API

- `CAN_LIST_USE_FDCAN`宏用于确定是否使用 FDCAN，STM32 HAL 库的 bxCAN 与 FDCAN 互不兼容！但是本模块代码是两者都兼容的。当芯片外设为 FDCAN 下使用！
- `CAN_LIST_MAX_CAN_NUMBER`宏用于确定当前设备最大支持的 CAN 外设数量，防止缓冲区溢出
- `CAN_LIST_USE_RTOS`宏用于确定是否使用操作系统任务来处理 CAN 消息，当使用操作系统后会创建一个线程来处理收到的 CAN 消息以加快中断退出时间，**启用后需要注意 CAN 中断的优先级不能高于 FreeRTOS 可管理的优先级！** 启用后，使用`can_list_add_can`时，一定要在`vTaskStartScheduler()`后使用！
- `can_list_add_can` 添加一个 CAN：
  - `can_select`添加那一个 CAN
  - `std_len` 标准 ID 哈希表键值，根据 ID 合理设置以减少查表时间（设置为 1 退化为链表）。并非设备数量限制！
  - `ext_len` 扩展 ID 哈希表键值，根据 ID 合理设置以减少查表时间（设置为 1 退化为链表）。并非设备数量限制！
- `can_list_add_new_node` 添加新节点，`node_ptr` 可以为空指针，`callback` 不能为空！
  - `can_select` 使用那个 CAN 接收，`can1_selected` 或 `can2_selected`
  - `id` 设备反馈时的 ID
  - `id_mask` 设备反馈 ID 掩码
  - `node_ptr` 设备指针，当收到数据并找到相应 ID 的设备后会将这个指针作为参数传入 `callback` 函数
  - `callback` 收到数据后调用的函数
- `can_list_del_node_by_id` 通过 ID 删除设备
- `can_list_change_callback` 通过 ID 更改回调函数

# 示例

处理 CAN 回调消息，按照 ID 调用相应的回调函数。

对于大多数使用扩展帧的 CAN 设备，其扩展 ID 基本还会包含一些数据格式。因此我们必须对 ID 进行处理。

例如，假设一个设备的 CAN 反馈报文帧 ID 格式如下：

```
| bit [29:22] 错误码 | bit [21:14] 当前模式 | bit [13:11] 反馈数据内容标识 | bit [10:8] 保留 | bit [7:0] ID |
```

由于 `bit [29:8]` 包含数据，具体内容是不确定的。而 `bit [7:0]` 是实际 ID，那么我们通过按位与把 `bit [7:0]` 提取出来就可以了。那么我们掩码 `id_mask` 就填 `0xFF`。这样当 CAN 中断产生回调时，遍历并依次按照事先指定的掩码将 ID 按位与就可以找到我们想要的设备了。

回调函数必须是如下形式：

``` C
void (*can_callback)(void * /* node_obj */,
                     can_rx_header_t * /* can_rx_header */,
                     uint8_t * /* can_msg */);
```

例如：
``` C
void motor_callback (void *node_obj,
                    can_rx_header_t *can_rx_header,
                    uint8_t *can_msg) {
}
```

## 设备关系

```
+---------------+                         +---------------+                            
|               |       Feed back         |               |                            
|               | <---------------------- |               |                            
|               |     ID = Master ID      |               |                            
|               |                         |               |                            
|    Master     |                         |    Device     |                            
|               |                         |               |                            
|               |         Send            |               |                            
|               | ----------------------> |               |                            
|               |      ID = Dev ID        |               |                            
+---------------+                         +---------------+                   
```

Master 为主机，这里定义为我们这一端，Device 为设备，我们要控制的一端。

Device 反馈时的 ID 为 `Master ID`，向 Device 发送时的 ID 为 `Dev ID`



## 反馈帧

使用扩展 ID，格式如下：

```
| bit [29:22] 错误码 | bit [21:14] 当前模式 | bit [13:11] 反馈数据内容标识 | bit [10:8] 保留 | bit [7:0] Master ID |
```

反馈数据域格式：

使用小端序。

数据内容标识为 `1` 时反馈如下内容

```
byte [0:3] 数据 1 (int32_t), byte [4:5] 数据 2 (uint16_t), byte [6:7] 数据 3 (int16_t)
```

数据内容标识为 `2` 时反馈如下内容

```
byte [0:3] 数据 4 (uint32_t), byte [4:7] 数据 5 (float, IEEE754)
```

## 发送帧

控制设备时 CAN 发送内容如下：

使用扩展 ID，格式如下：

```
| bit [15:8] Master ID | bit [7:0] Dev ID |
```

## 代码示例

``` C
typedef struct {
    uint32_t master_id;
    uint32_t dev_id;

    int32_t data1;
    uint16_t data2;
    int16_t data3;
    uint32_t data4;
    float data5;

    uint8_t mode;
    uint8_t error_code;
} can_dev_t;

static void can_dev_feedback(void *node_obj, can_rx_header_t *can_rx_header, uint8_t *can_msg);

void demo (void) {
    /* 初始化 can 设备 */
    can1_init ();
    can2_init ();

    uint8_t send_data [8];

    /* 假设用 CAN1 通信，Master ID = 1, Device ID = 2 */
    can_dev_t dev_demo;
    dev_demo.master_id = 1;
    dev_demo.dev_id = 2;
    
    /* 注册 CAN 回调设备示例 */
    can_list_add_new_node(can1_selected,          /* 使用 CAN1 */
                          dev_demo.master_id,     /* 反馈时是以 Master ID 作为标识，因此填 master_id */
                          0xFF,                   /* Bit [7:0] 为 ID 位 */
                          &dev_demo,              /* 设备对象指针 */
                          can_dev_feedback);      /* 设备对象回调函数 */
    /* 发送示例 */
    can_send_message(can1_selected,                                /* 使用 CAN1 */ 
                     CAN_ID_EXT,                                   /* 使用扩展帧 */
                     (dev_demo.master_id << 8) | dev_demo.dev_id,  /* 包装 ID */
                     8,                                            /* 数据长度为 8 */
                     send_data);                                   /* 发送的数据内容 */
}

void can_dev_feedback(void *node_obj, CAN_RxHeaderTypeDef *can_rx_header, uint8_t *can_msg) {
    can_dev_t *dev = (can_dev_t *) node_obj;
    /* 省略一些判断条件，比如是不是扩展帧，数据长度之类的 */
    
    /* 判断 bit [13:11] */
    switch ((can_rx_head->ExtId >> 11) & 0x07) {
        case 1: {
            /* 由于是小端序所以采取直接复制内存 */
            memcpy(dev->data1, &can_msg [0], sizeof (int32_t));
            memcpy(dev->data2, &can_msg [4], sizeof (uint16_t));
            memcpy(dev->data3, &can_msg [6], sizeof (int16_t));
        } break;
        
        case 2: {
            memcpy (dev->data4, &can_msg [0], sizeof (uint32_t));
            memcpy (dev->data5, &can_msg [4], sizeof (float));
        } break;
        
        default: {
        } break;
    }
}
```



