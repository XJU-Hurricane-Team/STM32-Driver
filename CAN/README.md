# CAN驱动

1. 将头文件复制到`User/Bsp/Inc/`中，源文件复制到`User/Bsp/Src`中
2. 将`can.c`和`can_list.c`添加到工程的`Bsp`分组中
3. 修改`can.h`的包含头文件为指定芯片，默认是包含的`stm32f4xx_hal.h`
4. 修改`can.h`中CAN1和CAN2的IO
5. 在`bsp.h`中包含`can.h`
6. 添加相应的HAL库驱动，例如：`stm32f4xx_hal_can.c`

# API

## `can`

包括CAN初始化，发送消息等API。

- 初始化调用`can1_init()`或`can2_init()`，需要在这两个函数中修改波特率等参数
- 发送CAN消息调用`can_send_message`
  - `can_select`使用那个CAN发送，`can1_selected`或`can2_selected`
  - `can_ide`ID标识，标准ID（`CAN_ID_STD`）或者扩展ID（`CAN_ID_EXT`）
  - `id`CAN ID
  - `len`消息长度，发送几字节数据
  - `msg`消息内容

## `can_list`

处理CAN回调消息，按照ID调用相应的回调函数。

对于大多数使用扩展帧的CAN设备，其扩展ID基本还会包含一些数据格式。因此我们必须对ID进行处理。

例如，假设一个设备的CAN反馈报文帧ID格式如下：

```
| bit[29:22] 错误码 | bit[21:14]当前模式 | bit[13:11]反馈数据内容标识 | bit[10:8]保留 | bit[7:0] ID |
```

由于`bit[29:8]`包含数据，具体内容是不确定的。而`bit[7:0]`是实际ID，那么我们通过按位与把`bit[7:0]`提取出来就可以了。那么我们掩码`id_mask`就填`0xFF`。这样当CAN中断产生回调时，遍历并依次按照事先指定的掩码将ID按位与就可以找到我们想要的设备了。

回调函数必须是如下形式：

```
void (*can_callback)(void * /* node_obj */,
                     CAN_RxHeaderTypeDef * /* can_rx_header */,
                     uint8_t * /* can_msg */)
例如：
void motor_callback(void *node_obj,
                    CAN_RxHeaderTypeDef *can_rx_header,
                    uint8_t *can_msg) {
}
```



- `can_list_add_new_node`添加新节点
  - `can_select`使用那个CAN接收，`can1_selected`或`can2_selected`
  - `id`设备反馈时的ID
  - `id_mask`设备反馈ID掩码
  - `node_ptr`设备指针，当收到数据并找到相应ID的设备后会将这个指针作为参数传入`callback`函数
  - `callback`收到数据后调用的函数
- `can_list_del_node_by_pointer`通过`node_ptr`删除设备
- `can_list_del_node_by_id`通过ID删除设备
- `can_list_change_id`通过`node_ptr`更改ID
- `can_list_change_callback`通过`node_ptr`更改回调函数
- `can_list_find_node_by_id`通过ID查找`node_ptr`

# 示例

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

Master为主机，这里定义为我们这一端，Device为设备，我们要控制的一端。

Device反馈时的ID为`Master ID`，向Device发送时的ID为`Dev ID`



## 反馈帧

使用扩展ID，格式如下：

```
| bit[29:22] 错误码 | bit[21:14]当前模式 | bit[13:11]反馈数据内容标识 | bit[10:8]保留 | bit[7:0] Master ID |
```

反馈数据域格式：

使用小端序。

数据内容标识为`1`时反馈如下内容

```
byte[0:3] 数据1(int32_t), byte[4:5]数据2(uint16_t), byte[6:7]数据3(int16_t)
```

数据内容标识为`2`时反馈如下内容

```
byte[0:3] 数据4(uint32_t), byte[4:7]数据5(float, IEEE754)
```

## 发送帧

控制设备时CAN发送内容如下：

使用扩展ID，格式如下：

```
| bit[15:8] Master ID | bit[7:0] Dev ID |
```

## 代码示例

```
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

static void can_dev_feedback(void *node_obj, CAN_RxHeaderTypeDef *can_rx_header, uint8_t *can_msg);

void demo (void) {
    /* 初始化can设备 */
    can1_init();
    can2_init();

    uint8_t send_data[8];

    /* 假设用CAN1通信, Master ID = 1, Device ID = 2 */
    can_dev_t dev_demo;
    dev_demo.master_id = 1;
    dev_demo.dev_id = 2;
    
    /* 注册CAN回调设备示例 */
    list_can_add_new_node(can1_selected,          /* 使用CAN1 */
                          dev_demo.master_id,     /* 反馈时是以Master ID作为标识, 因此填master_id */
                          0xFF,                   /* Bit[7:0]为ID位 */
                          &dev_demo,              /* 设备对象指针 */
                          can_dev_feedback);      /* 设备对象回调函数 */
    /* 发送示例 */
    can_send_message(can1_selected,                                /* 使用CAN1 */ 
                     CAN_ID_EXT,                                   /* 使用扩展帧 */
                     (dev_demo.master_id << 8) | dev_demo.dev_id,  /* 包装ID */
                     8,                                            /* 数据长度为8 */
                     send_data);                                   /* 发送的数据内容 */
}

void can_dev_feedback(void *node_obj, CAN_RxHeaderTypeDef *can_rx_header, uint8_t *can_msg) {
    can_dev_t *dev = (can_dev_t *)node_obj;
    /* 省略一些判断条件, 比如是不是扩展帧, 数据长度之类的 */
    
    /* 判断bit[13:11] */
    switch ((can_rx_head->ExtId >> 11) & 0x07) {
        case 1: {
            /* 由于是小端序所以采取直接复制内存 */
            memcpy(dev->data1, &can_msg[0], sizeof(int32_t));
            memcpy(dev->data2, &can_msg[4], sizeof(uint16_t));
            memcpy(dev->data3, &can_msg[6], sizeof(int16_t));
        } break;
        
        case 2: {
            memcpy(dev->data4, &can_msg[0], sizeof(uint32_t));
            memcpy(dev->data5, &can_msg[4], sizeof(float));
        } break;
        
        default: {
        } break;
    }
}
```



