# N300 使用demo
[time]:2025/4/30

## N300配置
- 串口波特率:1125200
- 发送频率100Hz
- 发送数据包类型 `N300_MSG_AHRS`
- stm32接收数据开启串口: 中断 & DMA—RX

## N300 公开数据
- `yaw` 范围:-180~180
- `roll` 范围:-180~180
- `pitch` 范围:-180~180
- (需要其它数据从frame中读取)

## Demo
```
#include "bsp.h"

/* 创建N300句柄 */
static n300_handle_t n300_data = {0};

int main(void) {
    bsp_init();

    static uint8_t n300_recieve_buf[56];
    uint32_t len;

    while (1) {
        /* 读取DMA接收数据 */
        len = uart_dmarx_read(&usart3_handle, n300_recieve_buf,
                              sizeof(n300_recieve_buf));
        /* 处理接收buf中的数据 */  
        n300_prase(&n300_data, n300_recieve_buf, len);
    }
}
```