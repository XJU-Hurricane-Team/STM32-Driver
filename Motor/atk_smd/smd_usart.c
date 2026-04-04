/**
 * @file smd_usart.c
 * @author PickingChip
 * @brief 正点原子 步进电机驱动器 USART通信代码
 * @version 0.1
 * @date 2026-04-03
 * @note 使用DMA发送时依赖uart_ex.c,需要手动开启uart_ex.c中的发送，但不要开启接收功能。     
 * 
 */

#include "smd_usart.h"
#include "smd.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

#define SMD_UART_WAIT_TICKS pdMS_TO_TICKS(50) /* 等待上一次发送-应答完成 */
#define SMD_RX_QUEUE_LEN    1                 /* 接收队列长度 */
#define SMD_DMA             0                 /* 是否使用DMA发送 */

#if SMD_DMA
#include "./usart_ex/usart_ex.h"
#endif

typedef struct {
    UART_HandleTypeDef *huart;
    size_t Size;
} smd_recv_msg_t;

uint8_t g_rx_cmd[RX_BUFFER_SIZE]; /* 存放接收到的指令 */
SERIAL_FRAME g_serial_frame;      /* 消息帧 */

static QueueHandle_t smd_queue_handle = NULL;
static TaskHandle_t smd_receive_task_handle = NULL;
static SemaphoreHandle_t smd_uart_available = NULL;
static smd_recv_msg_t send_msg_from_isr;

/**
 * @brief       串口发送发命令
 * @param       *cmd: 数据指针
 * @param       len: 长度
 * @retval      无
 */
void smd_usart_send_cmd(uint8_t *data, uint8_t len) {
    if ((data == NULL) || (len == 0) || (smd_uart_available == NULL)) {
        return;
    }

    /* 确保串口没有被占用 */
    if (xSemaphoreTake(smd_uart_available, SMD_UART_WAIT_TICKS) != pdTRUE) {
        /* 串口忙，当前命令不发送，避免打断正在进行的接收 */
        return;
    }

    RS485_RE(1); /* 进入发送模式 */

#if SMD_DMA
    if (uart_dmatx_write(SMD_UART, data, len) == 0) {
        RS485_RE(0); /* 进入接收模式 */
        xSemaphoreGive(smd_uart_available);
        return;
    }

    if (uart_dmatx_send(SMD_UART) == 0) {
        RS485_RE(0); /* 进入接收模式 */
        xSemaphoreGive(smd_uart_available);
        return;
    }
#else /* 阻塞式发送 */

    if (HAL_UART_Transmit(SMD_UART, data, len, 100) != HAL_OK) {
        /* 发送失败时立即释放串口，避免发送任务永久阻塞 */
        RS485_RE(0); /* 进入接收模式 */
        xSemaphoreGive(smd_uart_available);
        return;
    }

    /*
     * HAL_UART_Transmit 是阻塞发送，不会触发 HAL_UART_TxCpltCallback。
     * 因此在这里直接切回接收并启动 ToIdle DMA。
     */
    RS485_RE(0); /* 进入接收模式 */
    if (HAL_UARTEx_ReceiveToIdle_DMA(SMD_UART, g_rx_cmd, sizeof(g_rx_cmd)) !=
        HAL_OK) {
        xSemaphoreGive(smd_uart_available);
    }

#endif /* SMD_DMA */
}

/**
 * @brief 正点步进串口发送完成回调函数
 * @note 在usart_ex.c中被HAL_UART_TxCpltCallback调用
 * @param huart 串口句柄
 */
void atksmd_uart_tx_cplt_callback(UART_HandleTypeDef *huart) {
    if (huart == SMD_UART) {
        /* 发送完成后立即启动接收 */
        RS485_RE(0); /* 进入接收模式 */
        if (HAL_UARTEx_ReceiveToIdle_DMA(huart, g_rx_cmd, sizeof(g_rx_cmd)) !=
            HAL_OK) {
            /* 接收 DMA 启动失败时释放串口，避免发送任务永久阻塞 */
            xSemaphoreGiveFromISR(smd_uart_available, NULL);
        }
    }
}

/**
 * @brief 正点电机串口接收完成回调函数
 * @note 在usart_ex.c中被HAL_UART_RxCpltCallback调用
 * @param huart 串口句柄
 * @param Size 接收数据长度
 */
void atksmd_uart_rx_cplt_callback(UART_HandleTypeDef *huart, uint16_t Size) {

    if (huart == SMD_UART) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        if ((smd_queue_handle == NULL) || (smd_uart_available == NULL)) {
            return;
        }

        send_msg_from_isr.huart = huart;
        send_msg_from_isr.Size = Size;
        (void)xQueueSendFromISR(smd_queue_handle, &send_msg_from_isr,
                                &xHigherPriorityTaskWoken);
        /* 接收完成释放对串口的占用 */
        xSemaphoreGiveFromISR(smd_uart_available, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

#if SMD_DMA /* 如果使用DMA，在uart_ex.c中实现回调函数 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    atksmd_uart_tx_cplt_callback(huart);
}
#endif /* SMD_DMA */


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    atksmd_uart_rx_cplt_callback(huart, Size);
}

/**
 * @brief 正点电机串口接收解析任务
 * @param pvParameters 参数
 */
void atk_smd_recv_task(void *pvParameters) {
    UNUSED(pvParameters);

    smd_recv_msg_t recv_msg;

    while (1) {
        if (xQueueReceive(smd_queue_handle, &recv_msg, portMAX_DELAY) ==
            pdPASS) {
            if (recv_msg.Size >= 6 && recv_msg.Size <= RX_BUFFER_SIZE) {
                /* 处理接收到的数据 */
                serial_frame_process(g_rx_cmd, (uint8_t)recv_msg.Size,
                                     &g_serial_frame);
            }
        }
    }
}

/**
 * @brief 正点原子步进电机串口接收初始化
 * 
 * @param pvParameters 
 */
void atk_smd_usart_init(void) {

    /* 创建消息队列 */
    smd_queue_handle = xQueueCreate(SMD_RX_QUEUE_LEN, sizeof(smd_recv_msg_t));
    configASSERT(smd_queue_handle != NULL);

    /* 创建接收任务 */
    BaseType_t result = xTaskCreate(atk_smd_recv_task, "atk_smd_recv_task", 512,
                                    NULL, 4, &smd_receive_task_handle);
    configASSERT(result == pdPASS);
    /* 创建串口二值信号量 */
    smd_uart_available = xSemaphoreCreateBinary();
    configASSERT(smd_uart_available != NULL);
    xSemaphoreGive(smd_uart_available);
}
