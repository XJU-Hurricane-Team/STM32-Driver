/**
 * @file usart_pro.c
 * @author Deadline039 PickingChip
 * @brief 
 * @version 0.2
 * @date 2026-03-17
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>

#include "ring_fifo.h"
#include <usart.h>
#include "usart_ex.h"

/*****************************************************************************
 * @defgroup Private types and variables of UART.
 * @{
 */

/* The buf of `uart_printf` and `uart_scanf`. */
static char uart_buffer[256];

/**
 * @brief Send buf of UART.
 */
typedef struct {
    uint8_t *send_buf; /*!< Send data buf.                                  */
    uint32_t head_ptr; /*!< Pointer of send buf to control the length of DMA
                            transfer.                                       */
    size_t buf_size;   /*!< The size of buffer. Prevent overflow.           */
} uart_tx_buf_t;

/**
 * @brief Receive fifo of UART.
 */
typedef struct {
    ring_fifo_t *rx_fifo; /*!< Receive fifo.                 */
    uint8_t *rx_fifo_buf; /*!< The storage area of fifo.     */
    uint8_t *recv_buf;    /*!< Data buf of DMA to transfer.  */
    uint32_t head_ptr;    /*!< Pointer of receive buf to
                               control the DMA receive.      */
    uint32_t buf_size;    /*!< Size of `recv_buf`.           */
    uint32_t fifo_size;   /*!< Size of `rx_fifo_buf`.        */
} uart_rx_fifo_t;

/**
 * @}
 */

/*****************************************************************************
 * @defgroup Private functions.
 * @{
 */

static void uart_dmarx_halfdone_callback(UART_HandleTypeDef *huart);
static void uart_dmarx_done_callback(UART_HandleTypeDef *huart);
void uart_dmarx_idle_callback(UART_HandleTypeDef *huart);

static inline uart_rx_fifo_t *uart_rx_identify(UART_HandleTypeDef *huart);
static inline uart_tx_buf_t *uart_tx_identify(UART_HandleTypeDef *huart);

/**
 * @}
 */

/*****************************************************************************
 * @defgroup usart buffer.
 * @{
 */

#ifdef LPUART1_TX_DMA
static uart_tx_buf_t lpuart1_tx_buf = {.buf_size = LPUART1_TX_DMA_BUF_SIZE};
#endif /* LPUART1_TX_DMA */ 
#ifdef LPUART1_RX_DMA
static uart_rx_fifo_t lpuart1_rx_fifo = {.buf_size = LPUART1_RX_DMA_BUF_SIZE,
                                        .fifo_size = LPUART1_RX_DMA_FIFO_SIZE};
#endif /* LPUART1_RX_DMA */
 
#ifdef USART1_TX_DMA
static uart_tx_buf_t usart1_tx_buf = {.buf_size = USART1_TX_DMA_BUF_SIZE};
#endif /* USART1_TX_DMA */
#ifdef USART1_RX_DMA
static uart_rx_fifo_t usart1_rx_fifo = {.buf_size = USART1_RX_DMA_BUF_SIZE,
                                        .fifo_size = USART1_RX_DMA_FIFO_SIZE};
#endif /* USART1_RX_DMA */

#if USART2_TX_DMA
static uart_tx_buf_t usart2_tx_buf = {.buf_size = USART2_TX_DMA_BUF_SIZE};
#endif /* USART2_TX_DMA */
#if USART2_RX_DMA
static uart_rx_fifo_t usart2_rx_fifo = {.buf_size = USART2_RX_DMA_BUF_SIZE,
                                        .fifo_size = USART2_RX_DMA_FIFO_SIZE};
#endif /* USART2_RX_DMA */

#if USART3_TX_DMA
static uart_tx_buf_t usart3_tx_buf = {.buf_size = USART3_TX_DMA_BUF_SIZE};
#endif /* USART3_TX_DMA */
#if USART3_RX_DMA
static uart_rx_fifo_t usart3_rx_fifo = {.buf_size = USART3_RX_DMA_BUF_SIZE,
                                        .fifo_size = USART3_RX_DMA_FIFO_SIZE};
#endif /* USART3_RX_DMA */
#if UART4_TX_DMA

static uart_tx_buf_t uart4_tx_buf = {.buf_size = UART4_TX_DMA_BUF_SIZE};
#endif /* UART4_TX_DMA */
#if UART4_RX_DMA
static uart_rx_fifo_t uart4_rx_fifo = {.buf_size = UART4_RX_DMA_BUF_SIZE,
                                       .fifo_size = UART4_RX_DMA_FIFO_SIZE};
#endif /* UART4_RX_DMA */
#if UART5_TX_DMA
static uart_tx_buf_t uart5_tx_buf = {.buf_size = UART5_TX_DMA_BUF_SIZE};
#endif /* UART5_TX_DMA */
#if UART5_RX_DMA
static uart_rx_fifo_t uart5_rx_fifo = {.buf_size = UART5_RX_DMA_BUF_SIZE,
                                       .fifo_size = UART5_RX_DMA_FIFO_SIZE};
#endif /* UART5_RX_DMA */

/**
 * @}
 */

/*****************************************************************************
 * @defgroup Public UART functions.
 * @{
 */

/**
  * @brief Initialize the UART promote functions. 
  * 
  * @param huart The handle of UART.
  */
void uart_ex_init(UART_HandleTypeDef *huart) {

    if (((huart->gState) & HAL_UART_STATE_READY) == 0) {
        /* The UART is not inited. */
        Error_Handler();
    }
    if (huart->hdmarx != NULL) {
        /* Initialize the UART DMA Rx. */
        uart_rx_fifo_t *uart_rx_fifo = uart_rx_identify(huart);
        if (uart_rx_fifo != NULL) {
            uart_rx_fifo->recv_buf = malloc(uart_rx_fifo->buf_size);
            uart_rx_fifo->rx_fifo_buf = malloc(uart_rx_fifo->fifo_size);
            if ((uart_rx_fifo->recv_buf == NULL) ||
                (uart_rx_fifo->rx_fifo_buf == NULL)) {
                Error_Handler();
            }
            uart_rx_fifo->rx_fifo =
                ring_fifo_init(uart_rx_fifo->rx_fifo_buf,
                               uart_rx_fifo->fifo_size, RF_TYPE_STREAM);
            if (uart_rx_fifo->rx_fifo == NULL) {
                Error_Handler();
            }

            __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
            __HAL_UART_CLEAR_IDLEFLAG(huart);

            HAL_UART_Receive_DMA(huart, uart_rx_fifo->recv_buf,
                                 uart_rx_fifo->buf_size);
        }
    } else {
        Error_Handler();
    }

    if (huart->hdmatx != NULL) {
        /* Initialize the UART DMA Tx. */
        uart_tx_buf_t *uart_tx_buf = uart_tx_identify(huart);
        if (uart_tx_buf != NULL) {
            uart_tx_buf->send_buf = malloc(uart_tx_buf->buf_size);
            if (uart_tx_buf->send_buf == NULL) {
                Error_Handler();
            }
        }
    } else {
        Error_Handler();
    }
}

/**
 * @brief Deinitialize the UART promote functions.
 * 
 * @param huart The handle of UART.
 */
void uart_ex_deinit(UART_HandleTypeDef *huart) {
    if (((huart->gState) & HAL_UART_STATE_READY) == 0) {
        /* The UART is not inited. */
        Error_Handler();
    }
    if (huart->hdmarx != NULL) {
        /* Initialize the UART DMA Rx. */
        uart_rx_fifo_t *uart_rx_fifo = uart_rx_identify(huart);
        if (uart_rx_fifo != NULL) {
            HAL_DMA_Abort(huart->hdmarx);
            free(uart_rx_fifo->recv_buf);
            free(uart_rx_fifo->rx_fifo_buf);
            ring_fifo_destroy(uart_rx_fifo->rx_fifo);
        }
    } else {
        Error_Handler();
    }

    if (huart->hdmatx != NULL) {
        /* Initialize the UART DMA Tx. */
        uart_tx_buf_t *uart_tx_buf = uart_tx_identify(huart);
        if (uart_tx_buf != NULL) {
            HAL_DMA_Abort(huart->hdmatx);
            free(uart_tx_buf->send_buf);
        }
    } else {
        Error_Handler();
    }
}

/**
 * @brief Formatted print to the UART.
 *
 * @param huart The handle of UART.
 * @param __format The string with format.
 * @return The number of characters that would have been written in the
 *         array, not counting the terminating null character.
 */
int uart_printf(UART_HandleTypeDef *huart, const char *__format, ...) {
    int res;
    uint16_t len;
    va_list ap;

    if (((huart->gState) & HAL_UART_STATE_READY) == 0) {
        /* The UART is not inited. */
        return 0;
    }

    /* Wait for last transfer end. */
    while (__HAL_UART_GET_FLAG(huart, UART_FLAG_TC) == RESET)
        ;

    va_start(ap, __format);
    res = vsnprintf(uart_buffer, sizeof(uart_buffer), __format, ap);
    va_end(ap);

    len = strlen(uart_buffer);

    if (huart->hdmatx != NULL) {
        HAL_UART_Transmit_DMA(huart, (uint8_t *)uart_buffer, len);
    } else {
        HAL_UART_Transmit(huart, (uint8_t *)uart_buffer, len, 1000);
    }

    return res;
}

/**
 * @brief Formatted receive from the UART.
 *
 * @param huart The handle of UART.
 * @param __format The string with format.
 * @return The value of the macro EOF if an input failure occurs before any
 *         conversion. Otherwise, the scanf function returns the number of
 *         input items assigned, which can be fewer than provided for, or
 *         even zero, in the event of an early matching failure.
 */
int uart_scanf(UART_HandleTypeDef *huart, const char *__format, ...) {
    uint16_t str_len = 0;
    int res;
    va_list ap;

    if (((huart->gState) & HAL_UART_STATE_READY) == 0) {
        /* The UART is not inited. */
        return 0;
    }

    if (huart->hdmarx != NULL) {
        while (str_len == 0) {
            str_len = uart_dmarx_read(huart, uart_buffer, sizeof(uart_buffer));
        }
    } else {
        HAL_UARTEx_ReceiveToIdle(huart, (uint8_t *)uart_buffer,
                                 sizeof(uart_buffer), &str_len, 0xFFFF);
    }

    va_start(ap, __format);
    res = vsscanf((char *)uart_buffer, __format, ap);
    va_end(ap);

    return res;
}

/**
 * @}
 */

/*****************************************************************************
 * @defgroup Public UART DMA Rx functions.
 * @{
 */

/**
 * @brief Identify the UART receive fifo by handle.
 *
 * @param huart The handle of UART
 * @return The point of UART rx fifo.
 */
static inline uart_rx_fifo_t *uart_rx_identify(UART_HandleTypeDef *huart) {

    switch ((uintptr_t)huart->Instance) {
#if LPUART1_RX_DMA
        case LPUART1_BASE: {
            return &lpuart1_rx_fifo;
        }
#endif /* LPUART1_RX_DMA */

#if USART1_RX_DMA
        case USART1_BASE: {
            return &usart1_rx_fifo;
        }
#endif /* USART1_RX_DMA */

#if USART2_RX_DMA
        case USART2_BASE: {
            return &usart2_rx_fifo;
        }
#endif /* USART2_RX_DMA */
#if USART3_RX_DMA
        case USART3_BASE: {
            return &usart3_rx_fifo;
        }
#endif /* USART3_RX_DMA */

#if UART4_RX_DMA
        case UART4_BASE: {
            return &uart4_rx_fifo;
        }
#endif /* UART4_RX_DMA */

#if UART5_RX_DMA
        case UART5_BASE: {
            return &uart5_rx_fifo;
        }
#endif /* UART5_RX_DMA */

        default: {
        } break;
    }

    return NULL;
}

/**
 * @brief UART received idle callback.
 *
 * @param huart The handle of UART
 */
void uart_dmarx_idle_callback(UART_HandleTypeDef *huart) {
    uart_rx_fifo_t *uart_rx_fifo = uart_rx_identify(huart);
    if (uart_rx_fifo == NULL) {
        return;
    }

    uint32_t tail_ptr;
    uint32_t copy, offset;

    /**
     * +~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~+
     * |     head_ptr          tail_ptr         |
     * |         |                 |            |
     * |         v                 v            |
     * | --------*******************----------- |
     * +~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~+
     */

    /* Received */
    tail_ptr = huart->RxXferSize - __HAL_DMA_GET_COUNTER(huart->hdmarx);

    offset = (uart_rx_fifo->head_ptr) % (uint32_t)(huart->RxXferSize);
    copy = tail_ptr - offset;
    uart_rx_fifo->head_ptr += copy;

    ring_fifo_write(uart_rx_fifo->rx_fifo, huart->pRxBuffPtr + offset, copy);
}

/**
 * @brief UART DMA half overflow callback.
 *
 * @param huart The handle of UART
 */
void uart_dmarx_halfdone_callback(UART_HandleTypeDef *huart) {
    uart_rx_fifo_t *uart_rx_fifo = uart_rx_identify(huart);
    if (uart_rx_fifo == NULL) {
        return;
    }

    uint32_t tail_ptr;
    uint32_t offset, copy;

    /**
     * +~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~+
     * |                  half                  |
     * |     head_ptr   tail_ptr                |
     * |         |          |                   |
     * |         v          v                   |
     * | --------*******************----------- |
     * +~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~+
     */

    tail_ptr = (huart->RxXferSize >> 1) + (huart->RxXferSize & 1);

    offset = (uart_rx_fifo->head_ptr) % (uint32_t)(huart->RxXferSize);
    copy = tail_ptr - offset;
    uart_rx_fifo->head_ptr += copy;

    ring_fifo_write(uart_rx_fifo->rx_fifo, huart->pRxBuffPtr + offset, copy);
}

/**
 * @brief UART DMA overflow callback.
 *
 * @param huart The handle of UART
 */
void uart_dmarx_done_callback(UART_HandleTypeDef *huart) {
    uart_rx_fifo_t *uart_rx_fifo = uart_rx_identify(huart);
    if (uart_rx_fifo == NULL) {
        return;
    }

    uint32_t tail_ptr;
    uint32_t offset, copy;

    /**
     * +~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~+
     * |                  half                  |
     * |                    | head_ptr tail_ptr |
     * |                    |    |            | |
     * |                    v    v            v |
     * | ------------------------************** |
     * +~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~+
     */

    tail_ptr = huart->RxXferSize;

    offset = (uart_rx_fifo->head_ptr) % (uint32_t)(huart->RxXferSize);
    copy = tail_ptr - offset;
    uart_rx_fifo->head_ptr += copy;

    ring_fifo_write(uart_rx_fifo->rx_fifo, huart->pRxBuffPtr + offset, copy);

    if (huart->hdmarx->Init.Mode != DMA_CIRCULAR) {
        /* Reopen the DMA receive. */
        while (HAL_UART_Receive_DMA(huart, huart->pRxBuffPtr,
                                    huart->RxXferSize) != HAL_OK) {
            __HAL_UNLOCK(huart);
        }
    }
}

/**
 * @brief Read from UART Receive fifo.
 *
 * @param huart The handle of UART
 * @param[out] buf The data buf which receive the data from the fifo.
 * @param buf_size The size of buf.
 * @return The length that be received.
 */
uint32_t uart_dmarx_read(UART_HandleTypeDef *huart, void *buf,
                         size_t buf_size) {
    if ((buf == NULL) || (buf_size == 0)) {
        return 0;
    }
    uart_rx_fifo_t *uart_rx_fifo = uart_rx_identify(huart);

    if (uart_rx_fifo == NULL) {
        return 0;
    }

    return ring_fifo_read(uart_rx_fifo->rx_fifo, buf, buf_size);
}

/**
 * @brief Read from UART DMA raw receive buffer directly.
 *
 * @param huart The handle of UART
 * @param[out] buf The data buf which receive the data from the raw recv_buf.
 * @param max_len The max length to copy.
 * @return The length that be copied.
 */
uint32_t uart_dmarx_read_raw_buf(UART_HandleTypeDef *huart, void *buf,
                                 size_t max_len) {
    if ((buf == NULL) || (max_len == 0)) {
        return 0;
    }
    
    uart_rx_fifo_t *uart_rx_fifo = uart_rx_identify(huart);

    if ((uart_rx_fifo == NULL) || (uart_rx_fifo->recv_buf == NULL)) {
        return 0;
    }

    uint32_t copy_len = (max_len > uart_rx_fifo->buf_size) ? uart_rx_fifo->buf_size : max_len;
    memcpy(buf, uart_rx_fifo->recv_buf, copy_len);

    return copy_len;
}

/**
 * @brief Resize the receive buf and fifo of UART.
 *
 * @param huart The handle of UART
 * @param buf_size New buf size
 * @param fifo_size New fifo size
 * @return Resize message:
 * @retval - 0: Success
 * @retval - 1: This uart not enable DMA Rx.
 * @retval - 2: This UART is enabled.
 * @retval - 3: Parameter Error, size can't be 0.
 * @warning Must be disabled the UART before resize!
 *          You should call `u(s)artx_deinit()` before call this function,
 *          than call `u(s)artx_init()` to using new size.
 *          It may allocated fail when reinitialize uart.
 */
uint8_t uart_dmarx_resize_fifo(UART_HandleTypeDef *huart, uint32_t buf_size,
                               uint32_t fifo_size) {
    if ((buf_size == 0) || (fifo_size == 0)) {
        return 3;
    }

    uart_rx_fifo_t *uart_rx_fifo = uart_rx_identify(huart);
    if (uart_rx_fifo == NULL) {
        return 1;
    }

    if (huart->hdmatx != NULL) {
        return 2;
    }

    uart_rx_fifo->buf_size = buf_size;
    uart_rx_fifo->fifo_size = fifo_size;
    return 0;
}

/**
 * @brief Get the buffer size of UART DMA Rx.
 *
 * @param huart The handle of UART.
 * @return The buffer size of UART.
 */
uint32_t uart_dmarx_get_buf_size(UART_HandleTypeDef *huart) {
    uart_rx_fifo_t *uart_rx_fifo = uart_rx_identify(huart);
    if (uart_rx_fifo == NULL) {
        return 0;
    }

    return uart_rx_fifo->buf_size;
}

/**
 * @brief Get the fifo size of UART DMA Rx.
 *
 * @param huart The handle of UART.
 * @return The buffer size of UART.
 */
uint32_t uart_dmarx_get_fifo_size(UART_HandleTypeDef *huart) {
    uart_rx_fifo_t *uart_rx_fifo = uart_rx_identify(huart);
    if (uart_rx_fifo == NULL) {
        return 0;
    }

    return uart_rx_fifo->fifo_size;
}
/**
 * @}
 */

/*****************************************************************************
 * @defgroup Public UART DMA Tx functions.
 * @{
 */

/**
 * @brief Identify the UART transmit buffer by handle.
 *
 * @param huart The handle of UART
 * @return The point of UART tx buffer.
 */
static inline uart_tx_buf_t *uart_tx_identify(UART_HandleTypeDef *huart) {
    switch ((uintptr_t)(huart->Instance)) {
#if LPUART1_TX_DMA
        case LPUART1_BASE: {
            return &lpuart1_tx_buf;
        }
#endif /* LPUART1_TX_DMA */

#if USART1_TX_DMA
        case USART1_BASE: {
            return &usart1_tx_buf;
        }
#endif /* USART1_TX_DMA */

#if USART2_TX_DMA
        case USART2_BASE: {
            return &usart2_tx_buf;
        }
#endif /* USART2_TX_DMA */

#if USART3_TX_DMA
        case USART3_BASE: {
            return &usart3_tx_buf;
        }
#endif /* USART3_TX_DMA */

#if UART4_TX_DMA
        case UART4_BASE: {
            return &uart4_tx_buf;
        }
#endif /* UART4_TX_DMA */

#if UART5_TX_DMA
        case UART5_BASE: {
            return &uart5_tx_buf;
        }
#endif /* UART5_TX_DMA */

        default: {
        } break;
    }

    return NULL;
}

/**
 * @brief Write the transmit data to the buffer.
 *
 * @param huart The handle of UART.
 * @param data The data will be write.
 * @param len The data length will be written.
 * @return The length that be written.
 */
uint32_t uart_dmatx_write(UART_HandleTypeDef *huart, const void *data,
                          size_t len) {
    if ((data == NULL) || (len == 0)) {
        return 0;
    }

    uart_tx_buf_t *send_tx_buf = uart_tx_identify(huart);
    if (send_tx_buf == NULL) {
        return 0;
    }

    /* Get the remain length of buffer. */
    uint32_t buf_remain = send_tx_buf->buf_size - send_tx_buf->head_ptr;

    /* Prevent overflow. */
    if (buf_remain < len) {
        memcpy(send_tx_buf->send_buf + send_tx_buf->head_ptr, data, buf_remain);
        send_tx_buf->head_ptr += buf_remain;
        return buf_remain;
    } else {
        memcpy(send_tx_buf->send_buf + send_tx_buf->head_ptr, data, len);
        send_tx_buf->head_ptr += len;
        return len;
    }
}

/**
 * @brief Transmit the data in the buf.
 *
 * @param huart The handle of UART.
 * @return The length which is succeed transmitted.
 * @note If you want transmit data, using `uart_dmatx_write` before.
 *       If you have huge continous data to transmit, we recommand use
 *       `HAL_UART_Transmit_DMA()`.
 */
uint32_t uart_dmatx_send(UART_HandleTypeDef *huart) {
    uart_tx_buf_t *send_tx_buf = uart_tx_identify(huart);
    if (send_tx_buf == NULL) {
        return 0;
    }

    if (huart->hdmatx == NULL) {
        return 0;
    }

    uint32_t len = send_tx_buf->head_ptr;
    if (len == 0) {
        return 0;
    }

    /* Wait for last transfer end. */
    while (__HAL_UART_GET_FLAG(huart, UART_FLAG_TC) == RESET)
        ;

    HAL_UART_Transmit_DMA(huart, send_tx_buf->send_buf, (uint16_t)len);
    send_tx_buf->head_ptr = 0;
    return len;
}

/**
 * @brief Resize the send buf of UART.
 *
 * @param huart The handle of UART
 * @param size New size
 * @return Resize message:
 * @retval - 0: Success
 * @retval - 1: This uart not enable DMA Tx.
 * @retval - 2: No free memory to allocate.
 * @retval - 3: This uart is busy now.
 * @retval - 4: Parameter error, size can't be 0.
 */
uint8_t uart_dmatx_resize_buf(UART_HandleTypeDef *huart, uint32_t size) {
    if (size == 0) {
        return 4;
    }

    uart_tx_buf_t *send_tx_buf = uart_tx_identify(huart);
    if (send_tx_buf == NULL) {
        return 1;
    }

    if ((huart->gState) & (HAL_UART_STATE_BUSY_TX | HAL_UART_STATE_BUSY) &
        ~HAL_UART_STATE_READY) {
        /* The UART is busy. */
        return 3;
    }

    /* The UART is uninitialized, just adjust the size. */
    if (huart->hdmatx == NULL) {
        send_tx_buf->buf_size = size;
        return 0;
    }

    uint8_t *new_ptr = realloc(send_tx_buf->send_buf, size);

    if (new_ptr == NULL) {
        return 2;
    }

    send_tx_buf->send_buf = new_ptr;
    send_tx_buf->buf_size = size;

    return 0;
}

/**
 * @brief Get the buffer size of UART DMA Tx.
 *
 * @param huart The handle of UART.
 * @return The size of UART DMA Tx buf.
 */
uint32_t uart_dmatx_get_buf_size(UART_HandleTypeDef *huart) {
    uart_tx_buf_t *uart_tx_buf = uart_tx_identify(huart);

    if (uart_tx_buf == NULL) {
        return 0;
    }

    return uart_tx_buf->buf_size;
}

/**
 * @}
 */

/*****************************************************************************
 * @defgroup UART interrupt callbacks.
 * @{
 */

/**
 * @brief UART error callbacks.
 *
 * @param huart The handle of UART.
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    __IO uint32_t error_code = 0x00U;

    error_code = HAL_UART_GetError(huart);
    if (HAL_UART_ERROR_NONE == error_code) {
        return;
    }

    switch (error_code) {
        case HAL_UART_ERROR_PE: {
            __HAL_UART_CLEAR_PEFLAG(huart);
        } break;

        case HAL_UART_ERROR_NE: {
            __HAL_UART_CLEAR_NEFLAG(huart);
        } break;

        case HAL_UART_ERROR_FE: {
            __HAL_UART_CLEAR_FEFLAG(huart);
        } break;

        case HAL_UART_ERROR_ORE: {
            __HAL_UART_CLEAR_OREFLAG(huart);
        } break;

        case HAL_UART_ERROR_DMA: {
        } break;

        default: {
        } break;
    }

    __HAL_UART_FLUSH_DRREGISTER(huart);
}

#if USE_HAL_UART_REGISTER_CALLBACKS == 0

/**
 * @brief Rx Transfer completed callbacks.
 *
 * @param huart The handle of UART.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->hdmarx != NULL) {
        uart_dmarx_done_callback(huart);
    }
}

/**
 * @brief Rx Half Transfer completed callbacks.
 *
 * @param huart The handle of UART.
 */
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->hdmarx != NULL) {
        uart_dmarx_halfdone_callback(huart);
    }
}

#endif /* USE_HAL_UART_REGISTER_CALLBACKS == 0 */

/**
 * @}
 */
