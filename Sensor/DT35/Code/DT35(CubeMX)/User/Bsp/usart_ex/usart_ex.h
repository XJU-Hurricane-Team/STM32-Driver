/**
 * @file usart_ex.h
 * @author Deadline039 PickingChip
 * @brief 
 * @version 0.1
 * @date 2026-03-17
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef __USART_EX_H__
#define __USART_EX_H__

#ifdef __cplusplus
extern "C" {
#endif

/*****************************************************************************
 * @defgroup Configuration Macros of USART_EX.
 * @{
 */
//-------- <<< Use Configuration Wizard in Context Menu >>> --------------------

// <h> USART1 Configuration
//   <e> USART1_TX_DMA               Enable USART1 TX DMA
#define USART1_TX_DMA 1

#ifdef USART1_TX_DMA
//   <o> USART1_TX_DMA_BUF_SIZE      USART1 TX DMA Buffer Size <8-4096:8>
#define USART1_TX_DMA_BUF_SIZE 256
//   </e> USART1_TX_DMA
#endif /* USART1_TX_DMA */

//   <e> USART1_RX_DMA               Enable USART1 RX DMA
#define USART1_RX_DMA 1

#ifdef USART1_RX_DMA
//   <o> USART1_RX_DMA_BUF_SIZE      USART1 RX DMA Buffer Size <8-4096:8>
#define USART1_RX_DMA_BUF_SIZE  256
//   <o> USART1_RX_DMA_FIFO_SIZE     USART1 RX DMA FIFO Size   <8-4096:8>
#define USART1_RX_DMA_FIFO_SIZE 256
//   </e> USART1_RX_DMA
#endif /* USART1_RX_DMA */

// </h>

// <h> USART2 Configuration
//   <e> USART2_TX_DMA               Enable USART2 TX DMA
#define USART2_TX_DMA 0

#ifdef USART2_TX_DMA
//   <o> USART2_TX_DMA_BUF_SIZE      USART2 TX DMA Buffer Size <8-4096:8>
#define USART2_TX_DMA_BUF_SIZE 256
//   </e> USART2_TX_DMA
#endif /* USART2_TX_DMA */

//   <e> USART2_RX_DMA               Enable USART2 RX DMA
#define USART2_RX_DMA 0

#ifdef USART2_RX_DMA
//   <o> USART2_RX_DMA_BUF_SIZE      USART2 RX DMA Buffer Size <8-4096:8>
#define USART2_RX_DMA_BUF_SIZE  256
//   <o> USART2_RX_DMA_FIFO_SIZE     USART2 RX DMA FIFO Size   <8-4096:8>
#define USART2_RX_DMA_FIFO_SIZE 256
//   </e> USART2_RX_DMA
#endif /* USART2_RX_DMA */

// </h>

// <h> USART3 Configuration
//   <e> USART3_TX_DMA               Enable USART3 TX DMA
#define USART3_TX_DMA 0

#ifdef USART3_TX_DMA
//   <o> USART3_TX_DMA_BUF_SIZE      USART3 TX DMA Buffer Size <8-4096:8>
#define USART3_TX_DMA_BUF_SIZE 256
//   </e> USART3_TX_DMA
#endif /* USART3_TX_DMA */

//   <e> USART3_RX_DMA               Enable USART3 RX DMA
#define USART3_RX_DMA 0

#ifdef USART3_RX_DMA
//   <o> USART3_RX_DMA_BUF_SIZE      USART3 RX DMA Buffer Size <8-4096:8>
#define USART3_RX_DMA_BUF_SIZE  256
//   <o> USART3_RX_DMA_FIFO_SIZE     USART3 RX DMA FIFO Size   <8-4096:8>
#define USART3_RX_DMA_FIFO_SIZE 256
//   </e> USART3_RX_DMA
#endif /* USART3_RX_DMA */

// </h>

// <h> UART4 Configuration
//   <e> UART4_TX_DMA                Enable UART4 TX DMA
#define UART4_TX_DMA 0

#ifdef UART4_TX_DMA
//   <o> UART4_TX_DMA_BUF_SIZE       UART4 TX DMA Buffer Size  <8-4096:8>
#define UART4_TX_DMA_BUF_SIZE 256
//   </e> UART4_TX_DMA
#endif /* UART4_TX_DMA */

//   <e> UART4_RX_DMA                Enable UART4 RX DMA
#define UART4_RX_DMA 0

#ifdef UART4_RX_DMA
//   <o> UART4_RX_DMA_BUF_SIZE       UART4 RX DMA Buffer Size  <8-4096:8>
#define UART4_RX_DMA_BUF_SIZE  256
//   <o> UART4_RX_DMA_FIFO_SIZE      UART4 RX DMA FIFO Size    <8-4096:8>
#define UART4_RX_DMA_FIFO_SIZE 256
//   </e> UART4_RX_DMA
#endif /* UART4_RX_DMA */

// </h>

// <h> UART5 Configuration
//   <e> UART5_TX_DMA                Enable UART5 TX DMA
#define UART5_TX_DMA 0

#ifdef UART5_TX_DMA
//   <o> UART5_TX_DMA_BUF_SIZE       UART5 TX DMA Buffer Size  <8-4096:8>
#define UART5_TX_DMA_BUF_SIZE 256
//   </e> UART5_TX_DMA
#endif /* UART5_TX_DMA */

//   <e> UART5_RX_DMA                Enable UART5 RX DMA
#define UART5_RX_DMA 0

#ifdef UART5_RX_DMA
//   <o> UART5_RX_DMA_BUF_SIZE       UART5 RX DMA Buffer Size  <8-4096:8>
#define UART5_RX_DMA_BUF_SIZE  256
//   <o> UART5_RX_DMA_FIFO_SIZE      UART5 RX DMA FIFO Size    <8-4096:8>
#define UART5_RX_DMA_FIFO_SIZE 256
//   </e> UART5_RX_DMA
#endif /* UART5_RX_DMA */

// </h>

//------------- <<< end of configuration section >>> -----------------------

/**
 * @}
 */

/*****************************************************************************
 * @defgroup Public uart function.
 * @{
 */

void uart_ex_init(UART_HandleTypeDef *huart);
void uart_ex_deinit(UART_HandleTypeDef *huart);

int uart_printf(UART_HandleTypeDef *huart, const char *__format, ...);
int uart_scanf(UART_HandleTypeDef *huart, const char *__format, ...);

uint32_t uart_dmarx_read(UART_HandleTypeDef *huart, void *buf, size_t len);
uint8_t uart_dmarx_resize_fifo(UART_HandleTypeDef *huart, uint32_t buf_size,
                               uint32_t fifo_size);
uint32_t uart_dmarx_get_buf_size(UART_HandleTypeDef *huart);
uint32_t uart_dmarx_get_fifo_size(UART_HandleTypeDef *huart);

uint32_t uart_dmatx_write(UART_HandleTypeDef *huart, const void *data,
                          size_t len);
uint32_t uart_dmatx_send(UART_HandleTypeDef *huart);
uint8_t uart_dmatx_resize_buf(UART_HandleTypeDef *huart, uint32_t size);
uint32_t uart_dmatx_get_buf_size(UART_HandleTypeDef *huart);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __USART_EX_H__ */