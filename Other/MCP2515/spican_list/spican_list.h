/**
 * @file    spican_list.c
 * @author  Deadline039,Dominate0017
 * @brief   SPICAN Receive list.
 * @version 1.0
 * @date    2026-3-15
 * @note    We will overload the EXTI interrupt callback function.
 */

#ifndef __SPICAN_LIST_H
#define __SPICAN_LIST_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include <CSP_Config.h>
#include "SPICAN/CANSPI.h"
#include "./led/led.h"

/* Use FDCAN or bxCAN2.0. Determined by the chip. */

#define SPICAN_LIST_MAX_CAN_NUMBER 3

#define SPICAN_LIST_MALLOC(p)      malloc(p)
#define SPICAN_LIST_CALLOC(x, p)   calloc(x, p)
#define SPICAN_LIST_FREE(p)        free(p)

/**
 * When disabled, the message is processed in the interrupt.
 *
 * When enabled, a thread will be created to process the message. After
 * receiving the CAN message, a semaphore will be sent to the processing thread
 * to speed up the interrupt exit time.
 *
 * Attention: Only support FreeRTOS. You should modify the code if you want use
 * other RTOS.
 */
#define SPICAN_LIST_USE_RTOS       1

#if SPICAN_LIST_USE_RTOS
#define SPICAN_LIST_TASK_NAME     "Can list"
#define SPICAN_LIST_TASK_PRIORITY 4
#define SPICAN_LSIT_TASK_STK_SIZE 256
#define SPICAN_LIST_QUEUE_LENGTH  5
#endif /* CAN_LIST_USE_RTOS */

#define SPICAN_USE              1
/**
 * @brief Message header type. Compatibility with FDCAN.
 */
typedef struct {
    uint32_t id;         /*!< Message ID.                                     */
    uint32_t id_type;    /*!< ID type, `CAN_ID_STD` or `CAN_ID_EXT`.          */
    uint32_t frame_type; /*!< Frame type, `CAN_RTR_DATA` or `CAN_RTR_REMOTE`. */
    uint8_t data_length; /*!< Message Data length.                            */
} spican_rx_header_t;

typedef enum {
    spican1_selected = 0U, /*!< Select CAN1 */
    spican2_selected,      /*!< Select CAN2 */
    spican3_selected,      /*!< Select CAN3 */
} spican_selected_t;

/**
 * @brief CAN callback function pointer.
 *
 * @param node_obj Node data.
 * @param can_rx_header CAN message rx header.
 * @param can_msg CAN message data.
 */
typedef void (*spican_callback_t)(void * /* node_obj */,
                               spican_rx_header_t * /* can_rx_header */,
                               uint8_t * /* can_msg */);

uint8_t spican_list_add_can(spican_selected_t spican_select, uint32_t std_len,
                         uint32_t ext_len);

uint8_t spican_list_add_new_node(spican_selected_t spican_select, void *node_data,
                              uint32_t id, uint32_t id_mask, uint32_t id_type,
                              spican_callback_t callback);
uint8_t spican_list_del_node_by_id(spican_selected_t spican_select, uint32_t id_type,
                                uint32_t id);
uint8_t spican_list_change_callback(spican_selected_t spican_select, uint32_t id_type,
                                 uint32_t id, spican_callback_t new_callback); 
void mcp2515_process_msg(MCP2515_DevId dev_id);                       
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __SPICAN_LIST_H */


