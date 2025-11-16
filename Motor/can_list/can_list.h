/**
 * @file    can_list.c
 * @author  Deadline039
 * @brief   CAN Receive list.
 * @version 1.0
 * @date    2024-11-24
 * @note    We will overload the CAN interrupt callback functions, include CAN
 *          RX0 and RX1 FIFO pending callbacck.
 */

#ifndef __CAN_LIST_H
#define __CAN_LIST_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include <CSP_Config.h>

/* Use FDCAN or bxCAN2.0. Determined by the chip. */
#define CAN_LIST_USE_FDCAN      0

#define CAN_LIST_MAX_CAN_NUMBER 3

#define CAN_LIST_MALLOC(p)      malloc(p)
#define CAN_LIST_CALLOC(x, p)   calloc(x, p)
#define CAN_LIST_FREE(p)        free(p)

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
#define CAN_LIST_USE_RTOS       0

#if CAN_LIST_USE_RTOS
#define CAN_LIST_TASK_NAME     "Can list"
#define CAN_LIST_TASK_PRIORITY 2
#define CAN_LSIT_TASK_STK_SIZE 256
#define CAN_LIST_QUEUE_LENGTH  5
#endif /* CAN_LIST_USE_RTOS */

/**
 * @brief Message header type. Compatibility with FDCAN.
 */
typedef struct {
    uint32_t id;         /*!< Message ID.                                     */
    uint32_t id_type;    /*!< ID type, `CAN_ID_STD` or `CAN_ID_EXT`.          */
    uint32_t frame_type; /*!< Frame type, `CAN_RTR_DATA` or `CAN_RTR_REMOTE`. */
    uint8_t data_length; /*!< Message Data length.                            */
} can_rx_header_t;

/**
 * @brief CAN callback function pointer.
 *
 * @param node_obj Node data.
 * @param can_rx_header CAN message rx header.
 * @param can_msg CAN message data.
 */
typedef void (*can_callback_t)(void * /* node_obj */,
                               can_rx_header_t * /* can_rx_header */,
                               uint8_t * /* can_msg */);

uint8_t can_list_add_can(can_selected_t can_select, uint32_t std_len,
                         uint32_t ext_len);

uint8_t can_list_add_new_node(can_selected_t can_select, void *node_data,
                              uint32_t id, uint32_t id_mask, uint32_t id_type,
                              can_callback_t callback);
uint8_t can_list_del_node_by_id(can_selected_t can_select, uint32_t id_type,
                                uint32_t id);
uint8_t can_list_change_callback(can_selected_t can_select, uint32_t id_type,
                                 uint32_t id, can_callback_t new_callback);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __CAN_LIST_H */
