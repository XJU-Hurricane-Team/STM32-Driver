/**
 * @file    can_list.c
 * @author  Deadline039
 * @brief   CAN Receive list.
 * @version 1.0
 * @date    2024-11-24
 */

#include "can_list/can_list.h"

#include <stdlib.h>

#define STD_ID_TABLE 0
#define EXT_ID_TABLE 1

#if CAN_LIST_USE_RTOS
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

static QueueHandle_t can_list_queue_handle;
static TaskHandle_t can_list_task_handle;
void can_list_polling_task(void *args);

/**
 * @brief Queue message data type.
 */
typedef struct {
#if CAN_LIST_USE_FDCAN
    FDCAN_HandleTypeDef *hcan; /*!< The handle of FDCAN.     */
#else                          /* CAN_LIST_USE_FDCAN */
    CAN_HandleTypeDef *hcan; /*!< The handle of CAN.         */
#endif                         /* CAN_LIST_USE_FDCAN */
    uint32_t rx_fifo;          /*!< The FIFO which received. */
} queue_msg_t;

static queue_msg_t send_msg_from_isr;

#endif /* CAN_LIST_USE_RTOS */

/*****************************************************************************
 * @defgroup Private type and variables.
 * @{
 */

/**
 * @brief CAN list node type.
 */
typedef struct can_node {
    void *can_data;          /*!< The CAN data of this node.    */
    uint32_t id;             /*!< CAN ID.                       */
    uint32_t id_mask;        /*!< CAN ID mask.                  */
    can_callback_t callback; /*!< CAN callback function.        */
    struct can_node *next;   /*!< Next CAN list node.           */
} can_node_t;

/**
 * @brief CAN hash table.
 */
typedef struct {
    can_node_t **table; /*!< Node pointer array of table. */
    uint32_t len;       /*!< Table size.                  */
} hash_table_t;

/**
 * @brief The CAN table struct, each ID type has an independent table.
 */
typedef struct {
    hash_table_t id_table[2]; /*!< Std and Ext ID table.   */
} can_table_t;

/* The CAN instance, each CAN has an independent table. */
can_table_t *can_table[CAN_LIST_MAX_CAN_NUMBER];

/**
 * @}
 */

/*****************************************************************************
 * @defgroup CRUD functions of CAN hash table.
 * @{
 */

/**
 * @brief Find node pointer in the specific table.
 *
 * @param table Table to search.
 * @param id The id to be search.
 * @return The node data which be found.
 */
static can_node_t *can_list_find_node_by_id(const hash_table_t *table,
                                            const uint32_t id) {
    can_node_t *node = table->table[id % table->len];

    while ((node != NULL) && node->id != id) {
        node = node->next;
    }

    return node;
}

/**
 * @brief Create a CAN table to receive and process the CAN message.
 *
 * @param can_select Specific which CAN list will be created.
 * @param std_len Standard Id table length.
 * @param ext_len Extended Id table length.
 * @return Operational status:
 * @retval - 0: Success.
 * @retval - 1: This CAN does not exist.
 * @retval - 2: This CAN had created.
 * @retval - 3: Memory allocated failed.
 */
uint8_t can_list_add_can(can_selected_t can_select, uint32_t std_len,
                         uint32_t ext_len) {
    if (can_select >= CAN_LIST_MAX_CAN_NUMBER) {
        return 1;
    }

    if (can_table[can_select] != NULL) {
        return 2;
    }

    can_table[can_select] = (can_table_t *)CAN_LIST_MALLOC(sizeof(can_table_t));
    if (can_table[can_select] == NULL) {
        return 3;
    }

    can_table[can_select]->id_table[STD_ID_TABLE].table =
        (can_node_t **)CAN_LIST_CALLOC(std_len, sizeof(can_node_t *));
    if (can_table[can_select]->id_table[STD_ID_TABLE].table == NULL) {
        CAN_LIST_FREE(can_table[can_select]);
        can_table[can_select] = NULL;
        return 3;
    }
    can_table[can_select]->id_table[STD_ID_TABLE].len = std_len;

    can_table[can_select]->id_table[EXT_ID_TABLE].table =
        (can_node_t **)CAN_LIST_CALLOC(ext_len, sizeof(can_node_t *));
    if (can_table[can_select]->id_table[EXT_ID_TABLE].table == NULL) {
        CAN_LIST_FREE(can_table[can_select]->id_table[STD_ID_TABLE].table);
        CAN_LIST_FREE(can_table[can_select]);
        can_table[can_select] = NULL;
        return 3;
    }
    can_table[can_select]->id_table[EXT_ID_TABLE].len = ext_len;

#if CAN_LIST_USE_RTOS
    if (can_list_queue_handle == NULL) {
        can_list_queue_handle =
            xQueueCreate(CAN_LIST_QUEUE_LENGTH, sizeof(queue_msg_t));
        xTaskCreate(can_list_polling_task, CAN_LIST_TASK_NAME,
                    CAN_LSIT_TASK_STK_SIZE, NULL, CAN_LIST_TASK_PRIORITY,
                    &can_list_task_handle);
    }
#endif /* CAN_LIST_USE_RTOS */

    return 0;
}

/**
 * @brief Adding a node to the CAN table.
 *
 * @param can_select Specific which CAN will be added.
 * @param node_data The data pointer of this node.
 * @param id The id of this node.
 * @param id_mask The id mask of this node.
 * @param id_type The id type of this node.
 * @param callback The callback function of this node.
 * @return Operational status:
 * @retval - 0: Success.
 * @retval - 1: This CAN does not exists.
 * @retval - 2: The specific CAN table is not created.
 * @retval - 3: Parameter invaild.
 * @retval - 4: This ID already exists in the table.
 * @retval - 5: Memroy allocated failed.
 */
uint8_t can_list_add_new_node(can_selected_t can_select, void *node_data,
                              uint32_t id, uint32_t id_mask, uint32_t id_type,
                              can_callback_t callback) {
    if (can_select >= CAN_LIST_MAX_CAN_NUMBER) {
        return 1;
    }

    if (can_table[can_select] == NULL) {
        return 2;
    }

    if (id_type == CAN_ID_STD) {
        id_type = STD_ID_TABLE;
    } else if (id_type == CAN_ID_EXT) {
        id_type = EXT_ID_TABLE;
    } else {
        return 3;
    }

    if (callback == NULL) {
        return 3;
    }

    /* Specific hash table to insert. */
    hash_table_t *table = &can_table[can_select]->id_table[id_type];

    if (can_list_find_node_by_id(table, id) != NULL) {
        return 4;
    }

    can_node_t *new_node = (can_node_t *)CAN_LIST_MALLOC(sizeof(can_node_t));
    if (new_node == NULL) {
        return 5;
    }

    new_node->can_data = node_data;
    new_node->id = id;
    new_node->id_mask = id_mask;
    new_node->callback = callback;

    /* Calculate the table index to insert. */
    can_node_t **table_head = &(table->table[id % table->len]);

    new_node->next = *table_head;
    *table_head = new_node;

    return 0;
}

/**
 * @brief Delete node by data pointer.
 *
 * @param can_select Specific which can to operate.
 * @param id_type Specific which id table to operate.
 * @param id Specific which node will be deleted.
 * @return Operational status:
 * @retval - 0: Success.
 * @retval - 1: This CAN does not exists.
 * @retval - 2: The specific CAN table is not created.
 * @retval - 3: Parameter invaild.
 * @retval - 4: Node does not exists.
 */
uint8_t can_list_del_node_by_id(can_selected_t can_select, uint32_t id_type,
                                uint32_t id) {
    if (can_select >= CAN_LIST_MAX_CAN_NUMBER) {
        return 1;
    }

    if (can_table[can_select] == NULL) {
        return 2;
    }

    if (id_type == CAN_ID_STD) {
        id_type = STD_ID_TABLE;
    } else if (id_type == CAN_ID_EXT) {
        id_type = EXT_ID_TABLE;
    } else {
        return 3;
    }

    hash_table_t *table = &can_table[can_select]->id_table[id_type];
    can_node_t **list_head = &table->table[id % table->len];

    can_node_t *current_node = *list_head;
    can_node_t *previous_node = *list_head;

    while ((current_node != NULL) && (current_node->id != id)) {
        previous_node = current_node;
        current_node = current_node->next;
    }

    if (current_node == NULL) {
        /* The node does not exist */
        return 4;
    }

    if (previous_node == current_node) {
        *list_head = previous_node->next;
    }

    previous_node->next = current_node->next;

    CAN_LIST_FREE(current_node);

    return 0;
}

/**
 * @brief Delete node by data pointer.
 *
 * @param can_select Specific which can to operate.
 * @param id_type Specific which id table to operate.
 * @param id Specific which node will be deleted.
 * @param new_callback New callback of this node to set.
 * @return Operational status:
 * @retval - 0: Success.
 * @retval - 1: This CAN does not exists.
 * @retval - 2: The specific CAN table is not created.
 * @retval - 3: Parameter invaild.
 * @retval - 4: Node does not exists.
 */
uint8_t can_list_change_callback(can_selected_t can_select, uint32_t id_type,
                                 uint32_t id, can_callback_t new_callback) {
    if (can_select >= CAN_LIST_MAX_CAN_NUMBER) {
        return 1;
    }

    if (can_table[can_select] == NULL) {
        return 2;
    }

    if (id_type == CAN_ID_STD) {
        id_type = STD_ID_TABLE;
    } else if (id_type == CAN_ID_EXT) {
        id_type = EXT_ID_TABLE;
    } else {
        return 3;
    }

    hash_table_t *table = &can_table[can_select]->id_table[id_type];

    can_node_t *node = can_list_find_node_by_id(table, id);

    if (node == NULL) {
        return 4;
    }

    node->callback = new_callback;

    return 0;
}

/*
 * @}
 */

/*****************************************************************************
 * @defgroup Process CAN message function.
 * @{
 */

#if CAN_LIST_USE_RTOS

/**
 * @brief CAN list polling task.
 *
 * @param args Start arguments.
 */
void can_list_polling_task(void *args) {
    UNUSED(args);
#if CAN_LIST_USE_FDCAN
    /* The rx header read from the CAN. */
    static FDCAN_RxHeaderTypeDef rx_header;
    /* The rx data read from the CAN. */
    static uint8_t rx_data[64];
#else  /* CAN_LIST_USE_FDCAN */
    /* The rx header read from the CAN. */
    static CAN_RxHeaderTypeDef rx_header;
    /* The rx data read from the CAN. */
    static uint8_t rx_data[8];
#endif /* CAN_LIST_USE_FDCAN */

    /* The rx header to callback function. */
    static can_rx_header_t call_rx_header;

    uint32_t id = 0x00;
    hash_table_t *table = NULL;
    can_node_t *node = NULL;

    can_selected_t can_received;

    queue_msg_t recv_msg;

    while (1) {
        xQueueReceive(can_list_queue_handle, &recv_msg, portMAX_DELAY);

#if CAN_LIST_USE_FDCAN

        switch ((uintptr_t)(recv_msg.hcan->Instance)) {
#if FDCAN1_ENABLE
            case FDCAN1_BASE: {
                can_received = can1_selected;
            } break;
#endif /* FDCAN1_ENABLE */

#if FDCAN2_ENABLE
            case FDCAN2_BASE: {
                can_received = can2_selected;
            } break;
#endif /* FDCAN2_ENABLE */

#if FDCAN3_ENABLE
            case FDCAN3_BASE: {
                can_received = can3_selected;
            } break;
#endif /* FDCAN3_ENABLE */

            default:
                continue;
        }

        if (HAL_FDCAN_GetRxMessage(recv_msg.hcan, recv_msg.rx_fifo, &rx_header,
                                   rx_data) != HAL_OK) {
            continue;
        }

        if (rx_header.IdType == FDCAN_STANDARD_ID) {
            table = &can_table[can_received]->id_table[STD_ID_TABLE];
        } else {
            table = &can_table[can_received]->id_table[EXT_ID_TABLE];
        }
        id = rx_header.Identifier;

        node = table->table[id % table->len];

        while ((node != NULL) && (node->id) != (id & node->id_mask)) {
            node = node->next;
        }

        if (node == NULL || node->callback == NULL) {
            continue;
        }

        call_rx_header.id_type = rx_header.IdType;
        call_rx_header.frame_type = rx_header.RxFrameType;
        call_rx_header.data_length = rx_header.DataLength;

#else /* CAN_LIST_USE_FDCAN */
        switch ((uintptr_t)(recv_msg.hcan->Instance)) {
#if CAN1_ENABLE
            case CAN1_BASE: {
                can_received = can1_selected;
            } break;
#endif /* CAN1_ENABLE */

#if CAN2_ENABLE
            case CAN2_BASE: {
                can_received = can2_selected;
            } break;
#endif /* CAN2_ENABLE */

#if CAN3_ENABLE
            case CAN3_BASE: {
                can_received = can3_selected;
            } break;
#endif /* CAN3_ENABLE */

            default:
                continue;
        }

        if (HAL_CAN_GetRxMessage(recv_msg.hcan, recv_msg.rx_fifo, &rx_header,
                                 rx_data) != HAL_OK) {
            continue;
        }

        /* Enable the notification. */
        if (recv_msg.rx_fifo == CAN_RX_FIFO0) {
            HAL_CAN_ActivateNotification(recv_msg.hcan,
                                         CAN_IT_RX_FIFO0_MSG_PENDING);
        } else {
            HAL_CAN_ActivateNotification(recv_msg.hcan,
                                         CAN_IT_RX_FIFO1_MSG_PENDING);
        }

        if (rx_header.IDE == CAN_ID_STD) {
            table = &can_table[can_received]->id_table[STD_ID_TABLE];
            id = rx_header.StdId;
        } else {
            table = &can_table[can_received]->id_table[EXT_ID_TABLE];
            id = rx_header.ExtId;
        }

        node = table->table[id % table->len];

        while ((node != NULL) && (node->id) != (id & node->id_mask)) {
            node = node->next;
        }

        if (node == NULL || node->callback == NULL) {
            continue;
        }

        call_rx_header.id_type = rx_header.IDE;
        call_rx_header.frame_type = rx_header.RTR;
        call_rx_header.data_length = rx_header.DLC;

#endif /* CAN_LIST_USE_FDCAN */

        call_rx_header.id = id;

        node->callback(node->can_data, &call_rx_header, rx_data);
    }
}

#else /* CAN_LIST_USE_RTOS */

#if CAN_LIST_USE_FDCAN

/**
 * @brief Process the can message and call the function by FDCAN ID.
 *
 * @param hcan The handle of FDCAN.
 * @param rx_fifo Specific which FIFO will read.
 */
static void can_message_process(FDCAN_HandleTypeDef *hcan, uint32_t rx_fifo) {

#else /* CAN_LIST_USE_FDCAN */

/**
 * @brief Process the can message and call the function by CAN ID.
 *
 * @param hcan The handle of CAN.
 * @param rx_fifo Specific which FIFO will read.
 */
static void can_message_process(CAN_HandleTypeDef *hcan, uint32_t rx_fifo) {

#endif /* CAN_LIST_USE_FDCAN */

    uint8_t can_received = CAN_LIST_MAX_CAN_NUMBER;

#if CAN_LIST_USE_FDCAN

    switch ((uintptr_t)(hcan->Instance)) {
#if FDCAN1_ENABLE
        case FDCAN1_BASE: {
            can_received = can1_selected;
        } break;
#endif /* FDCAN1_ENABLE */

#if FDCAN2_ENABLE
        case FDCAN2_BASE: {
            can_received = can2_selected;
        } break;
#endif /* FDCAN2_ENABLE */

#if FDCAN3_ENABLE
        case FDCAN3_BASE: {
            can_received = can3_selected;
        } break;
#endif /* FDCAN3_ENABLE */

        default:
            return;
    }

#else /* CAN_LIST_USE_FDCAN */

    /* Specific which CAN had received message. */
    switch ((uintptr_t)(hcan->Instance)) {
#if CAN1_ENABLE
        case CAN1_BASE: {
            can_received = can1_selected;
        } break;
#endif /* CAN1_ENABLE */

#if CAN2_ENABLE
        case CAN2_BASE: {
            can_received = can2_selected;
        } break;
#endif /* CAN2_ENABLE */

#if CAN3_ENABLE
        case CAN3_BASE: {
            can_received = can3_selected;
        } break;
#endif /* CAN3_ENABLE */

        default:
            return;
    }

#endif /* CAN_LIST_USE_FDCAN */

    /* The rx header to callback function. */
    static can_rx_header_t call_rx_header;

    /* Specific hash table will search. */
    uint32_t id;
    hash_table_t *table;

#if CAN_LIST_USE_FDCAN
    /* The rx header read from the CAN. */
    static FDCAN_RxHeaderTypeDef rx_header;
    /* The rx data read from the CAN. */
    static uint8_t rx_data[64];
    if (HAL_FDCAN_GetRxMessage(hcan, rx_fifo, &rx_header, rx_data) != HAL_OK) {
        return;
    }

    if (rx_header.IdType == FDCAN_STANDARD_ID) {
        table = &can_table[can_received]->id_table[STD_ID_TABLE];

    } else {
        table = &can_table[can_received]->id_table[EXT_ID_TABLE];
    }
    id = rx_header.Identifier;
#else  /* CAN_LIST_USE_FDCAN */
    /* The rx header read from the CAN. */
    static CAN_RxHeaderTypeDef rx_header;
    /* The rx data read from the CAN. */
    static uint8_t rx_data[8];
    if (HAL_CAN_GetRxMessage(hcan, rx_fifo, &rx_header, rx_data) != HAL_OK) {
        return;
    }

    if (rx_header.IDE == CAN_ID_STD) {
        table = &can_table[can_received]->id_table[STD_ID_TABLE];
        id = rx_header.StdId;
    } else {
        table = &can_table[can_received]->id_table[EXT_ID_TABLE];
        id = rx_header.ExtId;
    }
#endif /* CAN_LIST_USE_FDCAN */

    can_node_t *node = table->table[id % table->len];

    while ((node != NULL) && (node->id) != (id & node->id_mask)) {
        node = node->next;
    }

    if (node == NULL || node->callback == NULL) {
        return;
    }

    call_rx_header.id = id;

#if CAN_LIST_USE_FDCAN
    call_rx_header.id_type = rx_header.IdType;
    call_rx_header.frame_type = rx_header.RxFrameType;
    call_rx_header.data_length = rx_header.DataLength;
#else  /* CAN_LIST_USE_FDCAN */
    call_rx_header.id_type = rx_header.IDE;
    call_rx_header.frame_type = rx_header.RTR;
    call_rx_header.data_length = rx_header.DLC;
#endif /* CAN_LIST_USE_FDCAN */

    node->callback(node->can_data, &call_rx_header, rx_data);
}

#endif /* CAN_LIST_USE_RTOS */

/**
 * @}
 */

/*****************************************************************************
 * @defgroup Interrupt callback overload.
 * @{
 */

#if CAN_LIST_USE_FDCAN

/**
 * @brief Rx FIFO 0 callback.
 * 
 * @param hfdcan pointer to an FDCAN_HandleTypeDef structure that contains
 *        the configuration information for the specified FDCAN.
 * @param RxFifo0ITs indicates which Rx FIFO 0 interrupts are signaled.
 *        This parameter can be any combination of @arg FDCAN_Rx_Fifo0_Interrupts.
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t RxFifo0ITs) {
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == RESET) {
        return;
    }

#if CAN_LIST_USE_RTOS
    if (can_list_queue_handle == NULL) {
        return;
    }

    send_msg_from_isr.hcan = hfdcan;
    send_msg_from_isr.rx_fifo = FDCAN_RX_FIFO0;
    xQueueSendFromISR(can_list_queue_handle, &send_msg_from_isr, NULL);
#else  /* CAN_LIST_USE_RTOS */
    can_message_process(hfdcan, FDCAN_RX_FIFO0);
#endif /* CAN_LIST_USE_RTOS */
}
/**
 * @brief Rx FIFO 1 callback.
 *
 * @param hfdcan pointer to an FDCAN_HandleTypeDef structure that contains
 *        the configuration information for the specified FDCAN.
 * @param RxFifo1ITs indicates which Rx FIFO 1 interrupts are signaled.
 *        This parameter can be any combination of @arg FDCAN_Rx_Fifo1_Interrupts.
 */
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t RxFifo1ITs) {
    if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) == RESET) {
        return;
    }

#if CAN_LIST_USE_RTOS
    if (can_list_queue_handle == NULL) {
        return;
    }

    send_msg_from_isr.hcan = hfdcan;
    send_msg_from_isr.rx_fifo = FDCAN_RX_FIFO1;
    xQueueSendFromISR(can_list_queue_handle, &send_msg_from_isr, NULL);
#else  /* CAN_LIST_USE_RTOS */
    can_message_process(hfdcan, FDCAN_RX_FIFO1);
#endif /* CAN_LIST_USE_RTOS */
}

#else /* CAN_LIST_USE_FDCAN */
/**
 * @brief Rx FIFO 0 message pending callback.
 *
 * @param hcan The handle of CAN.
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
#if CAN_LIST_USE_RTOS
    if (can_list_queue_handle == NULL) {
        return;
    }

    send_msg_from_isr.hcan = hcan;
    send_msg_from_isr.rx_fifo = CAN_RX_FIFO0;
    xQueueSendFromISR(can_list_queue_handle, &send_msg_from_isr, NULL);
    HAL_CAN_DeactivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
#else  /* CAN_LIST_USE_RTOS */
    can_message_process(hcan, CAN_RX_FIFO0);
#endif /* CAN_LIST_USE_RTOS */
}

/**
 * @brief Rx FIFO 1 message pending callback.
 *
 * @param hcan The handle of CAN.
 */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
#if CAN_LIST_USE_RTOS
    if (can_list_queue_handle == NULL) {
        return;
    }

    send_msg_from_isr.hcan = hcan;
    send_msg_from_isr.rx_fifo = CAN_RX_FIFO1;
    xQueueSendFromISR(can_list_queue_handle, &send_msg_from_isr, NULL);
    HAL_CAN_DeactivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
#else  /* CAN_LIST_USE_RTOS */
    can_message_process(hcan, CAN_RX_FIFO1);
#endif /* CAN_LIST_USE_RTOS */
}

#endif /* CAN_LIST_USE_RTOS */

/**
 * @}
 */
