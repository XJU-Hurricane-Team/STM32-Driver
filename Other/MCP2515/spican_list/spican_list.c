/**
 * @file    spican_list.c
 * @author  Deadline039,Dominate0017
 * @brief   SPICAN Receive list.
 * @version 1.0
 * @date    2026-3-15
 */

#include "spican_list/spican_list.h"
#include <stdlib.h>

#define STD_ID_TABLE 0
#define EXT_ID_TABLE 1

#if SPICAN_LIST_USE_RTOS
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

static QueueHandle_t spican_list_queue_handle;
static TaskHandle_t spican_list_task_handle;
void spican_list_polling_task(void *args);


static spican_selected_t mcp2515_get_spican_by_devid(MCP2515_DevId dev_id);
/**
 * @brief 队列消息数据类型（扩展：增加MCP2515设备ID）
 */
typedef struct {
    uint8_t is_mcp2515;        /*!< 是否为MCP2515消息：0=传统CAN，1=MCP2515 */
    MCP2515_DevId dev_id;      /*!< MCP2515设备ID（核心扩展） */
    GPIO_TypeDef *int_port;    /*!< MCP2515 INT引脚端口 */
    uint16_t int_pin;          /*!< MCP2515 INT引脚 */
} queue_msg_t;

static queue_msg_t send_msg_from_isr;

static const struct {
    MCP2515_DevId dev_id;
    GPIO_TypeDef *int_port;
    uint16_t int_pin;
    IRQn_Type irq_num;
} mcp2515_int_map[] = {
#if MCP2515_DEV1_ENABLE
    {MCP2515_DEV_1, MCP2515_DEV1_INT_PORT, MCP2515_DEV1_INT_PIN, MCP2515_DEV1_IRQn},
#endif /* MCP2515_DEV1_ENABLE */
#if MCP2515_DEV2_ENABLE
    {MCP2515_DEV_2, MCP2515_DEV2_INT_PORT, MCP2515_DEV2_INT_PIN, MCP2515_DEV2_IRQn},
#endif /* MCP2515_DEV2_ENABLE */
#if MCP2515_DEV3_ENABLE
    {MCP2515_DEV_3, MCP2515_DEV3_INT_PORT, MCP2515_DEV3_INT_PIN, MCP2515_DEV3_IRQn},
#endif /* MCP2515_DEV3_ENABLE */
}; 
#define MCP2515_INT_MAP_CNT (sizeof(mcp2515_int_map)/sizeof(mcp2515_int_map[0]))

#endif /* SPICAN_LIST_USE_RTOS */

/*****************************************************************************
 * @defgroup Private type and variables.
 * @{
 */

/**
 * @brief CAN list node type.
 */
typedef struct spican_node {
    void *spican_data;          /*!< The CAN data of this node.    */
    uint32_t id;             /*!< CAN ID.                       */
    uint32_t id_mask;        /*!< CAN ID mask.                  */
    spican_callback_t callback; /*!< CAN callback function.        */
    struct spican_node *next;   /*!< Next CAN list node.           */
} spican_node_t;

/**
 * @brief CAN hash table.
 */
typedef struct {
    spican_node_t **table; /*!< Node pointer array of table. */
    uint32_t len;       /*!< Table size.                  */
} hash_table_t;

/**
 * @brief The CAN table struct, each ID type has an independent table.
 */
typedef struct {
    hash_table_t id_table[2]; /*!< Std and Ext ID table.   */
} spican_table_t;

/* The CAN instance, each CAN has an independent table. */
spican_table_t *spican_table[SPICAN_LIST_MAX_CAN_NUMBER];

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
static spican_node_t *spican_list_find_node_by_id(const hash_table_t *table,
                                            const uint32_t id) {
    spican_node_t *node = table->table[id % table->len];

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
uint8_t spican_list_add_can(spican_selected_t spican_select, uint32_t std_len,
                         uint32_t ext_len) {
    if (spican_select >= SPICAN_LIST_MAX_CAN_NUMBER) {
        return 1;
    }

    if (spican_table[spican_select] != NULL) {
        return 2;
    }

    spican_table[spican_select] = (spican_table_t *)SPICAN_LIST_MALLOC(sizeof(spican_table_t));
    if (spican_table[spican_select] == NULL) {
        return 3;
    }

    spican_table[spican_select]->id_table[STD_ID_TABLE].table =
        (spican_node_t **)SPICAN_LIST_CALLOC(std_len, sizeof(spican_node_t *));
    if (spican_table[spican_select]->id_table[STD_ID_TABLE].table == NULL) {
        SPICAN_LIST_FREE(spican_table[spican_select]);
        spican_table[spican_select] = NULL;
        return 3;
    }
    spican_table[spican_select]->id_table[STD_ID_TABLE].len = std_len;

    spican_table[spican_select]->id_table[EXT_ID_TABLE].table =
        (spican_node_t **)SPICAN_LIST_CALLOC(ext_len, sizeof(spican_node_t *));
    if (spican_table[spican_select]->id_table[EXT_ID_TABLE].table == NULL) {
        SPICAN_LIST_FREE(spican_table[spican_select]->id_table[STD_ID_TABLE].table);
        SPICAN_LIST_FREE(spican_table[spican_select]);
        spican_table[spican_select] = NULL;
        return 3;
    }
    spican_table[spican_select]->id_table[EXT_ID_TABLE].len = ext_len;

#if SPICAN_LIST_USE_RTOS
    if (spican_list_queue_handle == NULL) {
        spican_list_queue_handle =
            xQueueCreate(SPICAN_LIST_QUEUE_LENGTH, sizeof(queue_msg_t));
        CANSPI_Initialize_Ext(MCP2515_DEV_1);
        xTaskCreate(spican_list_polling_task, SPICAN_LIST_TASK_NAME,
                    SPICAN_LSIT_TASK_STK_SIZE, NULL, SPICAN_LIST_TASK_PRIORITY,
                    &spican_list_task_handle);
    }
    
#endif /* SPICAN_LIST_USE_RTOS */
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
uint8_t spican_list_add_new_node(spican_selected_t spican_select, void *node_data,
                              uint32_t id, uint32_t id_mask, uint32_t id_type,
                              spican_callback_t callback) {
    if (spican_select >= SPICAN_LIST_MAX_CAN_NUMBER){
        return 1;
    }
    

    if (spican_table[spican_select] == NULL) {
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
    hash_table_t *table = &spican_table[spican_select]->id_table[id_type];

    if (spican_list_find_node_by_id(table, id) != NULL) {
        return 4;
    }

    spican_node_t *new_node = (spican_node_t *)SPICAN_LIST_MALLOC(sizeof(spican_node_t));
    if (new_node == NULL) {
        return 5;
    }

    new_node->spican_data = node_data;
    new_node->id = id;
    new_node->id_mask = id_mask;
    new_node->callback = callback;

    /* Calculate the table index to insert. */
    spican_node_t **table_head = &(table->table[id % table->len]);

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
uint8_t spican_list_del_node_by_id(spican_selected_t spican_select, uint32_t id_type,
                                uint32_t id) {
    if (spican_select >= SPICAN_LIST_MAX_CAN_NUMBER) {
        return 1;
    }

    if (spican_table[spican_select] == NULL) {
        return 2;
    }

    if (id_type == CAN_ID_STD) {
        id_type = STD_ID_TABLE;
    } else if (id_type == CAN_ID_EXT) {
        id_type = EXT_ID_TABLE;
    } else {
        return 3;
    }

    hash_table_t *table = &spican_table[spican_select]->id_table[id_type];
    spican_node_t **list_head = &table->table[id % table->len];

    spican_node_t *current_node = *list_head;
    spican_node_t *previous_node = *list_head;

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

    SPICAN_LIST_FREE(current_node);

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
uint8_t spican_list_change_callback(spican_selected_t spican_select, uint32_t id_type,
                                 uint32_t id, spican_callback_t new_callback) {
    if (spican_select >= SPICAN_LIST_MAX_CAN_NUMBER) {
        return 1;
    }

    if (spican_table[spican_select] == NULL) {
        return 2;
    }

    if (id_type == CAN_ID_STD) {
        id_type = STD_ID_TABLE;
    } else if (id_type == CAN_ID_EXT) {
        id_type = EXT_ID_TABLE;
    } else {
        return 3;
    }

    hash_table_t *table = &spican_table[spican_select]->id_table[id_type];

    spican_node_t *node = spican_list_find_node_by_id(table, id);

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

#if SPICAN_LIST_USE_RTOS

/**
 * @brief CAN list polling task.
 *
 * @param args Start arguments.
 */
void spican_list_polling_task(void *args) {
    UNUSED(args);
    
    queue_msg_t recv_msg;

    while (1) {
        xQueueReceive(spican_list_queue_handle, &recv_msg, portMAX_DELAY);
        if (recv_msg.is_mcp2515) {
            mcp2515_process_msg(recv_msg.dev_id);
        }
    }
}

#endif /* SPICAN_LIST_USE_RTOS */

#if MCP2515_INT_USE

void EXTI15_10_IRQHandler(void)
{
#if MCP2515_DEV1_ENABLE
    HAL_GPIO_EXTI_IRQHandler(MCP2515_DEV1_INT_PIN);
#endif /* MCP2515_DEV1_ENABLE */
#if MCP2515_DEV2_ENABLE
    HAL_GPIO_EXTI_IRQHandler(MCP2515_DEV2_INT_PIN);
#endif /* MCP2515_DEV2_ENABLE */
#if MCP2515_DEV3_ENABLE
    HAL_GPIO_EXTI_IRQHandler(MCP2515_DEV3_INT_PIN);
#endif /* MCP2515_DEV3_ENABLE */
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
#if SPICAN_LIST_USE_RTOS

    for(uint8_t i=0; i<MCP2515_INT_MAP_CNT; i++)
    {
        if(GPIO_Pin == mcp2515_int_map[i].int_pin)
        {
            send_msg_from_isr.is_mcp2515 = 1;
            send_msg_from_isr.dev_id = mcp2515_int_map[i].dev_id;
            send_msg_from_isr.int_port = mcp2515_int_map[i].int_port;
            send_msg_from_isr.int_pin = mcp2515_int_map[i].int_pin;
            
            xQueueSendFromISR(spican_list_queue_handle, &send_msg_from_isr, NULL);
            break;
        }
    }

    LED0_TOGGLE();
#endif
}

#endif


void mcp2515_process_msg(MCP2515_DevId dev_id) {
    uCAN_MSG can_msg;
    uint8_t rx_data[8] = {0}; 
    spican_rx_header_t call_rx_header = {0};
    hash_table_t *table = NULL;
    spican_node_t *node = NULL;
    uint32_t id = 0;
    spican_selected_t spican_dev = mcp2515_get_spican_by_devid(dev_id);

    if (spican_dev >= SPICAN_LIST_MAX_CAN_NUMBER) return;

    if (spican_table[spican_dev] == NULL) return;

        while(CANSPI_Receive_Ext(dev_id, &can_msg) == 1) {
        rx_data[0] = can_msg.frame.data0;
        rx_data[1] = can_msg.frame.data1;
        rx_data[2] = can_msg.frame.data2;
        rx_data[3] = can_msg.frame.data3;
        rx_data[4] = can_msg.frame.data4;
        rx_data[5] = can_msg.frame.data5;
        rx_data[6] = can_msg.frame.data6;
        rx_data[7] = can_msg.frame.data7;

        call_rx_header.data_length = can_msg.frame.dlc;
        call_rx_header.frame_type = CAN_RTR_DATA;

        if (can_msg.frame.idType == dSTANDARD_CAN_MSG_ID_2_0B) {
            call_rx_header.id_type = CAN_ID_STD;
            table = &spican_table[spican_dev]->id_table[STD_ID_TABLE];
            id = can_msg.frame.id;
        } else if (can_msg.frame.idType == dEXTENDED_CAN_MSG_ID_2_0B) {
            call_rx_header.id_type = CAN_ID_EXT;
            table = &spican_table[spican_dev]->id_table[EXT_ID_TABLE];
            id = can_msg.frame.id;
        } 
        call_rx_header.id = id;

        node = table->table[id % table->len];
        while ((node != NULL) && ((node->id & node->id_mask) != id)) {
            node = node->next;
        }

        if (node != NULL && node->callback != NULL) {
            node->callback(node->spican_data, &call_rx_header, rx_data);
        }
        MCP2515_ClearIntFlag_Ext(MCP2515_DEV_1, 0xFF);
    }
}

static spican_selected_t mcp2515_get_spican_by_devid(MCP2515_DevId dev_id) {
    // MCP2515_DevId -> spican_selected_t 映射
    switch (dev_id) {
        case MCP2515_DEV_1: return spican1_selected;
        case MCP2515_DEV_2: return spican2_selected;
        case MCP2515_DEV_3: return spican3_selected;
        default: return 0;
    }
}


/**
 * @}
 */
