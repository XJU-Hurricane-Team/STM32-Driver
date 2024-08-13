/**
 * @file    can_list.h
 * @author  Deadline039
 * @brief   CAN节点列表, 收到消息遍历列表, 找到对应的ID后回调
 * @version 0.1
 * @date    2024-04-11
 * @note    两个CAN各一个独立的链表
 */

#ifndef __CAN_LIST_H
#define __CAN_LIST_H

#include "can.h"

/**
 * @brief CAN回调函数指针
 */
typedef void (*can_callback)(void * /* node_obj */,
                             CAN_RxHeaderTypeDef * /* can_rx_header */,
                             uint8_t * /* can_msg */);

/**
 * @brief CAN节点列表, CAN1, CAN2各一个列表
 */
typedef struct can_node {
    void *node_ptr;        /*!< CAN对象 */
    uint32_t id;           /*!< CAN ID */
    uint32_t id_mask;      /*!< CAN ID掩码, 回调时判断ID */
    can_callback callback; /*!< 回调函数指针 */
    struct can_node *next; /*!< 链表下一个节点 */
} can_node_t;

void can_list_add_new_node(can_select_t can_select, uint32_t id,
                           uint32_t id_mask, void *node_ptr,
                           can_callback callback);
void can_list_del_node_by_pointer(can_select_t can_select, void *node);
void can_list_del_node_by_id(can_select_t can_select, uint32_t id);
void can_list_change_id(can_select_t can_select, void *node, uint32_t new_id,
                        uint32_t new_mask);
void can_list_change_callback(can_select_t can_select, void *node,
                              can_callback new_callback);
can_node_t *can_list_find_node_by_id(can_select_t can_select, uint32_t id);

#endif /* __CAN_LIST_H */