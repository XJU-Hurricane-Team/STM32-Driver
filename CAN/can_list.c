/**
 * @file    can_list.c
 * @author  Deadline039
 * @brief   CAN节点列表, 收到消息遍历列表, 找到对应的ID后回调
 * @version 1.0
 * @date    2024-04-11
 * @note    两个CAN各一个独立的链表
 */

#include "can_list.h"

#include <assert.h>
#include <stdlib.h>

static can_node_t *can_list_head[2] = {NULL, NULL};

/**
 * @brief 插入一个新节点
 *
 * @param can_select CAN1还是CAN2
 * @param id 节点CAN ID
 * @param id_mask 节点CAN ID掩码
 * @param node_ptr 新的节点数据
 * @param callback 节点回调函数
 */
void can_list_add_new_node(can_select_t can_select, uint32_t id,
                           uint32_t id_mask, void *node_ptr,
                           can_callback callback) {
    if (callback == NULL) {
        return;
    }

    if (can_list_find_node_by_id(can_select, id)) {
        /* ID冲突, 不予加入 */
        return;
    }

    can_node_t *new_node = (can_node_t *)malloc(sizeof(can_node_t));

#ifdef DEBUG
    assert(new_node != NULL);
#endif /* DEBUG */

    new_node->node_ptr = node_ptr;
    new_node->id = id;
    new_node->id_mask = id_mask;
    new_node->callback = callback;
    new_node->next = can_list_head[can_select];
    can_list_head[can_select] = new_node;
}

/**
 * @brief 从列表中删除一个节点
 *
 * @param can_select CAN1还是CAN2
 * @param node_ptr 要删除的节点指针
 */
void can_list_del_node_by_pointer(can_select_t can_select, void *node_ptr) {
    if (node_ptr == NULL) {
        return;
    }

    can_node_t *current_node = can_list_head[can_select];
    can_node_t *previous_node = can_list_head[can_select];

    while ((current_node != NULL) && (current_node->node_ptr != node_ptr)) {
        previous_node = current_node;
        current_node = current_node->next;
    }

    if (current_node == NULL) {
        /* 节点不存在 */
        return;
    }

    if (previous_node == current_node) {
        can_list_head[can_select] = previous_node->next;
    }

    previous_node->next = current_node->next;

    free(current_node);
}

/**
 * @brief 从列表中用ID删除一个节点
 *
 * @param can_select CAN1还是CAN2
 * @param id CAN ID
 */
void can_list_del_node_by_id(can_select_t can_select, uint32_t id) {

    can_node_t *current_node = can_list_head[can_select];
    can_node_t *previous_node = can_list_head[can_select];

    while ((current_node != NULL) && (current_node->id != id)) {
        previous_node = current_node;
        current_node = current_node->next;
    }

    if (current_node == NULL) {
        /* 节点不存在 */
        return;
    }

    if (previous_node == current_node) {
        can_list_head[can_select] = previous_node->next;
    }

    previous_node->next = current_node->next;

    free(current_node);
}

/**
 * @brief 更改CAN列表中的ID
 *
 * @param can_select CAN1还是CAN2
 * @param node 要更改的节点指针
 * @param new_id 新的ID
 * @param new_mask 新的ID掩码
 */
void can_list_change_id(can_select_t can_select, void *node, uint32_t new_id,
                        uint32_t new_mask) {
    can_node_t *current_node = can_list_head[can_select];

    while ((current_node != NULL) && current_node->node_ptr != node) {
        current_node = current_node->next;
    }

    if (current_node == NULL) {
        return;
    }

    current_node->id = new_id;
    current_node->id_mask = new_mask;
}

/**
 * @brief 更改CAN列表中的回调函数
 *
 * @param can_select CAN1还是CAN2
 * @param node 要更改的节点指针
 * @param new_callback 新的回调函数
 */
void can_list_change_callback(can_select_t can_select, void *node,
                              can_callback new_callback) {
    can_node_t *current_node = can_list_head[can_select];

    while ((current_node != NULL) && current_node->node_ptr != node) {
        current_node = current_node->next;
    }

    if (current_node == NULL) {
        return;
    }

    current_node->callback = new_callback;
}

/**
 * @brief 用ID在列表中遍历, 并返回找到的对象指针
 *
 * @param can_select CAN1还是CAN2
 * @param id 电机ID
 * @return 找到的节点
 */
can_node_t *can_list_find_node_by_id(can_select_t can_select, uint32_t id) {
    can_node_t *node = can_list_head[can_select];

    while ((node != NULL) && node->id != id) {
        node = node->next;
    }

    return node;
}

/**
 * @brief CAN回调
 *
 * @param can_select CAN1还是CAN2
 * @param can_rx_header CAN消息头
 * @param recv_msg 收到的消息
 */
void can_list_callback(can_select_t can_select,
                       CAN_RxHeaderTypeDef *can_rx_header, uint8_t *recv_msg) {
    can_node_t *node = can_list_head[can_select];
    uint32_t id;

    if (can_rx_header->IDE == CAN_ID_STD) {
        id = can_rx_header->StdId;
    } else {
        id = can_rx_header->ExtId;
    }

    while ((node != NULL) && (node->id) != (id & node->id_mask)) {
        node = node->next;
    }

    if (node == NULL) {
        return;
    }

    node->callback(node->node_ptr, can_rx_header, recv_msg);
}
