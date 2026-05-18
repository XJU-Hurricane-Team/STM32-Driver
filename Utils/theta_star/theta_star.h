/**
 * @file theta_star.h
 * @author PickingChip Jackrainman
 * @brief Theta* 任意角度路径规划接口
 * @version 0.1
 * @date 2026-05-04
 *
 * @note 由 a_star.h 演化而来。地图、栅格工具函数、路径压缩等通用接口
 *       与 a_star.h 保持一致命名，以便上层（chassis.c / main_ctrl.c）零侵入；
 *       仅算法接口本身改名为 theta_star_find_path_by_coord。
 */
#ifndef THETA_STAR_H
#define THETA_STAR_H

#include <stdint.h>
#include <stdbool.h>

/* ====== Theta* 算法配置 ====== */
#define THETA_MAX_NODES    500     /*!< 允许的最大节点数（受限于内部静态数组） */
#define THETA_NODE_INVALID 0xFFFFu /*!< 无效节点 ID */
#define THETA_INF_COST_F   1.0e9f  /*!< 初始无穷大代价 */

/**
 * @brief 栅格可行走判断回调函数指针
 * @param row 行坐标
 * @param col 列坐标
 * @return true 表示可通行，false 表示障碍物
 */
typedef bool (*theta_is_walkable_cb_t)(int row, int col);

/**
 * @brief Theta* 算法运行环境配置
 */
typedef struct {
    int rows;                               /*!< 地图总行数 */
    int cols;                               /*!< 地图总列数 */
    theta_is_walkable_cb_t is_walkable;     /*!< 碰撞检测回调函数 */
} theta_config_t;

int theta_star_find_path(const theta_config_t *config, int start_row, int start_col,
                         int goal_row, int goal_col, uint16_t *path_out, int max_len);

#endif /* THETA_STAR_H */
