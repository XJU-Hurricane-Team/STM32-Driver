/**
 * @file a_star.h
 * @author PickingChip
 * @brief
 * @version 0.2
 * @date 2026-04-27
 */
#ifndef A_STAR_H
#define A_STAR_H

#include <stdint.h>
#include <stdbool.h>

/* A* 算法配置 */
#define ASTAR_MAX_NODES    500     /*!< 允许的最大地图栅格数量 */
#define ASTAR_NODE_INVALID 0xFFFFu /*!< 无效节点 ID */
#define ASTAR_INF_COST     0xFFFFu /*!< 初始无穷大代价 */
#define TURN_PENALTY       3       /* 拐弯额外代价，值越大路径越直(建议2-5) */

/**
 * @brief 栅格可行走判断回调函数指针
 * @param row 行坐标
 * @param col 列坐标
 * @return true 表示可通行，false 表示障碍物
 */
typedef bool (*astar_is_walkable_cb_t)(int row, int col);

/**
 * @brief A* 算法运行环境配置
 */
typedef struct {
    int rows;                           /*!< 地图总行数 */
    int cols;                           /*!< 地图总列数 */
    astar_is_walkable_cb_t is_walkable; /*!< 碰撞检测回调函数 */
} astar_config_t;

int astar_find_path(const astar_config_t *config, int start_row, int start_col,
                    int goal_row, int goal_col, uint16_t *path_out,
                    int max_len);

#endif /* A_STAR_H */
