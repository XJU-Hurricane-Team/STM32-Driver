/**
 * @file a_star.h
 * @author PickingChip
 * @brief 
 * @version 0.1
 * @date 2026-04-27
 */
#ifndef A_STAR_H
#define A_STAR_H

#include <stdint.h>

#define MAP_ROWS     30                    /* 地图行数 */
#define MAP_COLS     15                    /* 地图列数 */
#define MAX_NODES    (MAP_ROWS * MAP_COLS) /* 地图中节点的最大数量 */
#define INF_COST     0xFFFFu /* 无穷大代价，用于初始化 g_score 和 f_score */
#define NODE_INVALID 0xFFFFu /* 无效节点 id，用于初始化和错误返回 */


int is_valid_rc(uint16_t row, uint16_t col);
int is_walkable_id(uint16_t id);
uint16_t rc_to_id(uint16_t row, uint16_t col);
uint16_t id_to_row(uint16_t id);
uint16_t id_to_col(uint16_t id);

void print_map_with_path(const uint16_t *path, int path_len);
int astar_find_path_by_coord(uint16_t start_row, uint16_t start_col, uint16_t goal_row,
                             uint16_t goal_col, uint16_t *path_out, int max_len);

#endif /* A_STAR_H */