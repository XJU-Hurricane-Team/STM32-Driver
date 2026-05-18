/**
 * @file a_star.c
 * @author PickingChip
 * @brief a_star 算法
 * @version 0.2
 * @date 2026-04-27
 *
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "a_star.h"

typedef struct {
    uint16_t id;    /* 节点索引 id（0~MAX_NODES-1） */
    uint16_t f;     /* 对应节点的 f 值 */
} OpenNode;

static uint16_t came_from[ASTAR_MAX_NODES];   /* 到达节点 n 的前驱节点 id */
static uint16_t g_score[ASTAR_MAX_NODES];     /* 起点到节点 n 的当前最小已知代价 */
static uint16_t f_score[ASTAR_MAX_NODES];     /* f_score[n] = g_score[n] + h(n) */
static uint8_t closed_set[ASTAR_MAX_NODES];   /* 节点完成扩展标志 */
static OpenNode open_list[ASTAR_MAX_NODES];   /* 开放表（二叉小顶堆） */
static int16_t open_pos[ASTAR_MAX_NODES];     /* 节点在堆中的位置 */
static int open_count;                  /* 开放表当前元素个数。*/


/* 保存当前环境配置，避免在各静态函数中反复传递 */
static const astar_config_t *curr_cfg = NULL;

/**
 * @brief 计算移动方向，用于拐弯惩罚判断。
 * @note 利用 id 差值直接判断方向，无需计算行列坐标，效率最高。
 * @param from 起点节点id
 * @param to 终点节点id
 * @return 移动方向: 0=上, 1=下, 2=左, 3=右, 4=非法
 */
static inline uint8_t get_move_dir(uint16_t from, uint16_t to) {
    int16_t diff = (int16_t)to - (int16_t)from;
    if (diff == -(int16_t)curr_cfg->cols) return 0;
    if (diff == (int16_t)curr_cfg->cols)  return 1;
    if (diff == -1)                 return 2;  /* 左: id 减少 1 */
    if (diff == 1)                  return 3;  /* 右: id 增加 1 */
    return 4;  /* 应该不会发生 */
}

/**
 * @brief 计算从当前节点移动到邻居节点的步进代价。
 * @note 直行=1，转弯=1+TURN_PENALTY，从起点出发无惩罚。
 * @param current 当前节点id
 * @param nb 邻居节点id
 * @return 计算得到的步进代价
 */
static inline uint16_t calc_step_cost(uint16_t current, uint16_t nb) {
    uint16_t prev = came_from[current];
    if (prev == ASTAR_NODE_INVALID) {
        return 1;  /* 起点，无转弯 */
    }
    uint8_t prev_dir = get_move_dir(prev, current);
    uint8_t curr_dir = get_move_dir(current, nb);
    return (prev_dir == curr_dir) ? 1 : (uint16_t)(1 + TURN_PENALTY);
}

/**
 * @brief 启发函数 h(n)：计算到终点的曼哈顿距离。
 * @note 由于本实现是4邻接（上下左右，每步代价=1），
 *       曼哈顿距离满足可采纳性，A* 能得到最短路径。
 * @param id 当前节点id
 * @param goal_row 终点行坐标
 * @param goal_col 终点列坐标
 * @return 预估的最小代价 (曼哈顿距离)
 */
static uint16_t heuristic_to_goal(uint16_t id, int goal_row, int goal_col) {
    int r = (int)(id / curr_cfg->cols);
    int c = (int)(id % curr_cfg->cols);
    int dr = r - goal_row;
    int dc = c - goal_col;
    if (dr < 0) dr = -dr;
    if (dc < 0) dc = -dc;
    return (uint16_t)(dr + dc);
}

/**
 * @brief 交换开放表(二叉堆)中的两个节点位置
 * @param i 第一个节点的堆索引
 * @param j 第二个节点的堆索引
 */
static void open_swap(int i, int j) {
    OpenNode tmp = open_list[i];
    open_list[i] = open_list[j];
    open_list[j] = tmp;
    open_pos[open_list[i].id] = (int16_t)i;
    open_pos[open_list[j].id] = (int16_t)j;
}

/**
 * @brief 开放表(二叉小顶堆)向上调整操作
 * @param idx 需要调整的节点索引
 */
static void open_sift_up(int idx) {
    while (idx > 0) {
        int parent = (idx - 1) / 2;
        if (open_list[parent].f <= open_list[idx].f) {
            break;
        }
        open_swap(parent, idx);
        idx = parent;
    }
}

/**
 * @brief 开放表(二叉小顶堆)向下调整操作
 * @param idx 需要调整的节点索引
 */
static void open_sift_down(int idx) {
    for (;;) {
        int left = idx * 2 + 1;
        int right = left + 1;
        int smallest = idx;

        if (left < open_count && open_list[left].f < open_list[smallest].f) {
            smallest = left;
        }
        if (right < open_count && open_list[right].f < open_list[smallest].f) {
            smallest = right;
        }
        if (smallest == idx) {
            break;
        }

        open_swap(idx, smallest);
        idx = smallest;
    }
}

/**
 * @brief 将新节点压入开放表，或更新已有节点的f值
 * @param id 节点id
 * @param f 节点的综合代价f值
 */
static void open_push_or_update(uint16_t id, uint16_t f) {
    int idx = open_pos[id];

    if (idx >= 0) {
        if (f < open_list[idx].f) {
            open_list[idx].f = f;
            open_sift_up(idx);
        }
        return;
    }

    if (open_count < ASTAR_MAX_NODES) {
        int insert_idx = open_count;
        open_list[insert_idx].id = id;
        open_list[insert_idx].f = f;
        open_pos[id] = (int16_t)insert_idx;
        open_count++;
        open_sift_up(insert_idx);
    }
}

/**
 * @brief 从开放表中弹出f值最小的节点
 * @return f值最小的节点id，若开放表为空则返回 NODE_INVALID
 */
static uint16_t open_pop_min(void) {
    if (open_count <= 0) {
        return ASTAR_NODE_INVALID;
    }

    uint16_t id = open_list[0].id;
    open_pos[id] = -1;

    open_count--;
    if (open_count > 0) {
        open_list[0] = open_list[open_count];
        open_pos[open_list[0].id] = 0;
        open_sift_down(0);
    }

    return id;
}

/**
 * @brief 获取当前节点周围的4个正交邻居节点(上下左右)
 * @param id 当前节点id
 * @param neighbors 用于存储合法邻居节点id的数组
 * @return 合法邻居的数量
 */
static int get_neighbors4(uint16_t id, uint16_t neighbors[4]) {
    int row = (int)(id / curr_cfg->cols);
    int col = (int)(id % curr_cfg->cols);
    int count = 0;

    if (row > 0 && curr_cfg->is_walkable(row - 1, col)) {
        neighbors[count++] = (uint16_t)(id - curr_cfg->cols);
    }
    if (row + 1 < curr_cfg->rows && curr_cfg->is_walkable(row + 1, col)) {
        neighbors[count++] = (uint16_t)(id + curr_cfg->cols);
    }
    if (col > 0 && curr_cfg->is_walkable(row, col - 1)) {
        neighbors[count++] = (uint16_t)(id - 1u);
    }
    if (col + 1 < curr_cfg->cols && curr_cfg->is_walkable(row, col + 1)) {
        neighbors[count++] = (uint16_t)(id + 1u);
    }

    return count;
}

/**
 * @brief 回溯并生成正向路径
 * @note 从 goal 沿 came_from 反向回溯到 start，再反转得到正向路径。
 * @param start_id 起点节点id
 * @param goal_id 终点节点id
 * @param path_out 输出的正向路径数组
 * @param max_len 最大允许的路径长度
 * @return 生成的路径长度，若失败则返回0
 */
static int reconstruct_path(uint16_t start_id, uint16_t goal_id, uint16_t *path_out, int max_len) {
    uint16_t rev_path[ASTAR_MAX_NODES];
    int rev_len = 0;
    uint16_t node = goal_id;

    while (node != ASTAR_NODE_INVALID && rev_len < ASTAR_MAX_NODES) {
        rev_path[rev_len++] = node;
        if (node == start_id) {
            break;
        }
        node = came_from[node];
    }

    if (rev_len == 0 || rev_path[rev_len - 1] != start_id) {
        return 0;
    }
    if (rev_len > max_len) {
        return 0;
    }

    for (int i = 0; i < rev_len; i++) {
        path_out[i] = rev_path[rev_len - 1 - i];
    }

    return rev_len;
}


/**
 * @brief A* 寻路主函数
 *
 * @param config A*环境配置
 * @param start_row 起点行
 * @param start_col 起点列
 * @param goal_row 终点行
 * @param goal_col 终点列
 * @param path_out 输出路径数组，存储路径上节点的 id 序列
 * @param max_len 输出路径数组的最大长度
 *
 * @note  输入起终点格子坐标，返回路径上的格子编号序列。
 *        编号规则: id = row * MAP_COLS + col (从0开始)
 *        返回值: >0 路径长度, 0 无路径或输入非法。
 */
int astar_find_path(const astar_config_t *config, int start_row, int start_col,
                    int goal_row, int goal_col, uint16_t *path_out, int max_len) {
    /* 参数合法性检查 */
    if (path_out == NULL || max_len <= 0 || config == NULL || config->is_walkable == NULL) {
        return 0;
    }
    if (config->rows * config->cols > ASTAR_MAX_NODES) {
        return 0;
    }
    if (start_row < 0 || start_row >= config->rows || start_col < 0 || start_col >= config->cols ||
        goal_row < 0 || goal_row >= config->rows || goal_col < 0 || goal_col >= config->cols) {
        return 0;
    }

    curr_cfg = config;

    uint16_t start_id = (uint16_t)(start_row * config->cols + start_col);
    uint16_t goal_id = (uint16_t)(goal_row * config->cols + goal_col);

    if (start_id == goal_id) {
        path_out[0] = start_id;
        return 1;
    }

    /* 起点或终点落在障碍上，直接判失败 */
    if (!config->is_walkable(start_row, start_col) || !config->is_walkable(goal_row, goal_col)) {
        return 0;
    }

    /* 每次规划前清空状态数组 */
    for (int i = 0; i < ASTAR_MAX_NODES; i++) {
        came_from[i] = ASTAR_NODE_INVALID;
        g_score[i] = ASTAR_INF_COST;
        f_score[i] = ASTAR_INF_COST;
        closed_set[i] = 0;
        open_pos[i] = -1;
    }

    open_count = 0;
    g_score[start_id] = 0;
    f_score[start_id] = heuristic_to_goal(start_id, goal_row, goal_col);
    open_push_or_update(start_id, f_score[start_id]);

    /* A* 主循环 */
    while (open_count > 0) {
        uint16_t current = open_pop_min();
        if (current == ASTAR_NODE_INVALID) {
            break;
        }

        /* 到达终点，回溯路径并返回 */
        if (current == goal_id) {
            return reconstruct_path(start_id, goal_id, path_out, max_len);
        }

        if (closed_set[current]) {
            continue;
        }
        closed_set[current] = 1;

        uint16_t neighbors[4];
        int nb_count = get_neighbors4(current, neighbors);
        for (int i = 0; i < nb_count; i++) {
            uint16_t nb = neighbors[i];
            if (closed_set[nb]) {
                continue;
            }

            /* 计算步进代价：直行=1，转弯=1+TURN_PENALTY */
            uint16_t step_cost = calc_step_cost(current, nb);
            uint16_t tentative_g = (uint16_t)(g_score[current] + step_cost);
            if (tentative_g < g_score[nb]) {
                /* 找到更优路径，更新父节点和代价 */
                came_from[nb] = current;
                g_score[nb] = tentative_g;
                f_score[nb] = (uint16_t)(tentative_g + heuristic_to_goal(nb, goal_row, goal_col));
                open_push_or_update(nb, f_score[nb]);
            }
        }
    }

    return 0;
}

#if 0
/**
 * @brief 主函数，用于pc调试演示
 *
 */

int main(void) {

    int start_row, start_col;
    int goal_row, goal_col;
    uint16_t path[MAX_NODES];

    printf("Map size: %d x %d\n", MAP_ROWS, MAP_COLS);
    printf("Input start coordinate row col (row:0~%d, col:0~%d): ", MAP_ROWS - 1, MAP_COLS - 1);
    if (scanf("%d %d", &start_row, &start_col) != 2) {
        printf("Input error\n");
        return 1;
    }

    printf("Input goal coordinate row col (row:0~%d, col:0~%d): ", MAP_ROWS - 1, MAP_COLS - 1);
    if (scanf("%d %d", &goal_row, &goal_col) != 2) {
        printf("Input error\n");
        return 1;
    }

    if (!is_valid_rc(start_row, start_col) || !is_valid_rc(goal_row, goal_col)) {
        printf("Coordinate out of range\n");
        return 1;
    }

    int len = astar_find_path_by_coord(start_row, start_col,
                                       goal_row, goal_col,
                                       path, MAX_NODES);
    if (len <= 0) {
        printf("No path or invalid input\n");
        return 0;
    }

    path_process_print_map(path, len);
    printf("Path node id sequence: ");
    for (int i = 0; i < len; i++) {
        printf("%u", (unsigned int)path[i]);
        if (i + 1 < len) {
            printf(" -> ");
        }
    }
    printf("\n");

    printf("Coordinate sequence: ");
    for (int i = 0; i < len; i++) {
        int r = id_to_row(path[i]);
        int c = id_to_col(path[i]);
        printf("(%d,%d)", r, c);
        if (i + 1 < len) {
            printf(" -> ");
        }
    }
    printf("\n");

    print_map_with_path(path, len);

    return 0;
}
#endif /* 0 */
