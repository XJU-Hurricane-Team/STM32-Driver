/**
 * @file theta_star.c
 * @author PickingChip Jackrainman
 * @brief Theta* 任意角度路径规划，基于 a_star.c 演化
 * @version 0.1
 * @date 2026-05-04
 *
 * @note Theta* = A* + line-of-sight 平滑：当邻居 s' 与当前节点 s 的父节点
 *       parent(s) 之间存在直线无障碍通道时，跳过 s 直接令 parent(s')=parent(s)，
 *       从而把多段折线折叠为一条任意角度的直线。
 *       与原 a_star.c 相比：
 *         - 4 邻接 -> 8 邻接（含对角线，禁止对角穿墙）
 *         - 曼哈顿启发 -> 欧氏启发
 *         - 单位代价 1 -> 欧氏代价（1.0 / sqrt(2)）
 *         - 整型 g/f -> float g/f
 *         - 邻居展开后增加 LOS 检查，决定走 Path-2 还是 Path-1
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "theta_star.h"

typedef struct {
    uint16_t id;    /* 节点索引 id（0~MAX_NODES-1） */
    float    f;     /* 对应节点的 f 值，float 以匹配欧氏代价 */
} OpenNode;

static uint16_t came_from[THETA_MAX_NODES];   /* 到达节点 n 的前驱节点 id */
static float    g_score[THETA_MAX_NODES];     /* 起点到节点 n 的当前最小已知代价 */
static float    f_score[THETA_MAX_NODES];     /* f_score[n] = g_score[n] + heuristic(n, goal) */
static uint8_t  closed_set[THETA_MAX_NODES];  /* 节点 n 已完成扩展的标志 */
static OpenNode open_list[THETA_MAX_NODES];   /* 开放表（二叉小顶堆） */
static int16_t  open_pos[THETA_MAX_NODES];    /* 节点在堆中的位置 */
static int      open_count;             /* 开放表当前元素个数。 */

/* 保存当前环境配置，避免在各静态函数中反复传递 */
static const theta_config_t *curr_cfg = NULL;

/**
 * @brief 启发函数 h(n)：计算到终点的欧氏距离。
 * @note 边代价也是欧氏距离 -> heuristic 满足可采纳性与一致性，
 *       Theta* 收敛到「LOS 最优」的近似最短路径。
 * @param id 当前节点id
 * @param goal_row 终点行坐标
 * @param goal_col 终点列坐标
 * @return 预估的欧氏距离代价
 */
static float heuristic_euclidean(uint16_t id, int goal_row, int goal_col) {
    int r = (int)(id / curr_cfg->cols);
    int c = (int)(id % curr_cfg->cols);
    float dr = (float)(r - (int)goal_row);
    float dc = (float)(c - (int)goal_col);
    return sqrtf(dr * dr + dc * dc);
}

/**
 * @brief 计算两个相邻节点之间的欧氏边代价
 * @note 用作 8 邻接边代价 / Path-2 跳跃代价。
 * @param a_id 节点A的id
 * @param b_id 节点B的id
 * @return 两点间的欧氏距离
 */
static float edge_cost(uint16_t a_id, uint16_t b_id) {
    int dr = (int)(a_id / curr_cfg->cols) - (int)(b_id / curr_cfg->cols);
    int dc = (int)(a_id % curr_cfg->cols) - (int)(b_id % curr_cfg->cols);
    return sqrtf((float)(dr * dr + dc * dc));
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
 * @param f 节点的综合代价f值 (float类型)
 */
static void open_push_or_update(uint16_t id, float f) {
    int idx = open_pos[id];

    if (idx >= 0) {
        if (f < open_list[idx].f) {
            open_list[idx].f = f;
            open_sift_up(idx);
        }
        return;
    }

    if (open_count < THETA_MAX_NODES) {
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
        return THETA_NODE_INVALID;
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
 * @brief 获取当前节点周围的8个相邻节点
 * @note 含 4 条对角线，禁止对角穿墙（两条正交相邻格至少有一格是障碍则不展开对角邻居）
 * @param id 当前节点id
 * @param neighbors 用于存储合法邻居节点id的数组
 * @return 合法邻居的数量
 */
static int get_neighbors8(uint16_t id, uint16_t neighbors[8]) {
    int row = (int)(id / curr_cfg->cols);
    int col = (int)(id % curr_cfg->cols);
    static const int8_t dr_tab[8] = {-1, -1, -1,  0, 0,  1, 1, 1};
    static const int8_t dc_tab[8] = {-1,  0,  1, -1, 1, -1, 0, 1};
    int count = 0;

    for (int i = 0; i < 8; i++) {
        int nr = row + dr_tab[i];
        int nc = col + dc_tab[i];
        if (nr < 0 || nr >= curr_cfg->rows || nc < 0 || nc >= curr_cfg->cols || !curr_cfg->is_walkable(nr, nc)) {
            continue;
        }
        /* 对角邻居：要求两条正交方向相邻格都可走，避免从夹角穿墙 */
        if (dr_tab[i] != 0 && dc_tab[i] != 0) {
            if (!curr_cfg->is_walkable(row + dr_tab[i], col)) {
                continue;
            }
            if (!curr_cfg->is_walkable(row, col + dc_tab[i])) {
                continue;
            }
        }
        neighbors[count++] = (uint16_t)(nr * curr_cfg->cols + nc);
    }
    return count;
}

/**
 * @brief Bresenham 视距(Line-of-Sight)检查
 * @note 从 a_id 到 b_id 的整数直线沿途每一格都要可走。
 *       对角步增量发生时，需额外要求两条正交相邻格都可走（与 get_neighbors8 的对角穿墙策略保持一致）。
 * @param a_id 起点节点id
 * @param b_id 终点节点id
 * @return 1 表示存在直线无障碍通道，0 表示路径被障碍阻断
 */
static int has_line_of_sight(uint16_t a_id, uint16_t b_id) {
    int r0 = (int)(a_id / curr_cfg->cols);
    int c0 = (int)(a_id % curr_cfg->cols);
    int r1 = (int)(b_id / curr_cfg->cols);
    int c1 = (int)(b_id % curr_cfg->cols);

    int dr = (r1 > r0) ? (r1 - r0) : (r0 - r1);
    int dc = (c1 > c0) ? (c1 - c0) : (c0 - c1);
    int sr = (r0 < r1) ? 1 : -1;
    int sc = (c0 < c1) ? 1 : -1;
    int err = dr - dc;
    int r = r0;
    int c = c0;

    for (;;) {
        if (r < 0 || r >= curr_cfg->rows || c < 0 || c >= curr_cfg->cols || !curr_cfg->is_walkable(r, c)) {
            return 0;
        }
        if (r == r1 && c == c1) {
            return 1;
        }

        int e2 = 2 * err;
        int step_r = 0;
        int step_c = 0;
        if (e2 > -dc) { err -= dc; r += sr; step_r = 1; }
        if (e2 <  dr) { err += dr; c += sc; step_c = 1; }

        /* 对角增量：前一格的两条正交邻格都要可走，否则视为穿墙阻断 */
        if (step_r && step_c) {
            if (!curr_cfg->is_walkable(r - sr, c)) {
                return 0;
            }
            if (!curr_cfg->is_walkable(r, c - sc)) {
                return 0;
            }
        }
    }
}

/**
 * @brief 回溯并生成正向路径
 * @note 从 goal 沿 came_from 反向回溯到 start，再反转得到正向路径。
 *       Theta* 与 A* 在 came_from 链表上语义一致，差异仅在于 came_from 可能跨格指向祖父甚至更远祖先。
 * @param start_id 起点节点id
 * @param goal_id 终点节点id
 * @param path_out 输出的正向路径数组
 * @param max_len 最大允许的路径长度
 * @return 生成的路径长度，若失败则返回0
 */
static int reconstruct_path(uint16_t start_id, uint16_t goal_id, uint16_t *path_out, int max_len) {
    uint16_t rev_path[THETA_MAX_NODES];
    int rev_len = 0;
    uint16_t node = goal_id;

    while (node != THETA_NODE_INVALID && rev_len < THETA_MAX_NODES) {
        rev_path[rev_len++] = node;
        if (node == start_id) {
            break;
        }
        uint16_t prev = came_from[node];
        if (prev == node) {
            /* 起点自指父节点，正常终止条件已在上面命中；防御性 break。 */
            break;
        }
        node = prev;
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
 * @brief Theta* 寻路主函数
 *
 * @param config Theta*运行环境配置
 * @param start_row 起点行
 * @param start_col 起点列
 * @param goal_row 终点行
 * @param goal_col 终点列
 * @param path_out 输出路径数组，存储路径上节点的 id 序列
 * @param max_len 输出路径数组的最大长度
 *
 * @note 输入起终点格子坐标，返回路径上的格子编号序列。
 *       与原 astar_find_path_by_coord 的差异：
 *         1) 8 邻接展开 + 对角穿墙过滤；
 *         2) 邻居入选先做 LOS(parent(current), nb)：通过则 came_from[nb]=parent(current)（Path-2，跳过 current）；
 *            否则按 A* 标准更新 came_from[nb]=current，代价 = g(current) + euclidean(current, nb)（Path-1）；
 *         3) 启发与边代价均改用欧氏距离。
 *       编号规则: id = row * MAP_COLS + col (从0开始)
 *       返回值: >0 路径长度, 0 无路径或输入非法。
 */
int theta_star_find_path(const theta_config_t *config, int start_row, int start_col,
                         int goal_row, int goal_col, uint16_t *path_out, int max_len) {
    /* 参数合法性检查 */
    if (path_out == NULL || max_len <= 0 || config == NULL || config->is_walkable == NULL) {
        return 0;
    }
    if (config->rows * config->cols > THETA_MAX_NODES) {
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
    for (int i = 0; i < THETA_MAX_NODES; i++) {
        came_from[i] = THETA_NODE_INVALID;
        g_score[i] = THETA_INF_COST_F;
        f_score[i] = THETA_INF_COST_F;
        closed_set[i] = 0;
        open_pos[i] = -1;
    }

    open_count = 0;
    g_score[start_id] = 0.0f;
    f_score[start_id] = heuristic_euclidean(start_id, goal_row, goal_col);
    came_from[start_id] = start_id; /* 起点自指父节点，方便 Path-2 LOS 检查统一处理 */
    open_push_or_update(start_id, f_score[start_id]);

    /* Theta* 主循环 */
    while (open_count > 0) {
        uint16_t current = open_pop_min();
        if (current == THETA_NODE_INVALID) {
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

        uint16_t parent_c = came_from[current];

        uint16_t neighbors[8];
        int nb_count = get_neighbors8(current, neighbors);
        for (int i = 0; i < nb_count; i++) {
            uint16_t nb = neighbors[i];
            if (closed_set[nb]) {
                continue;
            }

            float tentative_g;
            uint16_t tentative_parent;

            /* Path-2：parent(current) 与 nb 之间有 LOS 时，直接跳过 current */
            if (has_line_of_sight(parent_c, nb)) {
                tentative_g = g_score[parent_c] + edge_cost(parent_c, nb);
                tentative_parent = parent_c;
            } else {
                /* Path-1：标准 A* 更新（8 邻接 + 欧氏代价） */
                tentative_g = g_score[current] + edge_cost(current, nb);
                tentative_parent = current;
            }

            if (tentative_g < g_score[nb]) {
                came_from[nb] = tentative_parent;
                g_score[nb] = tentative_g;
                f_score[nb] = tentative_g + heuristic_euclidean(nb, goal_row, goal_col);
                open_push_or_update(nb, f_score[nb]);
            }
        }
    }

    return 0;
}

#if 0
/**
 * @brief 主函数，用于 PC 调试演示
 * @note  本地编译命令：gcc -DPC_TEST -o theta theta_star.c -lm
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

    int len = theta_star_find_path_by_coord(start_row, start_col,
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
