/**
 * @file a_star.c
 * @author PickingChip
 * @brief a_star 算法
 * @version 0.1
 * @date 2026-04-27
 * 
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "a_star.h"

/*
 * 26RC 比赛场地的静态栅格地图，30行15列。
 * 场地被划分为450个40cm x 40cm的格子，每个格子对应地图中的一个元素。
 * 1 表示该格子可通行，0 表示该格子不可通行
 */
static const uint8_t g_map[MAP_ROWS][MAP_COLS] = {
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},/* 武馆开始 */
    {0,1,1,1,1,0,0,0,1,1,1,1,1,1,0},
    {0,1,1,1,1,1,1,1,1,1,1,1,1,1,0},
    {0,1,1,1,1,1,1,1,1,1,1,1,1,1,0},
    {0,1,1,1,1,1,1,1,1,1,1,1,1,1,0},
    {0,1,1,1,1,1,1,1,1,1,1,1,1,1,0},/* 梅林开始 */
    {0,1,1,1,1,1,1,1,1,1,1,1,1,1,0},
    {0,1,0,0,0,0,0,0,0,0,0,0,0,1,0},
    {0,1,0,0,0,0,0,0,0,0,0,0,0,1,0},
    {0,1,0,0,0,0,0,0,0,0,0,0,0,1,0},
    {0,1,0,0,0,0,0,0,0,0,0,0,0,1,0},
    {0,1,0,0,0,0,0,0,0,0,0,0,0,1,0},
    {0,1,0,0,0,0,0,0,0,0,0,0,0,1,0},
    {0,1,0,0,0,0,0,0,0,0,0,0,0,1,0},
    {0,1,0,0,0,0,0,0,0,0,0,0,0,1,0},
    {0,1,0,0,0,0,0,0,0,0,0,0,0,1,0},
    {0,1,0,0,0,0,0,0,0,0,0,0,0,1,0},
    {0,1,0,0,0,0,0,0,0,0,0,0,0,1,0},
    {0,1,0,0,0,0,0,0,0,0,0,0,0,1,0},
    {0,1,0,0,0,0,0,0,0,0,0,0,0,1,0},
    {0,1,0,0,0,0,0,0,0,0,0,0,0,1,0},
    {0,1,1,1,1,1,1,1,1,1,1,1,1,1,0},
    {0,1,1,1,1,1,1,1,1,1,1,1,1,1,0},
    {0,1,1,0,0,0,0,0,0,0,0,0,0,0,0},/* 对抗区开始 */
    {0,1,1,0,0,0,0,0,0,0,0,0,0,0,0},
    {0,1,1,0,1,1,1,1,1,1,1,1,1,1,0},
    {0,1,1,1,1,1,1,1,1,1,1,1,1,1,0},
    {0,1,1,1,1,1,1,1,1,1,1,1,1,1,0},
    {0,1,1,1,1,1,1,1,1,1,1,1,1,1,0},
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}
};

typedef struct {
    uint16_t id;    /* 节点索引 id（0~MAX_NODES-1） */
    uint16_t f;     /* 对应节点的 f 值 */
} OpenNode;

static uint16_t came_from[MAX_NODES];   /*  到达节点 n 的前驱节点 id，用于终点回溯整条路径。 */
static uint16_t g_score[MAX_NODES];     /* 起点到节点 n 的当前最小已知代价。*/
static uint16_t f_score[MAX_NODES];     /* f_score[n] = g_score[n] + heuristic(n, goal)，A* 的排序依据。 */
static uint8_t closed_set[MAX_NODES];   /* closed_set[n] = 1 表示节点 n 已完成扩展，不再重复处理。 */
static OpenNode open_list[MAX_NODES];   /* 开放表（待扩展节点集合），当前用顺序表实现。*/
static int16_t open_pos[MAX_NODES];     /* 节点在堆中的位置，-1 表示不在 open 表。*/
static int open_count;                  /* 开放表当前元素个数。*/


/* 行列边界检查：防止访问地图越界。 */
int is_valid_rc(uint16_t row, uint16_t col) {
    return (row < MAP_ROWS && col < MAP_COLS);
}

/* 坐标转索引：id = row * MAP_COLS + col */
uint16_t rc_to_id(uint16_t row, uint16_t col) {
    return (uint16_t)(row * MAP_COLS + col);
}

/* 索引转行号 */
uint16_t id_to_row(uint16_t id) {
    return (uint16_t)(id / MAP_COLS);
}

/* 索引转列号 */
uint16_t id_to_col(uint16_t id) {
    return (uint16_t)(id % MAP_COLS);
}

/* 判断节点是否可通行（地图值为1则可走） */
int is_walkable_id(uint16_t id) {
    uint16_t row = id_to_row(id);
    uint16_t col = id_to_col(id);
    return g_map[row][col] == 1;
}

/*
 * 启发函数 h(n)：曼哈顿距离。
 * 由于本实现是4邻接（上下左右，每步代价=1），
 * 曼哈顿距离满足可采纳性，A* 能得到最短路径。
 */
static uint16_t heuristic_to_goal(uint16_t id, uint16_t goal_row, uint16_t goal_col) {
    uint16_t r = (uint16_t)(id / MAP_COLS);
    uint16_t c = (uint16_t)(id - r * MAP_COLS);
    int dr = r - goal_row;
    int dc = c - goal_col;
    if (dr < 0) dr = -dr;
    if (dc < 0) dc = -dc;
    return (uint16_t)(dr + dc);
}

static void open_swap(int i, int j) {
    OpenNode tmp = open_list[i];
    open_list[i] = open_list[j];
    open_list[j] = tmp;
    open_pos[open_list[i].id] = (int16_t)i;
    open_pos[open_list[j].id] = (int16_t)j;
}

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

static void open_push_or_update(uint16_t id, uint16_t f) {
    int idx = open_pos[id];

    if (idx >= 0) {
        if (f < open_list[idx].f) {
            open_list[idx].f = f;
            open_sift_up(idx);
        }
        return;
    }

    if (open_count < MAX_NODES) {
        int insert_idx = open_count;
        open_list[insert_idx].id = id;
        open_list[insert_idx].f = f;
        open_pos[id] = (int16_t)insert_idx;
        open_count++;
        open_sift_up(insert_idx);
    }
}

static uint16_t open_pop_min(void) {
    if (open_count <= 0) {
        return NODE_INVALID;
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

static int get_neighbors4(uint16_t id, uint16_t neighbors[4]) {
    uint16_t row = (uint16_t)(id / MAP_COLS);
    uint16_t col = (uint16_t)(id - row * MAP_COLS);
    int count = 0;

    if (row > 0 && g_map[row - 1][col]) {
        neighbors[count++] = (uint16_t)(id - MAP_COLS);
    }
    if (row + 1 < MAP_ROWS && g_map[row + 1][col]) {
        neighbors[count++] = (uint16_t)(id + MAP_COLS);
    }
    if (col > 0 && g_map[row][col - 1]) {
        neighbors[count++] = (uint16_t)(id - 1u);
    }
    if (col + 1 < MAP_COLS && g_map[row][col + 1]) {
        neighbors[count++] = (uint16_t)(id + 1u);
    }

    return count;
}

static int reconstruct_path(uint16_t start_id, uint16_t goal_id, uint16_t *path_out, int max_len) {
    /*
     * 从 goal 沿 came_from 反向回溯到 start，
     * 再反转得到正向路径。
     */
    uint16_t rev_path[MAX_NODES];
    int rev_len = 0;
    uint16_t node = goal_id;

    while (node != NODE_INVALID && rev_len < MAX_NODES) {
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
 * @param start_row 起点行坐标
 * @param start_col 起点列坐标
 * @param goal_row 终点行坐标
 * @param goal_col 终点列坐标
 * @param path_out 输出路径数组，存储路径上节点的 id 序列
 * @param max_len 输出路径数组的最大长度
 * 
 * @note  输入起终点格子坐标，返回路径上的格子编号序列。
 *        编号规则: id = row * MAP_COLS + col (从0开始)
 *        返回值: >0 路径长度, 0 无路径或输入非法。
 */
int astar_find_path_by_coord(uint16_t start_row, uint16_t start_col,
                             uint16_t goal_row, uint16_t goal_col,
                             uint16_t *path_out, int max_len) {
    /* 参数合法性检查 */
    if (path_out == NULL || max_len <= 0) {
        return 0;
    }
    if (!is_valid_rc(start_row, start_col) || !is_valid_rc(goal_row, goal_col)) {
        return 0;
    }

    uint16_t start_id = rc_to_id(start_row, start_col);
    uint16_t goal_id = rc_to_id(goal_row, goal_col);

    if (start_id == goal_id) {
        path_out[0] = start_id;
        return 1;
    }

    /* 起点或终点落在障碍上，直接判失败 */
    if (!is_walkable_id(start_id) || !is_walkable_id(goal_id)) {
        return 0;
    }

    /* 每次规划前清空状态数组 */
    for (int i = 0; i < MAX_NODES; i++) {
        came_from[i] = NODE_INVALID;
        g_score[i] = INF_COST;
        f_score[i] = INF_COST;
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
        if (current == NODE_INVALID) {
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

            /* 4邻接每步代价固定为1 */
            uint16_t tentative_g = (uint16_t)(g_score[current] + 1u);
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

void print_map_with_path(const uint16_t *path, int path_len) {
    /*
     * 该函数用于调试可视化：
     * '1' 可走，'0' 障碍，'*' 路径，'S' 起点，'G' 终点。
     * 在 STM32 裸机中可改为串口输出，或直接删去。
     */
    printf("Map with path overlay:\n");
    for (int r = 0; r < MAP_ROWS; r++) {
        for (int c = 0; c < MAP_COLS; c++) {
            uint16_t id = rc_to_id(r, c);
            char ch = g_map[r][c] ? '1' : '0';

            if (path_len > 0) {
                if (id == path[0]) {
                    ch = 'S';
                } else if (id == path[path_len - 1]) {
                    ch = 'G';
                } else if (ch == '1') {
                    for (int i = 1; i < path_len - 1; i++) {
                        if (path[i] == id) {
                            ch = '*';
                            break;
                        }
                    }
                }
            }

            putchar(ch);
        }
        putchar('\n');
    }
}

#if 0

/**
 * @brief 路径打印函数
 * @note 打印路径覆盖后的地图：
 * '1' 可走, '0' 障碍, '*' 路径, 'S' 起点, 'G' 终点
 */
static void print_map_with_path(const uint16_t *path, int path_len) {
    /*
     * 该函数用于调试可视化：
     * '1' 可走，'0' 障碍，'*' 路径，'S' 起点，'G' 终点。
     * 在 STM32 裸机中可改为串口输出，或直接删去。
     */
    printf("Map with path overlay:\n");
    for (int r = 0; r < MAP_ROWS; r++) {
        for (int c = 0; c < MAP_COLS; c++) {
            uint16_t id = rc_to_id(r, c);
            char ch = g_map[r][c] ? '1' : '0';

            if (path_len > 0) {
                if (id == path[0]) {
                    ch = 'S';
                } else if (id == path[path_len - 1]) {
                    ch = 'G';
                } else if (ch == '1') {
                    for (int i = 1; i < path_len - 1; i++) {
                        if (path[i] == id) {
                            ch = '*';
                            break;
                        }
                    }
                }
            }

            putchar(ch);
        }
        putchar('\n');
    }
}

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

    printf("Path length: %d\n", len);
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