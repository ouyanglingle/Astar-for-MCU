/**
 * @file astar.c
 * @brief 更适合单片机体质的C语言 A* 寻路实现，使用宽128高64的地图，避免浮点运算
 * @author AUST-WMZ
 * @version 1.0
 * @date 2025.10.06
 */

#include "astar.h"
#include <string.h>
#include <stdio.h>

// ------------------ 位带 -------------------

// 把整个 128×64 的地图“压缩”成一个一维字节数组，每个 bit 代表一格是否可通行。
static uint8_t map[MAP_W * MAP_H / 8];

// 想象把地图每一行首尾接起来，变成一条8192个比特的长带子
// 下面的map_idx就是坐标的序号

/// @brief 通过坐标获取地图索引
/// @param x 格子的x
/// @param y 格子的y
/// @return 索引
static inline uint16_t map_idx(uint16_t x, uint16_t y)
{
    return y * MAP_W + x;
}

/// @brief 通过索引获取地图坐标
/// @param idx 索引
/// @return 坐标
static inline Point map_to_point(uint16_t idx)
{
    Point p;
    p.x = idx % MAP_W;
    p.y = idx / MAP_W;
    return p;
}

/// @brief 判断指定格子是不是障碍
/// @param x 格子的x
/// @param y 格子的y
/// @return 0：空地，1：障碍
uint8_t map_get(uint16_t x, uint16_t y)
{
    if (x >= MAP_W || y >= MAP_H)
        return 1; // 越界就当墙

    uint16_t bit_idx = map_idx(x, y); // 算出是第几个比特
    uint16_t byte_idx = bit_idx / 8;  // 找到这个比特在0~1023的哪个字节中
    uint8_t bit_pos = bit_idx % 8;    // 找到这个比特在第几位

    uint8_t bit_val = (map[byte_idx] >> bit_pos) & 1; // 把那一比特抠出来

    return bit_val;
}

/// @brief “给地图画障碍/擦障碍”的 位图写入函数。
/// @param x 坐标
/// @param y
/// @param on 0：擦障碍，1：给障碍
void astar_set_barrier(uint16_t x, uint16_t y, uint8_t on)
{
    if (x >= MAP_W || y >= MAP_H)
        return;
    if (on)
        map[map_idx(x, y) >> 3] |= 1 << (map_idx(x, y) & 7);
    else
        map[map_idx(x, y) >> 3] &= ~(1 << (map_idx(x, y) & 7));
}

// ------------------ 节点 -------------------
typedef struct
{
    uint16_t x : 8, y : 8;    // 128=7bit 64=6bit
    uint16_t g : 12;          // g 最大 4095，单位“代价”
    uint16_t parent_idx : 12; // 父节点索引，不使用 struct Node *parent了
    uint16_t in_open : 1;     // 标记是否在开放列表
} Node;                       // 你想爆栈么？
static Node pool[MAX_OPEN];   // 静态节点内存池，用于存储所有开放列表中的节点
static uint16_t pool_cnt;     // 节点内存池使用计数器，记录已分配的节点数量

// 从节点内存池中分配一个节点，返回分配的节点在内存池中的索引
static inline uint16_t pool_alloc(void)
{
    return pool_cnt++;
}
// 重置节点内存池计数器，相当于清空内存池
static inline void pool_reset(void)
{
    pool_cnt = 0;
}

static Point target;
/**
 * 计算节点的f_score值(f=g+h)，用于A*算法的路径评估
 * @param idx 节点在pool中的索引
 * @return 节点的f_score值
 *
 * @details 该函数使用曼哈顿距离乘以10作为启发式函数h的估计值，计算公式为：
 * h = 10 * (|x1-x2| + |y1-y2|)
 * f = g + h
 */
static uint16_t f_score(uint16_t idx) /* f=g+h */
{
    Node *n = &pool[idx];
    uint16_t dx = n->x > target.x ? n->x - target.x : target.x - n->x;
    uint16_t dy = n->y > target.y ? n->y - target.y : target.y - n->y;
    uint16_t h10 = 10 * (dx + dy);
    return n->g + h10;
}

// -------------- 小顶堆（优先队列） ---------------

static uint16_t heap[MAX_OPEN];
static uint16_t heap_cnt;

/**
 * 交换堆中两个元素的位置
 * @param i 第一个元素的索引
 * @param j 第二个元素的索引
 */
static void heap_swap(uint16_t i, uint16_t j)
{
    uint16_t t = heap[i];
    heap[i] = heap[j];
    heap[j] = t;
}

/**
 * 将指定索引的元素向上调整以维护最小堆性质
 * @param idx 需要向上调整的元素索引
 */
static void heap_up(uint16_t idx)
{
    while (idx)
    {
        uint16_t p = (idx - 1) >> 1;
        if (f_score(heap[idx]) >= f_score(heap[p]))
            break;
        heap_swap(idx, p);
        idx = p;
    }
}

/**
 * 将堆顶元素向下调整以维护最小堆性质
 */
static void heap_down(void)
{
    uint16_t idx = 0;
    while (1)
    {
        uint16_t l = (idx << 1) + 1, r = l + 1, smallest = idx;
        if (l < heap_cnt && f_score(heap[l]) < f_score(heap[smallest]))
            smallest = l;
        if (r < heap_cnt && f_score(heap[r]) < f_score(heap[smallest]))
            smallest = r;
        if (smallest == idx)
            break;
        heap_swap(idx, smallest);
        idx = smallest;
    }
}

/**
 * 向堆中添加一个元素
 * @param idx 要添加的元素索引
 */
static void heap_push(uint16_t idx)
{
    heap[heap_cnt++] = idx;
    heap_up(heap_cnt - 1);
}

/**
 * 从堆中弹出f_score最小的元素
 * @return f_score最小的元素索引
 */
static uint16_t heap_pop(void)
{
    uint16_t ret = heap[0];
    heap[0] = heap[--heap_cnt];
    heap_down();
    return ret;
}

// -------------- 方向表 --------------
static const int8_t dx8[8] = {1, 1, 0, -1, -1, -1, 0, 1};
static const int8_t dy8[8] = {0, 1, 1, 1, 0, -1, -1, -1};
static const uint8_t cost8[8] = {10, 14, 10, 14, 10, 14, 10, 14}; // 直线走代价：10/格，对角线走代价：14/格

// -------------- 搜索路径 --------------

/// @brief A* 搜索
/// @param from 起点坐标
/// @param to 终点坐标
/// @param out_path 事先准备好的数组首地址，用来存储路径
/// @return 返回 0，表示无解，此时 out_path 内容无效。返回1，此时 out_path[0] 是起点，out_path[len-1] 是终点。
int astar_search(Point from, Point to, Point *out_path)
{
    // 检查起点和终点是否在地图范围内，以及是否为障碍物
    if (from.x >= MAP_W || from.y >= MAP_H || to.x >= MAP_W || to.y >= MAP_H || map_get(from.x, from.y) ||
        map_get(to.x, to.y))
        return 0;

    pool_reset();
    heap_cnt = 0;
    target = to;

    // 起点入池
    uint16_t start = pool_alloc();
    pool[start].x = from.x;
    pool[start].y = from.y;
    pool[start].g = 0;
    pool[start].parent_idx = 0xFFF; // 无效值
    pool[start].in_open = 1;
    heap_push(start);

    // 主搜索循环
    while (heap_cnt)
    {
        // 取出f值最小的节点
        uint16_t cur = heap_pop();
        Node *cn = &pool[cur];
        cn->in_open = 0;

        // 判断是否到达终点
        if (cn->x == to.x && cn->y == to.y)
        {
            // 回溯代码
            int len = 0;
            uint16_t idx = cur;
            static Point rev[MAX_PATH];
            while (idx != 0xFFF && len < MAX_PATH)
            {
                rev[len].x = pool[idx].x;
                rev[len].y = pool[idx].y;
                idx = pool[idx].parent_idx;
                len++;
            }
            for (int i = 0; i < len; i++)
                out_path[i] = rev[len - 1 - i];
            return len;
        }

        // 8 邻域搜索
        for (int d = 0; d < 8; d++)
        {
            int nx = cn->x + dx8[d];
            int ny = cn->y + dy8[d];
            if (nx < 0 || nx >= MAP_W || ny < 0 || ny >= MAP_H)
                continue;
            if (map_get(nx, ny))
                continue;

            uint16_t new_g = cn->g + cost8[d];
            // 查找是否已存在节点
            uint16_t i, found = 0xFFFF;
            for (i = 0; i < pool_cnt; i++)
            {
                if (pool[i].x == nx && pool[i].y == ny)
                {
                    found = i;
                    break;
                }
            }
            if (found != 0xFFFF)
            {
                // 已在池里，如果新路径更优则更新
                if (new_g < pool[found].g)
                {
                    pool[found].g = new_g;
                    pool[found].parent_idx = cur;
                    if (pool[found].in_open)
                        heap_up(found); // 更新堆
                }
            }
            else
            {
                // 新节点
                if (pool_cnt >= MAX_OPEN)
                {
                    continue;
                }
                uint16_t nd = pool_alloc();
                pool[nd].x = nx;
                pool[nd].y = ny;
                pool[nd].g = new_g;
                pool[nd].parent_idx = cur;
                pool[nd].in_open = 1;
                heap_push(nd);
            }
        }
    }
    printf("DEBUG: open list empty, total pool used %d\n", pool_cnt);
    return 0; // 无解
}

// -------------- 初始化 --------------
void astar_init(void)
{
    memset(map, 0, sizeof(map));
}