/**
 * @file astar.h
 * @brief 更适合单片机体质的C语言 A* 寻路实现，使用宽128高64的地图，避免浮点运算
 * @author AUST-WMZ
 * @version 1.0
 * @date 2025.10.06
 */

#ifndef ASTAR_H
#define ASTAR_H
#include <stdint.h>

#define MAP_W 32
#define MAP_H 32
#define MAX_OPEN 1024 // 同时打开的最大节点数
#define MAX_PATH 1024  // 返回路径最大长度

typedef struct
{
    uint16_t x, y;
} Point;

// 用户 API
void astar_init(void); // 上电调用一次
uint16_t map_idx(uint16_t x, uint16_t y);
uint8_t map_get(uint16_t x, uint16_t y);
void astar_set_barrier(uint16_t x, uint16_t y, uint8_t on); // 1=障碍 0=空地
int astar_search(Point from, Point to, Point *out_path);    // 返回路径长度，<=0 表示失败
uint8_t map_get(uint16_t x, uint16_t y);
void astar_reset();


#endif