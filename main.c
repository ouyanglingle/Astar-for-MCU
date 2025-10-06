#include <stdio.h>
#include <string.h>
#include "astar.h"

Point path[MAX_PATH];
int path_len;

/* -------------- 调试：打印地图+障碍+路径 -------------- */
static void draw_map(void)
{
    uint8_t on_path[MAP_W * MAP_H / 8];
    memset(on_path, 0, sizeof(on_path));
    for (int i = 0; i < path_len; ++i)
    {
        uint16_t idx = (path[i].y * MAP_W + path[i].x);
        on_path[idx >> 3] |= 1 << (idx & 7);
    }

    printf("\n==== %dx%d debug map ====\n", MAP_W, MAP_H);
    for (uint16_t y = 0; y < MAP_H; ++y)
    {
        for (uint16_t x = 0; x < MAP_W; ++x)
        {
            uint16_t idx = y * MAP_W + x;
            char c;
            if (x == 10 && y == 10)
                c = 'S';
            else if (x == 70 && y == 35)
                c = 'G';
            else if (on_path[idx >> 3] & (1 << (idx & 7)))
                c = '*';
            else if (map_get(x, y))
                c = '#';
            else
                c = ' ';
            putchar(c);
        }
        putchar('\n');
    }
}

/* -------------- 检查起点/终点合法性 -------------- */
static int ok_to_go(Point s, Point g)
{
    if (s.x >= MAP_W || s.y >= MAP_H || g.x >= MAP_W || g.y >= MAP_H)
    {
        printf("ERR: start or goal out of map boundary!\n");
        return 0;
    }
    if (map_get(s.x, s.y))
    {
        printf("ERR: start (%d,%d) is inside obstacle!\n", s.x, s.y);
        return 0;
    }
    if (map_get(g.x, g.y))
    {
        printf("ERR: goal (%d,%d) is inside obstacle!\n", g.x, g.y);
        return 0;
    }
    return 1;
}

static void print_neighbors(Point p)
{
    printf("\nNeighbor check around (%d,%d):\n", p.x, p.y);
    for (int dy = -1; dy <= 1; ++dy)
    {
        for (int dx = -1; dx <= 1; ++dx)
        {
            int nx = p.x + dx;
            int ny = p.y + dy;
            if (nx < 0 || nx >= MAP_W || ny < 0 || ny >= MAP_H)
            {
                putchar('X'); // 越界
            }
            else if (map_get(nx, ny))
            {
                putchar('#'); // 障碍
            }
            else
            {
                putchar('.'); // 空地
            }
        }
        putchar('\n');
    }
}

int main(void)
{
    astar_init();
    printf("MAP_W=%u  MAP_H=%u  MAX_OPEN=%u\n", MAP_W, MAP_H, MAX_OPEN);
    /* 画障碍（保持你原来那两条） */
    for (int i = 20; i < 40; ++i)
        astar_set_barrier(i, 30, 1);
    for (int i = 10; i < 20; ++i)
        astar_set_barrier(60, i, 1);
    for (int i = 23; i < 45; ++i)
        astar_set_barrier(60, i, 1);

    Point start = {10, 10};
    Point goal = {70, 35};

    /* 合法性检查 */
    if (!ok_to_go(start, goal))
        return 1;

    path_len = astar_search(start, goal, path);
    if (path_len > 0)
    {
        printf("\nPath found! length = %d\n", path_len);
        draw_map();
    }
    else
    {
        printf("\nNo path!  (check map above)\n");
        draw_map(); // 把当前障碍也打出来，看是不是被围死
    }
    return 0;
}