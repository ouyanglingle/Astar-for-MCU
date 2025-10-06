#include <stdio.h>
#include <string.h>
#include "astar.h"

Point path[MAX_PATH];
int path_len;

static void draw_map(void)
{
    /* 先打路径进位图，方便查询 */
    uint8_t on_path[MAP_W * MAP_H / 8];
    memset(on_path, 0, sizeof(on_path));
    for (int i = 0; i < path_len; ++i)
    {
        uint16_t idx = (path[i].y * MAP_W + path[i].x);
        on_path[idx >> 3] |= 1 << (idx & 7);
    }

    printf("\n==== 128x64 ASCII map ====\n");
    for (uint16_t y = 0; y < MAP_H; ++y)
    {
        for (uint16_t x = 0; x < MAP_W; ++x)
        {
            uint16_t idx = y * MAP_W + x;
            char c;
            if (x == 10 && y == 10)
                c = 'S'; /* 起点 */
            else if (x == 120 && y == 50)
                c = 'G'; /* 终点 */
            else if (on_path[idx >> 3] & (1 << (idx & 7)))
                c = '*'; /* 路径 */
            else if (map_get(x, y))
                c = '#'; /* 障碍 */
            else
                c = ' ';
            putchar(c);
        }
        putchar('\n');
    }
    printf("========== END ==========\n");
}

int main(void)
{
    astar_init();

    /* 画障碍 */
    for (int i = 20; i < 40; ++i)
        astar_set_barrier(i, 30, 1);
    for (int i = 10; i < 50; ++i)
        astar_set_barrier(60, i, 1);

    Point start = {10, 10};
    Point goal = {120, 50};

    path_len = astar_search(start, goal, path);
    if (path_len > 0)
    {
        printf("\nPath found! length = %d\n", path_len);
        draw_map();
    }
    else
    {
        printf("\nNo path!\n");
    }
    return 0;
}