#include <Arduino.h>
#include <TFT_eSPI.h>
#include <astar.h>
#include <map_barrier.h>
// 全局显示对象
TFT_eSPI tft = TFT_eSPI();          // 主显示对象
TFT_eSprite bf = TFT_eSprite(&tft); // 主显示全缓冲区（240x320）

#define PIX_OBSTACLE TFT_RED // 障碍点亮
#define PIX_PATH TFT_WHITE   // 路径也点亮（想区分可改成 2 或反色）
#define PIX_EMPTY TFT_BLACK  // 空地熄灭
#define RATIO 6 // 地图在屏幕上的缩放系数
Point path[MAX_PATH];
int path_len;
void oled_draw_map(void)
{
    /* 1. 先把路径坐标打成位图，方便后面查表 */
    uint8_t on_path[MAP_W * MAP_H / 8];
    memset(on_path, 0, sizeof(on_path));

    for (int i = 0; i < path_len; ++i)
    {
        uint16_t idx = map_idx(path[i].x, path[i].y);
        on_path[idx >> 3] |= 1 << (idx & 7);
    }

    /* 2. 逐点刷新 OLED */
    for (uint16_t y = 0; y < MAP_H; ++y)
    {
        for (uint16_t x = 0; x < MAP_W; ++x)
        {
            uint16_t idx = map_idx(x, y);
            uint8_t is_path = (on_path[idx >> 3] >> (idx & 7)) & 1;
            uint8_t is_ob = map_get(x, y);

            if (is_ob)
                // bf.drawPixel(RATIO * x, RATIO * y, PIX_OBSTACLE);
                bf.fillRect(RATIO * x, RATIO * y, RATIO, RATIO, PIX_OBSTACLE);
            else if (is_path)
                // bf.drawPixel(RATIO * x, RATIO * y, PIX_PATH);
                bf.fillRect(RATIO * x, RATIO * y, RATIO, RATIO, PIX_PATH);
            else
                // bf.drawPixel(RATIO * x, RATIO * y, PIX_EMPTY);
                bf.fillRect(RATIO * x, RATIO * y, RATIO, RATIO, PIX_EMPTY);
        }
    }
}

#define KEY1 9
#define KEY2 10
#define KEY3 20

void setup()
{
    tft.init();
    tft.fillScreen(TFT_BLACK);
    tft.setRotation(0);
    bf.createSprite(TFT_WIDTH, TFT_HEIGHT);
    bf.setTextColor(TFT_WHITE, TFT_BLACK);
    bf.setTextSize(1);

    astar_init();
    bf.setCursor(0, 200);
    bf.printf("MAP_W=%u  MAP_H=%u  MAX_OPEN=%u\n", MAP_W, MAP_H, MAX_OPEN);

    build_complex_map();
    Point start = {1, 1};
    Point goal = {13, 28};

    bf.drawRect(RATIO * start.x, RATIO * start.y, RATIO, RATIO, TFT_GREEN);
    bf.drawRect(RATIO * goal.x, RATIO * goal.y, RATIO, RATIO, TFT_YELLOW);
    path_len = astar_search(start, goal, path);

    if (path_len > 0)
    {
        oled_draw_map();
        bf.setCursor(0, 220);
        bf.printf("Found Path, take %d steps", path_len);
        bf.pushSprite(0, 0);
    }
    else
    {
        bf.fillScreen(TFT_BLACK); // 没路就清屏
        bf.setCursor(0, 220);
        oled_draw_map();
        bf.printf("Not Found Path: %d,%d,%d", path_len, map_get(1, 1), map_get(26, 26));
        bf.pushSprite(0, 0);
    }
}

void loop()
{
}
