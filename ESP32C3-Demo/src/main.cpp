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
#define RATIO 6              // 地图在屏幕上的缩放系数
Point path[MAX_PATH];
int path_len;

#define KEY1 9
#define KEY2 10
#define KEY3 20
Point start = {1, 1};
Point goal = {1, 1};
void setup()
{
    tft.init();
    tft.fillScreen(TFT_BLACK);
    tft.setRotation(0);
    bf.createSprite(TFT_WIDTH, TFT_HEIGHT);
    bf.setTextColor(TFT_WHITE, TFT_BLACK);
    bf.setTextSize(1);

    astar_init();
    build_complex_map();
    while (digitalRead(KEY3) != 0);
}

void loop()
{
    goal.x++;
    do
    {
        goal.x++;
        if (goal.x > 30)
        {
            goal.x = 1;
            goal.y++;
            if (goal.y > 30)
            {
                goal.y = 1;
            }
        }
    } while (map_get(goal.x, goal.y) == 1);

    bf.fillScreen(TFT_BLACK);
    bf.setCursor(0, 200);
    bf.printf("MAP_W=%u, MAP_H=%u, MAX_OPEN=%u\n", MAP_W, MAP_H, MAX_OPEN);
    bf.fillRect(RATIO * start.x, RATIO * start.y, RATIO, RATIO, TFT_GREEN);
    bf.fillRect(RATIO * goal.x, RATIO * goal.y, RATIO, RATIO, TFT_YELLOW);
    for (uint16_t i = 0; i < BARRIER_NUM; i++)
    {
        bf.fillRect(RATIO * BARRIERS[i].x, RATIO * BARRIERS[i].y, RATIO, RATIO, TFT_RED);
    }
    bf.pushSprite(0, 0);

    astar_reset();
    build_complex_map();
    uint32_t timestamp = millis();
    path_len = astar_search(start, goal, path);
    bf.setCursor(0, 220);
    bf.printf("TakeTime %u ms", millis() - timestamp);
    bf.pushSprite(0, 0);

    if (path_len > 0)
    {
        bf.setCursor(0, 210);
        bf.printf("Found Path, take %d steps", path_len);
        bf.pushSprite(0, 0);
        for (int kk = 1; kk < path_len - 1; kk++)
        {
            bf.drawRect(RATIO * path[kk].x, RATIO * path[kk].y, RATIO, RATIO, TFT_WHITE);
            if (digitalRead(KEY3) == 0)
                bf.pushSprite(0, 0);
        }
        bf.pushSprite(0, 0);
    }
    else
    {
        bf.setCursor(0, 210);
        bf.printf("Not Found Path: %d,%d,%d", path_len, map_get(start.x, start.y), map_get(goal.x, goal.y));
        bf.pushSprite(0, 0);
    }
    delay(10);
}
