# ESP32S3 Controller UI Menu System

[![License](https://img.shields.io/github/license/ouyanglingle/MyController)](https://github.com/ouyanglingle/MyController/main/LICENSE)

一个基于 ESP32-S3 的简单嵌入式菜单系统，支持水平滚动主菜单与竖直子菜单，使用 **TFT_eSPI** 显示驱动和 **PID 控制器** 实现复选框弹性动画效果，适用于物理按键交互的嵌入式设备界面开发。

---

## Visitors
![counter](https://count.getloli.com/@ouyanglingle?name=ouyanglingle&theme=booru-lewd&padding=7&offset=0&align=top&scale=1&pixelated=1&darkmode=auto)


## Star History

<a href="https://star-history.com/?repos=ouyanglingle/MyController&type=Timeline">
 <picture>
   <source media="(prefers-color-scheme: dark)" srcset="https://api.star-history.com/svg?repos=ouyanglingle/MyController&type=Timeline&theme=dark" />
   <source media="(prefers-color-scheme: light)" srcset="https://api.star-history.com/svg?repos=ouyanglingle/MyController&type=Timeline" />
   <img alt="Star History Chart" src="https://api.star-history.com/svg?repos=ouyanglingle/MyController&type=Timeline" />
 </picture>
</a>

---

## 📷 屏幕截图（示例）

> ![示例](https://github.com/ouyanglingle/MyController/tree/main/AllBMP/example.jpg)

---

## 🚀 功能特性

- ✅ 主菜单水平滚动显示图标（最多同时显示 3 个）
- ✅ 子菜单竖直滚动选择（支持超过屏幕可视范围的菜单项）
- ✅ 使用 PID 算法实现弹跳动画的高亮复选框
- ✅ 支持进度条（水平/垂直）展示当前可视区域占比
- ✅ 按键控制菜单切换与导航（LEFT / RIGHT / ENTER）
- ✅ 自动翻页：当选中超出可视范围时自动移动偏移量
- ✅ 支持多级菜单结构（目前为两级：主菜单 + 子菜单）

---

## 🧩 硬件需求

| 组件 | 引脚 | 备注 |
|------|------|------|
| TFT 显示屏 | SPI 接口 | 推荐使用 ILI9341 / ST7789 / ILI9488 等常见驱动 |
| 按键 - Enter | GPIO10 | 可修改 |
| 按键 - Left | GPIO9 | 可修改 |
| 按键 - Right | GPIO8 | 可修改 |

---

## 📦 软件依赖

- Arduino Core for ESP32 (推荐使用 Arduino IDE 或 PlatformIO)
- [TFT_eSPI](https://github.com/Bodmer/TFT_eSPI) 图形库（用于显示驱动）
- 自定义模块：
  - `menu`：菜单逻辑与动画系统
  - `key`：按键消抖与事件检测
  - `pid`：增量式 PID 控制器，用于动画平滑移动
  - `mypicture.h/.cpp`：图标资源头文件

---

## 📁 项目结构

```text
MyController/
├── src/
│   └── main.cpp            // 主程序入口
├── lib/
│   ├── MyUI/
│   │   ├── menu.cpp        // 菜单逻辑实现
│   │   ├── menu.h          // 菜单接口与数据结构
│   │   └── mypicture.h     // 图标资源定义
│   ├── Key/
│   │   ├── key.cpp         // 按键输入处理
│   │   └── key.h           // 按键状态枚举与配置
│   └── PID/
│       ├── pid.cpp         // PID 控制算法实现
│       └── pid.h           // PID 结构体与函数声明
└── README.md               // 当前文档
```

---

## 🛠️ 编译与烧录

本项目使用 **PlatformIO** 构建。确保已安装 VSCode + PlatformIO 插件。

### 安装依赖：

```bash
pio lib install "Bodmer/TFT_eSPI"
```

### 修改 `User_Setup_Select.h` 中的驱动配置以匹配你的屏幕型号。

---

## 📋 示例代码

在 `main.cpp` 中启动菜单系统：

```cpp
#include <menu.h>

void setup() {
    Key_Init();
    Menu_Init(); // 初始化显示与菜单
}

void loop() {
    Menu_Key_Handle(); // 处理按键事件并刷新界面
}
```

---

## ⚙️ 配置选项

你可以在 `menu.h` 中自定义以下参数（展示一部分）：

```c
#define BF_BG_COLOR TFT_BLACK     // 主菜单背景色
#define BF_FG_COLOR TFT_GOLD      // 主菜单字体颜色
#define CURSOR_COLOR TFT_RED      // 复选框颜色
#define PROGRESS_BAR_COLOR TFT_GREEN // 进度条颜色

uint8_t MaxVisibleMainMenuCNT = 3;   // 最大可见主菜单数量
uint8_t MaxVisibleSubMenuCNT = 11;  // 最大可见子菜单数量
float SelectBoxPID_posX_PID[] = {0.2, 0.05, 0.3}; // X轴弹性动画参数
```

---

## 📈 开发建议与扩展方向

你可以继续拓展的功能包括：

| 功能 | 说明 |
|------|------|
| 触摸支持 | 加入 XPT2046 / FT6206 触摸驱动，替代物理按键 |
| 更好的菜单管理 | 双向链表、折叠菜单等 |
| 图标+文字组合封装 | 将菜单项绘制封装为组件化函数 |
| 音效反馈 | 按下时播放提示音 |
| 子菜单动画 | 子菜单进入/退出时加入渐变或滑动动画 |
| 数据绑定 | 菜单项绑定变量，实时更新数值显示 |

---

## 📝 License

AGPL-3.0 License，详见 LICENSE 文件

---

## 🤝 贡献指南

欢迎提交 Issue 和 Pull Request！  
如果你希望添加新功能、优化动画逻辑或适配新的屏幕/控制器，请 Fork 并贡献你的代码。

---
