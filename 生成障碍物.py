#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
128×64 障碍绘制器（带坐标轴）
左上角格子中心 = (0,0)
左键画障碍，右键擦障碍，支持拖动
导出 build_complex_map() C 函数
"""

import tkinter as tk
from tkinter import messagebox, filedialog

MAP_W, MAP_H = 32, 32
CELL = 15  # 格子像素
ORIGIN_X = 20  # 坐标轴留白
ORIGIN_Y = 20
BAR_COLOR = "#FF3B30"
BG_COLOR = "#FFFFFF"
AXIS_COLOR = "#555555"


class MapEditor(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("128×64 障碍编辑器（带坐标轴）")
        self.resizable(False, False)
        self.barriers = set()  # (x,y)

        # 菜单
        menubar = tk.Menu(self)
        filemenu = tk.Menu(menubar, tearoff=0)
        filemenu.add_command(label="Export C", command=self.export_c)
        menubar.add_cascade(label="File", menu=filemenu)
        self.config(menu=menubar)

        # 画布（额外 20+20 留给坐标轴）
        canvas_w = ORIGIN_X + MAP_W * CELL + 20
        canvas_h = ORIGIN_Y + MAP_H * CELL + 20
        self.canvas = tk.Canvas(
            self, width=canvas_w, height=canvas_h,
            bg=BG_COLOR, bd=0, highlightthickness=0
        )
        self.canvas.pack()
        self.draw_grid_with_axis()

        # 鼠标事件（左键画，右键擦）
        self.canvas.bind("<Button-1>",  self.on_left_press)
        self.canvas.bind("<B1-Motion>", self.on_left_drag)
        self.canvas.bind("<ButtonRelease-1>",  self.on_release)

        self.canvas.bind("<Button-3>",  self.on_right_press)
        self.canvas.bind("<B3-Motion>", self.on_right_drag)
        self.canvas.bind("<ButtonRelease-3>",  self.on_release)

    # ---------- 画网格 + 坐标轴 ----------
    def draw_grid_with_axis(self):
        # 网格线
        for x in range(MAP_W + 1):
            self.canvas.create_line(
                ORIGIN_X + x * CELL, ORIGIN_Y,
                ORIGIN_X + x * CELL, ORIGIN_Y + MAP_H * CELL,
                fill="#CCCCCC"
            )
        for y in range(MAP_H + 1):
            self.canvas.create_line(
                ORIGIN_X, ORIGIN_Y + y * CELL,
                ORIGIN_X + MAP_W * CELL, ORIGIN_Y + y * CELL,
                fill="#CCCCCC"
            )
        # 坐标数字（每 10 格一个）
        for x in range(0, MAP_W + 1, 10):
            self.canvas.create_text(
                ORIGIN_X + x * CELL, ORIGIN_Y - 7,
                text=str(x), fill=AXIS_COLOR, font=("Arial", 8)
            )
        for y in range(0, MAP_H + 1, 10):
            self.canvas.create_text(
                ORIGIN_X - 7, ORIGIN_Y + y * CELL,
                text=str(y), fill=AXIS_COLOR, font=("Arial", 8)
            )
        # 原点标记
        self.canvas.create_oval(
            ORIGIN_X - 2, ORIGIN_Y - 2,
            ORIGIN_X + 2, ORIGIN_Y + 2,
            fill=AXIS_COLOR
        )

    # ---------- 坐标转换 ----------
    def canvas_to_map(self, cx, cy):
        """画布坐标 → 地图坐标（左上角格子中心 = 0,0）"""
        x = (cx - ORIGIN_X) // CELL
        y = (cy - ORIGIN_Y) // CELL
        return x, y

    # ---------- 左键画障碍 ----------
    def on_left_press(self, event):
        x, y = self.canvas_to_map(event.x, event.y)
        self._set_barrier(x, y, True)

    def on_left_drag(self, event):
        x, y = self.canvas_to_map(event.x, event.y)
        self._set_barrier(x, y, True)

    # ---------- 右键擦障碍 ----------
    def on_right_press(self, event):
        x, y = self.canvas_to_map(event.x, event.y)
        self._set_barrier(x, y, False)

    def on_right_drag(self, event):
        x, y = self.canvas_to_map(event.x, event.y)
        self._set_barrier(x, y, False)

    # ---------- 统一画/擦 ----------
    def _set_barrier(self, x, y, draw):
        """draw=True 画障碍，False 擦障碍"""
        if not (0 <= x < MAP_W and 0 <= y < MAP_H):
            return
        cx, cy = ORIGIN_X + x * CELL, ORIGIN_Y + y * CELL
        if draw:
            if (x, y) not in self.barriers:
                self.barriers.add((x, y))
                self.canvas.create_rectangle(
                    cx, cy, cx + CELL, cy + CELL,
                    fill=BAR_COLOR, outline="", tags=f"cell{x}_{y}"
                )
        else:
            if (x, y) in self.barriers:
                self.barriers.discard((x, y))
                self.canvas.delete(f"cell{x}_{y}")

    # ---------- 松开按键 ----------
    def on_release(self, event):
        pass

    # ---------- 导出 C ----------
    def export_c(self):
        if not self.barriers:
            messagebox.showwarning("Empty", "没有障碍物可导出！")
            return

        file_path = filedialog.asksaveasfilename(
            defaultextension=".cpp",
            filetypes=[("C++ files", "*.cpp"), ("All files", "*.*")],
            initialfile="map_barrier.cpp",
        )
        if not file_path:
            return

        coords = sorted(self.barriers)
        num = len(coords)

        with open(file_path, "w", encoding="utf-8") as f:
            f.write('#include "astar.h"\n\n')
            f.write("typedef struct { uint16_t x, y; } Point;\n\n")
            f.write(f"static const Point BARRIERS[{num}] = {{\n")

            # 每行 8 个
            for i in range(0, num, 8):
                chunk = coords[i:i+8]
                line = "    " + ", ".join(f"{{ {x}, {y} }}" for x, y in chunk)
                if i + 8 < num:
                    line += ","
                f.write(line + "\n")

            f.write("};\n\n")
            f.write(f"static const uint16_t BARRIER_NUM = {num};\n\n")

            f.write("void build_complex_map(void)\n")
            f.write("{\n")
            f.write("    for (uint16_t i = 0; i < BARRIER_NUM; ++i)\n")
            f.write("        astar_set_barrier(BARRIERS[i].x, BARRIERS[i].y, 1);\n")
            f.write("}\n")

        messagebox.showinfo("OK", f"已导出 {num} 个障碍到\n{file_path}")


if __name__ == "__main__":
    MapEditor().mainloop()