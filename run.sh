#!/usr/bin/env bash

set -e   # 遇错即停

echo "========  compiling  ========"
gcc -std=c99 -O2 astar.c main.c -o astar

echo "========  running    ========"
./astar | tee map.txt        # 同时落盘，方便二次查看

echo "========  done       ========"
# 如果终端够宽，可直接看；否则用 less -S 横向滚动
if command -v less >/dev/null 2>&1; then
    echo "按下 q 退出滚动查看"
    less -S map.txt
else
    echo "已生成 map.txt，请用编辑器查看"
fi