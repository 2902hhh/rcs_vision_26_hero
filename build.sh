#!/bin/bash

# 遇到错误立即退出
set -e

# 定义构建目录
BUILD_DIR="build"

echo "=========================================="
echo "   RoboMaster Vision Auto-Build Script    "
echo "=========================================="

# 1. 检查 build 目录是否存在，如果需要全量重新编译，可以取消下面 rm 的注释
# echo "[INFO] Cleaning build directory..."
# rm -rf $BUILD_DIR

# 创建 build 目录
if [ ! -d "$BUILD_DIR" ]; then
    echo "[INFO] Creating build directory..."
    mkdir $BUILD_DIR
fi

cd $BUILD_DIR

# 2. 运行 CMake 配置
echo "[INFO] Running CMake..."
# 这里的 .. 表示 CMakeLists.txt 在上一级目录
cmake .. 

# 3. 运行 Make 进行编译
# -j$(nproc) 表示使用所有可用的 CPU 核心进行并行编译，加快速度
echo "[INFO] Compiling with $(nproc) threads..."
make -j$(nproc)

echo "=========================================="
echo "          Build Success! ��               "
echo "=========================================="

# 可选：编译完成后运行生成的可执行文件（例如自动运行主程序）
# cd ..
# ./build/auto_aim_test configs/standard3.yaml