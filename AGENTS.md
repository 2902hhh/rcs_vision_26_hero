# AGENTS.md - sp_vision 编码规范

## 项目概述

本项目是 **sp_vision** - 同济大学 SuperPower 战队 2025 赛季自瞄视觉系统。这是一个用于机器人战斗竞赛自动瞄准的 C++17 计算机视觉项目。

## 构建命令

### 完整构建
```bash
# 标准 CMake 构建
./build.sh

# 或手动构建：
cmake -B build
make -C build/ -j$(nproc)
```

### 构建单个目标
```bash
make -C build/ <target_name> -j$(nproc)
```

### 清理构建
```bash
rm -rf build/
cmake -B build
make -C build/ -j$(nproc)
```

## 测试命令

### 运行测试
每个测试都是独立的可执行文件，位于 `build/` 目录下：

```bash
# 运行特定测试（最常用）
./build/auto_aim_test configs/demo.yaml
./build/camera_test
./build/gimbal_test configs/standard3.yaml

# 基于视频的测试
./build/auto_aim_test configs/demo.yaml assets/demo/demo

# 指定帧范围
./build/auto_aim_test -c=configs/demo.yaml -s=100 -e=500 assets/demo/demo
```

### 可用测试可执行文件
- `auto_aim_test` - 主自瞄测试（使用录制的视频）
- `camera_test` - 相机硬件测试
- `camera_detect_test` - 识别器测试（使用工业相机）
- `gimbal_test` - 云台通信测试
- `planner_test` - 轨迹规划器测试（实车）
- `planner_test_offline` - 规划器离线测试
- `fire_test` - 火控测试
- `handeye_test` - 手眼标定测试
- `cboard_test` - C 板（STM32）通信测试
- `detector_video_test` - 识别器测试（使用视频文件）
- `minimum_vision_system` - 最小视觉系统测试

## 代码风格规范

### 格式化（由 .clang-format 强制执行）
- **风格**: Google C++ 风格，含自定义项
- **行宽限制**: 100 字符
- **大括号**: 自定义 - 类/函数/命名空间/结构体/枚举后换行
- **指针对齐**: 中间对齐 (`int * ptr`)
- **缩进**: 2 个空格

### 命名约定
- **类名**: PascalCase（`ArmorDetector`, `TrajectoryPlanner`）
- **函数名**: camelCase（`detectArmors()`, `calculateTrajectory()`）
- **变量名**: snake_case（`armor_list`, `target_position`）
- **成员变量**: snake_case 加下划线后缀（`config_path_`, `threshold_`）
- **常量**: UPPER_SNAKE_CASE（`ARMOR_NAMES`, `MAX_DETECTION_RANGE`）
- **命名空间**: snake_case（`auto_aim`, `tools`, `io`）
- **文件名**: snake_case（头文件用 `.hpp`，源文件用 `.cpp`）

### 包含顺序
```cpp
// 1. C++ 标准头文件
#include <vector>
#include <memory>

// 2. 第三方库头文件
#include <opencv2/opencv.hpp>
#include <fmt/core.h>
#include <Eigen/Core>

// 3. 项目头文件（使用引号）
#include "tasks/auto_aim/detector.hpp"
#include "tools/logger.hpp"
```

### 命名空间
- 头文件中始终使用显式命名空间限定符
- `using namespace` 仅在 `.cpp` 文件中允许（最好在函数内部使用）
- 常用：`using namespace std::chrono;` 用于时间操作

### 类结构
```cpp
#ifndef AUTO_AIM__DETECTOR_HPP
#define AUTO_AIM__DETECTOR_HPP

namespace auto_aim {

class Detector {
public:
  explicit Detector(const std::string & config_path, bool debug = true);
  
  std::vector<Armor> detect(const cv::Mat & img);
  
private:
  double threshold_;
  bool debug_;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__DETECTOR_HPP
```

### 错误处理
- 使用 spdlog 进行日志记录（`tools::logger()->info()`, `->warn()`, `->error()`）
- 返回空容器表示"无检测结果"，不使用异常
- 检查 OpenCV Mat 是否为空：`if (img.empty()) return;`
- 使用 `tools::Exiter` 实现优雅退出处理

### 注释
- 单行注释使用 `//`
- 多行说明使用 `/* */`
- 允许使用中文注释描述领域特定术语
- 在头文件中说明函数行为

### 配置
- 配置文件使用 YAML 格式，位于 `configs/` 目录
- 配置路径通过构造函数参数传递
- 使用 `tools::yaml` 进行 YAML 解析

### 主要依赖
- OpenCV 4.x - 计算机视觉
- Eigen3 - 线性代数
- fmt - 字符串格式化
- spdlog - 日志记录
- yaml-cpp - 配置解析
- OpenVINO - 神经网络推理
- nlohmann/json - JSON 处理

## 项目结构

```
sp_vision/
├── src/           # 应用程序可执行文件（步兵、哨兵、无人机等）
├── tests/         # 测试可执行文件
├── tasks/         # 功能模块
│   ├── auto_aim/  # 自瞄算法
│   ├── auto_buff/ # 打符（能量机关）检测
│   └── omniperception/  # 全向感知
├── tools/         # 工具类和函数
├── io/            # 硬件抽象层（相机、串口等）
├── calibration/   # 标定工具
└── configs/       # YAML 配置文件
```

## 测试工作流

1. 录制测试数据：使用 `recorder.hpp` 捕获视频 + IMU 数据
2. 运行离线测试：`./build/auto_aim_test configs/demo.yaml path/to/recording`
3. 调试可视化：使用 `cv::imshow` 配合 `cv::waitKey(1)`
4. 绘制数据：使用 `tools::Plotter` 配合 PlotJuggler 实现实时曲线

## 硬件要求

- 平台：Intel NUC (i7-1260P)
- 操作系统：Ubuntu 22.04
- 相机：海康机器人 MV-CS016-10UC
- MCU：RoboMaster C 板（STM32F407）
- 通信方式：USB 虚拟串口

## 重要注意事项

- **无 ROS 依赖**（哨兵的导航 ROS2 接口除外）
- 线程安全：使用 `tools::ThreadSafeQueue` 进行线程间通信
- 时间同步：所有时间戳使用 `std::chrono::steady_clock`
- FPS 关键：在 i7-1260P 上目标 100+ FPS
- 内存：处理前检查 OpenCV Mat 是否为空
- 串口：使用 `io::Gimbal` 类与 MCU 通信

## 调试技巧

- 使用 `tools::draw_text()` 和 `tools::draw_points()` 进行可视化
- 显示前调整图像大小：`cv::resize(img, img, {}, 0.5, 0.5)`
- 生产环境配置中始终使用绝对路径
- 检查相机权限：`sudo usermod -a -G dialout $USER`
