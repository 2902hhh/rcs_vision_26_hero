#ifndef TOOLS__DEBUG_MONITOR_HPP
#define TOOLS__DEBUG_MONITOR_HPP

#include <chrono>
#include <mutex>
#include <nlohmann/json.hpp>
#include <string>

#include "tools/plotter.hpp"

namespace tools
{

/**
 * 全局 Debug Monitor 单例
 * 在代码任意位置通过 WATCH("名称", 值) 注册变量，
 * 每帧结束时调用 FLUSH_DEBUG() 将所有变量通过 UDP 发送到 Web 仪表盘。
 *
 * 用法:
 *   WATCH("armor_count", armors.size());
 *   WATCH("yaw", plan.yaw);
 *   FLUSH_DEBUG();  // 每帧结尾调用一次
 */
class DebugMonitor
{
public:
  static DebugMonitor & instance()
  {
    static DebugMonitor monitor;
    return monitor;
  }

  /// 注册一个标量值到当前帧
  template <typename T>
  void watch(const std::string & name, const T & value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    frame_data_[name] = value;
  }

  /// 注册一个 Eigen 向量（展开为 name.x, name.y, name.z）
  template <typename Derived>
  void watchVec3(const std::string & name, const Derived & vec)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    frame_data_[name + ".x"] = vec(0);
    frame_data_[name + ".y"] = vec(1);
    if (vec.size() >= 3) frame_data_[name + ".z"] = vec(2);
  }

  /// 每帧结束时调用，将当前帧数据发送并清空
  void flush()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (frame_data_.empty()) return;
    auto now = std::chrono::steady_clock::now();
    double t = std::chrono::duration<double>(now - t0_).count();
    frame_data_["t"] = t;
    plotter_.plot(frame_data_);
    frame_data_.clear();
  }

  /// 重置起始时间
  void resetTime() { t0_ = std::chrono::steady_clock::now(); }

private:
  DebugMonitor() : t0_(std::chrono::steady_clock::now()) {}
  DebugMonitor(const DebugMonitor &) = delete;
  DebugMonitor & operator=(const DebugMonitor &) = delete;

  std::mutex mutex_;
  nlohmann::json frame_data_;
  Plotter plotter_;
  std::chrono::steady_clock::time_point t0_;
};

}  // namespace tools

// ============ 便捷宏 ============

/// 监控一个变量: WATCH("name", value)
#define WATCH(name, value) tools::DebugMonitor::instance().watch(name, value)

/// 监控一个 Eigen 3D 向量: WATCH_VEC3("pos", position)
#define WATCH_VEC3(name, vec) tools::DebugMonitor::instance().watchVec3(name, vec)

/// 每帧结尾调用一次，发送所有数据
#define FLUSH_DEBUG() tools::DebugMonitor::instance().flush()

#endif  // TOOLS__DEBUG_MONITOR_HPP
