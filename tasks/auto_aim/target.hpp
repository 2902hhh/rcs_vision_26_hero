// --- START OF FILE text/plain ---
#ifndef AUTO_AIM__TARGET_HPP
#define AUTO_AIM__TARGET_HPP

#include <Eigen/Dense>
#include <chrono>
#include <optional>
#include <queue>
#include <string>
#include <vector>

#include "armor.hpp"
#include "tools/extended_kalman_filter.hpp"

namespace auto_aim
{

class Target
{
public:
  ArmorName name;
  ArmorType armor_type;
  ArmorPriority priority;
  bool jumped;
  int last_id;  // debug only

  // === 前哨站专用变量 ===
  bool outpost_initialized = false;
  double outpost_base_height = 0.0; // 基准高度
  int outpost_layer = 0; // 当前层级 (0, 1, 2)
  static constexpr double OUTPOST_HEIGHT_DIFF = 0.10; // 层间高度差 10cm

  // 前哨站静止/旋转状态判定 (基于观测角速度)
  bool outpost_is_static = false;       // 当前是否处于静止状态
  int outpost_static_count = 0;         // 连续静止帧计数
  double outpost_last_yaw = 0.0;        // 上一帧观测到的 yaw
  bool outpost_last_yaw_valid = false;  // 上一帧 yaw 是否有效
  double outpost_observed_omega = 0.0;  // 滑动平均观测角速度 (rad/s)
  static constexpr double OUTPOST_STATIC_OMEGA_THRESH = 0.3;   // 静止判定角速度阈值 (rad/s)
  static constexpr double OUTPOST_ROTATE_OMEGA_THRESH = 0.6;   // 旋转判定角速度阈值 (rad/s) — 滞回上界
  static constexpr int OUTPOST_STATIC_ENTER_COUNT = 20;        // 进入静止状态所需连续帧数
  static constexpr int OUTPOST_STATIC_EXIT_COUNT = 8;          // 退出静止状态所需连续帧数

  // 前哨站 layer 跳变抑制
  int outpost_pending_layer = -1;       // 待确认的新 layer
  int outpost_pending_count = 0;        // 新 layer 连续出现帧数
  static constexpr int OUTPOST_LAYER_CONFIRM_COUNT = 3; // 确认切换需要的连续帧数
  // ==========================

  Target() = default;
  Target(
    const Armor & armor, std::chrono::steady_clock::time_point t, double radius, int armor_num,
    Eigen::VectorXd P0_dig);
  Target(double x, double vyaw, double radius, double h);
  void print_outpost_debug_info();
  void predict(std::chrono::steady_clock::time_point t);
  void predict(double dt);
  void update(const Armor & armor);
  //void check_abnormal_state(const Armor & measurement, int layer);
  Eigen::VectorXd ekf_x() const;
  const tools::ExtendedKalmanFilter & ekf() const;
  std::vector<Eigen::Vector4d> armor_xyza_list() const;

  bool diverged() const;

  bool convergened();

  bool isinit = false;

  bool checkinit();

private:
  int armor_num_;
  int switch_count_;
  int update_count_;

  bool is_switch_, is_converged_;

  tools::ExtendedKalmanFilter ekf_;
  std::chrono::steady_clock::time_point t_;

  void update_ypda(const Armor & armor, int id);  // yaw pitch distance angle
  
  // === 新增：前哨站处理逻辑 ===
  void handle_outpost_update(const Armor & armor);

  Eigen::Vector3d h_armor_xyz(const Eigen::VectorXd & x, int id) const;
  Eigen::MatrixXd h_jacobian(const Eigen::VectorXd & x, int id) const;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__TARGET_HPP