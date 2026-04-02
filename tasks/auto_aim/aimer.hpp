#ifndef AUTO_AIM__AIMER_HPP
#define AUTO_AIM__AIMER_HPP

#include <Eigen/Dense>
#include <chrono>
#include <list>
#include <optional>

#include "io/cboard.hpp"
#include "io/gimbal/gimbal.hpp"
#include "io/command.hpp"
#include "target.hpp"

namespace auto_aim
{

struct AimPoint
{
  bool valid;
  Eigen::Vector4d xyza;
};

class Aimer
{
public:
  AimPoint debug_aim_point;
  explicit Aimer(const std::string & config_path);
  io::Command aim(
    std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
    bool to_now = true);

  io::Command aim(
    std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
    io::ShootMode shoot_mode, bool to_now = true);

  // ========== 新增：获取预瞄状态（供Shooter使用）==========
  bool get_aim_preview() const { return aim_preview_; }

private:
  double yaw_offset_;
  std::optional<double> left_yaw_offset_, right_yaw_offset_;
  double pitch_offset_;
  double comming_angle_;
  double leaving_angle_;
  double lock_id_ = -1;
  double high_speed_delay_time_;
  double low_speed_delay_time_;
  double decision_speed_;
  bool use_manual_rotate_speed_ = false;
  double manual_rotate_speed_ = 2.5;
  double spin_enter_speed_;
  double spin_exit_speed_;
  double spin_speed_lpf_alpha_;
  bool spin_mode_ = false;
  std::optional<double> filtered_rotate_speed_abs_;

  // ========== 新增：预瞄相关成员 ==========
  bool aim_preview_ = false;              // 是否处于预瞄模式
  double static_track_face_angle_;        // 静态追踪角度（从配置读取）
  double max_stability_track_rotate_speed_; // 云台最大稳定追踪角速度

  // ========== 新增：策略类型选择 ==========
  enum class SpinStrategy {
    preview,        // 预瞄模式
    coming_leaving, // Coming/Leaving模式
    shoot_middle,   // 瞄准车辆中心模式
    adaptive        // 自适应模式（根据转速自动选择）
  };
  SpinStrategy spin_strategy_;                // 策略类型

  // ========== shoot_middle 模式参数 ==========
  double min_shoot_middle_rpm_;      // 最小瞄准中心转速阈值 (RPM)
  double max_shoot_middle_rpm_;      // 最大瞄准中心转速阈值 (RPM)
  double min_shoot_middle_distance_; // 最小距离阈值 (m)
  double max_shoot_middle_distance_; // 最大距离阈值 (m)

  AimPoint choose_aim_point(const Target & target);

  // ========== 新增：预瞄相关方法 ==========
  // 计算旋转后的预瞄点位置
  Eigen::Vector2d calculate_rotate_point2d(
      const Eigen::Vector2d& car_middle, double radius, double rotate_angle) const;
  // 自适应计算追踪角度（三分法）
  double adaptive_calculate_track_face_angle(double rotate_speed_abs) const;
  // 计算旋转代价函数
  double calculate_rotate_cost(double track_face_angle, double rotate_speed_abs) const;

  // ========== shoot_middle 模式方法 ==========
  // 计算车辆几何中心瞄准点
  Eigen::Vector4d calculate_middle_aim_point(
      const Target& target,
      const std::vector<Eigen::Vector4d>& armor_xyza_list) const;

  // 判断是否应该使用 shoot_middle 模式
  bool should_use_shoot_middle(double rotate_speed_rpm, double distance_m) const;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__AIMER_HPP