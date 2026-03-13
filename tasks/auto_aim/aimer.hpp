#ifndef AUTO_AIM__AIMER_HPP
#define AUTO_AIM__AIMER_HPP

#include <Eigen/Dense>
#include <chrono>
#include <list>

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

  // ========== 新增：预瞄相关成员 ==========
  bool aim_preview_ = false;              // 是否处于预瞄模式
  double static_track_face_angle_;        // 静态追踪角度（从配置读取）
  double max_stability_track_rotate_speed_; // 云台最大稳定追踪角速度

  AimPoint choose_aim_point(const Target & target);

  // ========== 新增：预瞄相关方法 ==========
  // 计算旋转后的预瞄点位置
  Eigen::Vector2d calculate_rotate_point2d(
      const Eigen::Vector2d& car_middle, double radius, double rotate_angle) const;
  // 计算三点角度（取绝对值）
  double calculate_angle_abs(
      const Eigen::Vector2d& A, const Eigen::Vector2d& B, const Eigen::Vector2d& C) const;
  // 自适应计算追踪角度（三分法）
  double adaptive_calculate_track_face_angle(double rotate_speed_abs) const;
  // 计算旋转代价函数
  double calculate_rotate_cost(double track_face_angle, double rotate_speed_abs) const;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__AIMER_HPP