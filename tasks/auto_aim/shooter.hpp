#ifndef AUTO_AIM__SHOOTER_HPP
#define AUTO_AIM__SHOOTER_HPP

#include <chrono>
#include <string>
#include <deque>

#include "io/command.hpp"
#include "tasks/auto_aim/aimer.hpp"

namespace auto_aim
{
class Shooter
{
public:
  Shooter(const std::string & config_path);

  bool shoot(
    const io::Command & command, const auto_aim::Aimer & aimer,
    const std::list<auto_aim::Target> & targets, const Eigen::Vector3d & gimbal_pos);

  // ========== 新增：更新帧时间队列 ==========
  void update_frame_time(double dt);

private:
  io::Command last_command_;
  double judge_distance_;
  double first_tolerance_;
  double second_tolerance_;
  bool auto_fire_;
  double fire_cooldown_;
  double fire_cooldown_arm_delay_;
  std::chrono::steady_clock::time_point last_fire_time_;
  bool cooldown_cycle_active_ = false;

  // ========== 新增：精确发射时机相关 ==========
  bool precision_mode_ = false;           // 是否启用精确发射模式
  double last_face_angle_ = 0.0;          // 上一帧装甲板角度
  bool first_precision_ = true;           // 是否首次进入精确模式
  bool be_shooted_ = false;               // 当前装甲板是否已发射
  int last_armor_id_ = -1;                // 上一帧目标装甲板ID
  bool precision_shoot_enabled_ = true;     // Whether precision shoot mode is enabled
  std::deque<double> frame_time_queue_;   // 帧时间队列
  static constexpr int queue_max_size_ = 10;

  // ========== 新增：精确发射方法 ==========
  // 寻找旋转圆与Y轴交点
  std::vector<Eigen::Vector2d> find_intersections(
      const Eigen::Vector2d& center, double radius) const;
  // 精确发射判断逻辑
  bool judging_precision_shoot(
      const Eigen::Vector4d& shoot_target,
      const Eigen::Vector2d& car_middle,
      double rotate_speed);
  // 获取平均帧时间
  double get_frame_time_average() const;
};
}  // namespace auto_aim

#endif  // AUTO_AIM__SHOOTER_HPP