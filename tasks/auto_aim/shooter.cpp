#include "shooter.hpp"

#include <yaml-cpp/yaml.h>
#include <algorithm>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/debug_monitor.hpp"

namespace auto_aim
{
Shooter::Shooter(const std::string & config_path)
: last_command_{false, false, 0, 0}, last_fire_time_{}
{
  auto yaml = YAML::LoadFile(config_path);
  // 读取配置参数并转换为弧度 (degree -> rad)
  first_tolerance_ = yaml["first_tolerance"].as<double>() / 57.3;
  second_tolerance_ = yaml["second_tolerance"].as<double>() / 57.3;
  judge_distance_ = yaml["judge_distance"].as<double>();
  auto_fire_ = yaml["auto_fire"].as<bool>();
  fire_cooldown_ = yaml["fire_cooldown"].as<double>();
  fire_cooldown_arm_delay_ =
    yaml["fire_cooldown_arm_delay"] ? yaml["fire_cooldown_arm_delay"].as<double>() : 0.01;
}

bool Shooter::shoot(
  const io::Command & command, const auto_aim::Aimer & aimer,
  const std::list<auto_aim::Target> & targets, const Eigen::Vector3d & gimbal_pos)
{
  // 强制打印，确认函数被调用了
  //tools::logger()->info("Shooter called!");

  // 1. 基础条件检查：如果不控制、无目标或未开启自动开火，直接返回 false
  if (!command.control || targets.empty() || !auto_fire_)
  {
      //打印具体是哪个条件没满足
        tools::logger()->info("Shooter Early Return: Ctrl={} Targets={} Auto={}",
        command.control, targets.size(), auto_fire_);

         return false;
  }

  // 1.5 开火静默：开火后先等待 fire_cooldown_arm_delay_ 秒，再进入 fire_cooldown_ 秒静默
  auto now = std::chrono::steady_clock::now();
  if (cooldown_cycle_active_) {
    double since_last_fire = std::chrono::duration<double>(now - last_fire_time_).count();
    double cooldown_start = fire_cooldown_arm_delay_;
    double cooldown_end = fire_cooldown_arm_delay_ + fire_cooldown_;
    if (since_last_fire >= cooldown_start && since_last_fire < cooldown_end) {
      last_command_ = command;
      return false;
    }
    if (since_last_fire >= cooldown_end) {
      cooldown_cycle_active_ = false;
    }
  }

  auto target = targets.front();
  auto ekf_x = target.ekf_x();
  double rotate_speed_rpm = std::abs(ekf_x[7]) * 30 / CV_PI;

  // ========== 新增：高速小陀螺精确发射模式 ==========
  if (rotate_speed_rpm > 60 && !aimer.get_aim_preview()) {
    precision_mode_ = true;

    auto armor_list = target.armor_xyza_list();
    if (armor_list.size() < 2) {
      precision_mode_ = false;
    } else {
      Eigen::Vector2d car_middle(ekf_x[0], ekf_x[2]);

      // 按距离排序装甲板
      std::vector<std::pair<Eigen::Vector4d, int>> sorted;
      for (size_t i = 0; i < armor_list.size(); i++) {
        sorted.push_back({armor_list[i], (int)i});
      }
      std::sort(sorted.begin(), sorted.end(),
        [](const auto& a, const auto& b) {
          return Eigen::Vector2d(a.first[0], a.first[1]).norm() <
                 Eigen::Vector2d(b.first[0], b.first[1]).norm();
        });

      // 根据旋转方向选择目标装甲板
      Eigen::Vector4d left_point = sorted[0].first[0] < sorted[1].first[0] ?
        sorted[0].first : sorted[1].first;
      Eigen::Vector4d right_point = sorted[0].first[0] >= sorted[1].first[0] ?
        sorted[0].first : sorted[1].first;
      Eigen::Vector4d shoot_target = ekf_x[7] > 0 ? left_point : right_point;
      int current_armor_id = ekf_x[7] > 0 ?
        (sorted[0].first[0] < sorted[1].first[0] ? sorted[0].second : sorted[1].second) :
        (sorted[0].first[0] >= sorted[1].first[0] ? sorted[0].second : sorted[1].second);

      // ========== 装甲板切换检测 ==========
      if (current_armor_id != last_armor_id_) {
        be_shooted_ = false;  // 新装甲板，重置发射标志
        last_armor_id_ = current_armor_id;
      }

      // ========== 精确发射判断 ==========
      if (!be_shooted_) {
        bool can_shoot = judging_precision_shoot(shoot_target, car_middle, ekf_x[7]);
        if (can_shoot) {
          be_shooted_ = true;
          if (!cooldown_cycle_active_) {
            last_fire_time_ = now;
            cooldown_cycle_active_ = true;
          }
          last_command_ = command;
          WATCH("precision_shoot", 1);
          return true;
        }
      }

      WATCH("precision_mode", 1);
      WATCH("precision_shoot", 0);
      last_command_ = command;
      return false;
    }
  }
  else {
    precision_mode_ = false;
    be_shooted_ = false;  // 退出精确模式时重置
  }

  // 2. 计算目标距离 (水平距离近似)
  auto target_x = ekf_x[0];
  auto target_y = ekf_x[2];
  auto distance = std::sqrt(tools::square(target_x) + tools::square(target_y));

  // 3. 动态选择容忍度 (Tolerance)
  // 近距离用 first_tolerance (大)，远距离用 second_tolerance (小)
  auto tolerance = distance > judge_distance_ ? second_tolerance_ : first_tolerance_;

  // 4. 计算各项误差
  // [稳定性] 本次解算指令 vs 上次解算指令 的差异 (Yaw轴)
  double yaw_cmd_diff = std::abs(last_command_.yaw - command.yaw);

  // [Yaw跟随误差] 当前云台实际Yaw (gimbal_pos[0]) vs 上次指令Yaw
  double yaw_aim_error = std::abs(gimbal_pos[0] - last_command_.yaw);
  WATCH("gimbal_yaw_deg", gimbal_pos[0] * 57.3);
  // === 新增: [Pitch跟随误差] 当前云台实际Pitch (gimbal_pos[1]) vs 上次指令Pitch ===
  // 注意：gimbal_pos[1] 对应 Pitch 轴，last_command_.pitch 是 aim 算出的目标 Pitch
  double pitch_aim_error = std::abs(gimbal_pos[1] - last_command_.pitch);

  // 5. 状态判定
  // 命令稳定: Yaw 指令突变小于 2倍容忍度
  bool is_yaw_stable = yaw_cmd_diff < tolerance * 2;

  // Yaw 对准: 实际 Yaw 误差小于容忍度
  bool is_yaw_aimed = yaw_aim_error < 0.3/57.3; // 固定0.2度的Yaw对准要求，防止过于宽松导致误伤

  // === 新增: Pitch 对准 ===
  bool is_pitch_aimed = pitch_aim_error < tolerance;

  // 弹道有效: Aimer 解算成功
  bool is_valid = aimer.debug_aim_point.valid;

  // 6. 调试日志 (可选，防止刷屏可加计数器)
  static int debug_cnt = 0;
  if (debug_cnt++ % 100 == 0) { // 每100次调用打印一次
      tools::logger()->info(
          "[Shooter] Dist:{:.2f}m Tol:{:.3f} | YawErr:{:.3f} OK:{} | PitchErr:{:.3f} OK:{} | RPM={:.1f}",
          distance, tolerance*57.3,
          yaw_aim_error, is_yaw_aimed,
          pitch_aim_error, is_pitch_aimed,
          rotate_speed_rpm
      );
  }
  WATCH("yaw_diff",yaw_aim_error*57.3);
  WATCH("pitch_diff",pitch_aim_error*57.3);
  WATCH("precision_mode", 0);

  // 7. 最终开火判据
  // 原逻辑: if (is_yaw_stable && is_yaw_aimed && is_valid)
  // 修改后: 加入 is_pitch_aimed
  if (is_yaw_stable && is_yaw_aimed && is_pitch_aimed && is_valid) {
    if (!cooldown_cycle_active_) {
      last_fire_time_ = now;
      cooldown_cycle_active_ = true;
    }
    last_command_ = command;
    return true; // 允许开火
  }

  last_command_ = command;
  return false; // 禁止开火
}

// ========== 更新帧时间队列 ==========
void Shooter::update_frame_time(double dt)
{
  if (frame_time_queue_.size() >= queue_max_size_) {
    frame_time_queue_.pop_front();
  }
  frame_time_queue_.push_back(dt);
}

// ========== 获取平均帧时间 ==========
double Shooter::get_frame_time_average() const
{
  if (frame_time_queue_.empty()) return 0.01;
  double sum = 0;
  for (const auto& t : frame_time_queue_) sum += t;
  return sum / frame_time_queue_.size();
}

// ========== 寻找旋转圆与Y轴交点 ==========
std::vector<Eigen::Vector2d> Shooter::find_intersections(
    const Eigen::Vector2d& center, double radius) const
{
  std::vector<Eigen::Vector2d> intersections;

  // 圆与Y轴（x=0）的交点
  // (0 - center.x)^2 + (y - center.y)^2 = radius^2
  if (radius * radius >= center.x() * center.x()) {
    double dy = std::sqrt(radius * radius - center.x() * center.x());
    intersections.push_back(Eigen::Vector2d(0, center.y() + dy));
    if (dy > 1e-6) {
      intersections.push_back(Eigen::Vector2d(0, center.y() - dy));
    }
  }
  return intersections;
}

// ========== 计算三点角度 ==========
double Shooter::angle_abc(const Eigen::Vector2d& A, const Eigen::Vector2d& B, const Eigen::Vector2d& C) const
{
  Eigen::Vector2d BA = A - B;
  Eigen::Vector2d BC = C - B;
  double dot = BA.dot(BC);
  double norm_product = BA.norm() * BC.norm();
  if (norm_product < 1e-6) return 0.0;
  double cos_theta = std::clamp(dot / norm_product, -1.0, 1.0);
  return std::acos(cos_theta);
}

// ========== 精确发射判断逻辑 ==========
bool Shooter::judging_precision_shoot(
    const Eigen::Vector4d& shoot_target,
    const Eigen::Vector2d& car_middle,
    double rotate_speed)
{
  bool turn_right = rotate_speed > 0;
  bool target_left = shoot_target[0] < 0;

  // 目标还未进入射击区域（枪口右侧的装甲板对于右旋转来说还未到位）
  if ((turn_right ^ target_left) == 1) {
    return false;
  }

  Eigen::Vector2d shoot_target2d(shoot_target[0], shoot_target[1]);
  double radius = (car_middle - shoot_target2d).norm();

  // ========== 找旋转圆与Y轴（枪口线）的交点 ==========
  auto intersections = find_intersections(car_middle, radius);
  if (intersections.size() != 2) return false;

  // 取枪口前方的交点
  Eigen::Vector2d near_intersection = intersections[0].y() < intersections[1].y() ?
    intersections[0] : intersections[1];

  // ========== 计算装甲板到最佳位置的角度差 ==========
  double included_angle = angle_abc(shoot_target2d, car_middle, near_intersection);

  // ========== 核心：计算理想发射等待时间 ==========
  double perfect_shoot_wait_time = included_angle / std::abs(rotate_speed);

  // 如果等待时间大于半帧时间，不允许发射
  double avg_frame_time = get_frame_time_average();
  if (perfect_shoot_wait_time > avg_frame_time * 0.5 || perfect_shoot_wait_time < 0) {
    return false;
  }

  return true;  // 允许发射
}

}  // namespace auto_aim
