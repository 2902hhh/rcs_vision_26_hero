#include "shooter.hpp"

#include <yaml-cpp/yaml.h>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
Shooter::Shooter(const std::string & config_path) : last_command_{false, false, 0, 0}
{
  auto yaml = YAML::LoadFile(config_path);
  // 读取配置参数并转换为弧度 (degree -> rad)
  first_tolerance_ = yaml["first_tolerance"].as<double>() / 57.3;
  second_tolerance_ = yaml["second_tolerance"].as<double>() / 57.3;
  judge_distance_ = yaml["judge_distance"].as<double>();
  auto_fire_ = yaml["auto_fire"].as<bool>();
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
 

  // 2. 计算目标距离 (水平距离近似)
  auto target_x = targets.front().ekf_x()[0];
  auto target_y = targets.front().ekf_x()[2];
  auto distance = std::sqrt(tools::square(target_x) + tools::square(target_y));

  // 3. 动态选择容忍度 (Tolerance)
  // 近距离用 first_tolerance (大)，远距离用 second_tolerance (小)
  auto tolerance = distance > judge_distance_ ? second_tolerance_ : first_tolerance_;

  // 4. 计算各项误差
  // [稳定性] 本次解算指令 vs 上次解算指令 的差异 (Yaw轴)
  double yaw_cmd_diff = std::abs(last_command_.yaw - command.yaw);
  
  // [Yaw跟随误差] 当前云台实际Yaw (gimbal_pos[0]) vs 上次指令Yaw
  double yaw_aim_error = std::abs(gimbal_pos[0] - last_command_.yaw);

  // === 新增: [Pitch跟随误差] 当前云台实际Pitch (gimbal_pos[1]) vs 上次指令Pitch ===
  // 注意：gimbal_pos[1] 对应 Pitch 轴，last_command_.pitch 是 aim 算出的目标 Pitch
  double pitch_aim_error = std::abs(gimbal_pos[1] - last_command_.pitch);

  // 5. 状态判定
  // 命令稳定: Yaw 指令突变小于 2倍容忍度
  bool is_yaw_stable = yaw_cmd_diff < tolerance * 2;
  
  // Yaw 对准: 实际 Yaw 误差小于容忍度
  bool is_yaw_aimed = yaw_aim_error < tolerance;

  // === 新增: Pitch 对准 ===
  bool is_pitch_aimed = pitch_aim_error < tolerance;

  // 弹道有效: Aimer 解算成功
  bool is_valid = aimer.debug_aim_point.valid;

  // 6. 调试日志 (可选，防止刷屏可加计数器)
  static int debug_cnt = 0;
  if (debug_cnt++ % 100 == 0) { // 每100次调用打印一次
      tools::logger()->info(
          "[Shooter] Dist:{:.2f}m Tol:{:.3f} | YawErr:{:.3f} OK:{} | PitchErr:{:.3f} OK:{} | Fire:{}", 
          distance, tolerance, 
          yaw_aim_error, is_yaw_aimed, 
          pitch_aim_error, is_pitch_aimed,
          (is_yaw_stable && is_yaw_aimed && is_pitch_aimed && is_valid)
      );
  }

  // 7. 最终开火判据
  // 原逻辑: if (is_yaw_stable && is_yaw_aimed && is_valid)
  // 修改后: 加入 is_pitch_aimed
  if (is_yaw_stable && is_yaw_aimed && is_pitch_aimed && is_valid) {
    last_command_ = command;
    return true; // 允许开火
  }

  last_command_ = command;
  return false; // 禁止开火
}

}  // namespace auto_aim