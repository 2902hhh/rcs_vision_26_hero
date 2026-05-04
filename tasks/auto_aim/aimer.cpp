#include "aimer.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <vector>
#include "tools/debug_monitor.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/trajectory.hpp"

namespace auto_aim
{
Aimer::Aimer(const std::string & config_path)
: left_yaw_offset_(std::nullopt), right_yaw_offset_(std::nullopt)
{
  auto yaml = YAML::LoadFile(config_path);
  yaw_offset_ = yaml["yaw_offset"].as<double>() / 57.3;        // degree to rad
  pitch_offset_ = yaml["pitch_offset"].as<double>() / 57.3;    // degree to rad
  comming_angle_ = yaml["comming_angle"].as<double>() / 57.3;  // degree to rad
  leaving_angle_ = yaml["leaving_angle"].as<double>() / 57.3;  // degree to rad
  high_speed_delay_time_ = yaml["high_speed_delay_time"].as<double>();
  low_speed_delay_time_ = yaml["low_speed_delay_time"].as<double>();
  decision_speed_ = yaml["decision_speed"].as<double>();
  use_manual_rotate_speed_ = yaml["use_manual_rotate_speed"].as<bool>(false);
  manual_rotate_speed_ = yaml["manual_rotate_speed"].as<double>(2.5);
  spin_enter_speed_ = yaml["spin_enter_speed"].as<double>(2.4);
  spin_exit_speed_ = yaml["spin_exit_speed"].as<double>(1.8);
  spin_speed_lpf_alpha_ = yaml["spin_speed_lpf_alpha"].as<double>(0.25);
  spin_speed_lpf_alpha_ = std::clamp(spin_speed_lpf_alpha_, 0.01, 1.0);
  if (spin_exit_speed_ > spin_enter_speed_) {
    spin_exit_speed_ = spin_enter_speed_;
  }
  if (yaml["left_yaw_offset"].IsDefined() && yaml["right_yaw_offset"].IsDefined()) {
    left_yaw_offset_ = yaml["left_yaw_offset"].as<double>() / 57.3;    // degree to rad
    right_yaw_offset_ = yaml["right_yaw_offset"].as<double>() / 57.3;  // degree to rad
    tools::logger()->info("[Aimer] successfully loading shootmode");
  }

  // ========== 新增：预瞄配置参数 ==========
  static_track_face_angle_ = yaml["static_track_face_angle"].as<double>(15.0) / 57.3; // 默认15度
  max_stability_track_rotate_speed_ = yaml["max_stability_track_rotate_speed"].as<double>(3.0); // 默认3 rad/s

  // ========== 新增：策略类型选择 ==========
  std::string strategy_str = yaml["spin_strategy"].as<std::string>("adaptive");
  if (strategy_str == "preview") {
    spin_strategy_ = SpinStrategy::preview;
    tools::logger()->info("[Aimer] Spin strategy: preview (强制预瞄)");
  } else if (strategy_str == "coming_leaving") {
    spin_strategy_ = SpinStrategy::coming_leaving;
    tools::logger()->info("[Aimer] Spin strategy: coming_leaving (强制Coming/Leaving)");
  } else if (strategy_str == "shoot_middle") {
    spin_strategy_ = SpinStrategy::shoot_middle;
    tools::logger()->info("[Aimer] Spin strategy: shoot_middle (瞄准车辆中心)");
  } else {
    spin_strategy_ = SpinStrategy::adaptive;
    tools::logger()->info("[Aimer] Spin strategy: adaptive (自适应)");
  }

  // ========== 新增：shoot_middle 模式配置参数 ==========
  min_shoot_middle_rpm_ = yaml["min_shoot_middle_rpm"].as<double>(60.0);
  max_shoot_middle_rpm_ = yaml["max_shoot_middle_rpm"].as<double>(90.0);
  min_shoot_middle_distance_ = yaml["min_shoot_middle_distance"].as<double>(1.0);
  max_shoot_middle_distance_ = yaml["max_shoot_middle_distance"].as<double>(3.2);
}

io::Command Aimer::aim(
  std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
  bool to_now)
{
  if (targets.empty()) return {false, false, 0, 0};
  auto target = targets.front();

  auto ekf = target.ekf();
  double observed_rotate_speed = target.ekf_x()[7];
  double effective_rotate_speed =
    use_manual_rotate_speed_ ? manual_rotate_speed_ : observed_rotate_speed;
  double delay_time = std::abs(effective_rotate_speed) > decision_speed_ ? high_speed_delay_time_
                                                                         : low_speed_delay_time_;

  // 针对英雄机器人大弹丸射速较低的情况进行保护，避免除以0或弹道无解
  if (bullet_speed < 10) bullet_speed = 12;
  //bullet_speed = 11.8;
   WATCH("bullet_speed", bullet_speed);
  // 考虑detecor和tracker所消耗的时间，此外假设aimer的用时可忽略不计
  auto future = timestamp;
  if (to_now) {
    double dt;
    dt = tools::delta_time(std::chrono::steady_clock::now(), timestamp) + delay_time;
    future += std::chrono::microseconds(int(dt * 1e6));
    target.predict(future);
  }

  else {
    auto dt = 0.005 + delay_time;  //detector-aimer耗时0.005+发弹延时0.1
    // tools::logger()->info("dt is {:.4f} second", dt);
    future += std::chrono::microseconds(int(dt * 1e6));
    target.predict(future);
  }

  auto aim_point0 = choose_aim_point(target);
  debug_aim_point = aim_point0;
  if (!aim_point0.valid) {
    // tools::logger()->debug("Invalid aim_point0.");
    return {false, false, 0, 0};
  }

  Eigen::Vector3d xyz0 = aim_point0.xyza.head(3);
  auto d0 = std::sqrt(xyz0[0] * xyz0[0] + xyz0[1] * xyz0[1]);
  tools::Trajectory trajectory0(bullet_speed, d0, xyz0[2]);
  if (trajectory0.unsolvable) {
    tools::logger()->debug(
      "[Aimer] Unsolvable trajectory0: {:.2f} {:.2f} {:.2f}", bullet_speed, d0, xyz0[2]);
    debug_aim_point.valid = false;
    return {false, false, 0, 0};
  }

  // 迭代求解飞行时间 (最多10次，收敛条件：相邻两次fly_time差 <0.001)
  bool converged = false;
  double prev_fly_time = trajectory0.fly_time;
  tools::Trajectory current_traj = trajectory0;
  std::vector<Target> iteration_target(10, target);  // 创建10个目标副本用于迭代预测

  for (int iter = 0; iter < 10; ++iter) {
    // 预测目标在 future + prev_fly_time 时刻的位置
    auto predict_time = future + std::chrono::microseconds(static_cast<int>(prev_fly_time * 1e6));
    iteration_target[iter].predict(predict_time);

    // 计算瞄准点
    auto aim_point = choose_aim_point(iteration_target[iter]);
    debug_aim_point = aim_point;
    if (!aim_point.valid) {
      return {false, false, 0, 0};
    }

    // 计算新弹道
    Eigen::Vector3d xyz = aim_point.xyza.head(3);
    double d = std::sqrt(xyz.x() * xyz.x() + xyz.y() * xyz.y());
    current_traj = tools::Trajectory(bullet_speed, d, xyz.z());

    // 检查弹道是否可解
    if (current_traj.unsolvable) {
      tools::logger()->debug(
        "[Aimer] Unsolvable trajectory in iter {}: speed={:.2f}, d={:.2f}, z={:.2f}", iter + 1,
        bullet_speed, d, xyz.z());
      debug_aim_point.valid = false;
      return {false, false, 0, 0};
    }

    // 检查收敛条件
    if (std::abs(current_traj.fly_time - prev_fly_time) < 0.001) {
      converged = true;
      break;
    }
    prev_fly_time = current_traj.fly_time;
  }

  // 计算最终角度 (单位：弧度)
  Eigen::Vector3d final_xyz = debug_aim_point.xyza.head(3);
  double yaw = std::atan2(final_xyz.y(), final_xyz.x()) + yaw_offset_;
  double pitch = -(current_traj.pitch + pitch_offset_);  //世界坐标系下pitch向上为负

  // ==================== [新增] 计算前馈速度 (Feedforward Velocity) ====================
  // 利用 EKF 预测出的线性速度 (vx, vy, vz) 计算云台 Yaw/Pitch 轴所需的角速度
  // 这能显著减小对移动目标的跟随滞后
  
  // 1. 获取目标的世界坐标和速度 (来自 EKF 状态: x, vx, y, vy, z, vz...)
  auto x_state = target.ekf_x(); 
  double x = x_state[0];
  double vx = x_state[1];
  double y = x_state[2];
  double vy = x_state[3];
  double z = x_state[4];
  double vz = x_state[5];

  // 2. 计算 Yaw 轴角速度 (rad/s)
  // 公式: d(atan2(y, x))/dt = (x*vy - y*vx) / (x^2 + y^2)
  double dist_sq = x * x + y * y;
  double yaw_vel = 0.0;
  if (dist_sq > 1e-6) {
      yaw_vel = (x * vy - y * vx) / dist_sq;
  }

  // 3. 计算 Pitch 轴角速度 (rad/s)
  // 简化模型近似：pitch ≈ atan2(z, r) -> d(pitch)/dt = (r*vz - z*vr) / (r^2 + z^2)
  // 其中 vr (径向速度) = (x*vx + y*vy) / r
  double r = std::sqrt(dist_sq);
  double pitch_vel = 0.0;
  if (r > 1e-6) {
      double vr = (x * vx + y * vy) / r;
      double dist_3d_sq = dist_sq + z * z;
      if (dist_3d_sq > 1e-6) {
          // 注意符号：如果 pitch 定义为向上为负，这里需要反号
          // 这里的 current_traj.pitch 是正值代表向上，所以导数也是正值代表向上速度
          // 最终 pitch = -(traj + offset)，所以 pitch_vel 也要取反
          double raw_pitch_vel = (r * vz - z * vr) / dist_3d_sq;
          pitch_vel = -raw_pitch_vel; 
      }
  }

  // 4. 加速度 (Acceleration) - 暂时设为0，二阶导数噪声较大，通常速度前馈已足够
  double yaw_acc = 0.0;
    if (dist_sq > 1e-6) {
      // 径向速度项 (Position dot Velocity)
      double pos_dot_vel = x * vx + y * vy;
      // alpha = -2 * omega * (r . v) / r^2
      yaw_acc = -2.0 * yaw_vel * pos_dot_vel / dist_sq;
  }
  double pitch_acc = 0.0;

  // === [新增] 动态软件补偿 (针对左右运动不对称) ===
  // 现象：右->左准，左->右偏左(落后)。
  // 策略：当目标向右运动 (yaw_vel < 0，注意坐标系定义) 时，额外增加 Yaw 偏移。
  // 注意：RoboMaster 常用坐标系是 右手系，逆时针为正。
  // 目标向右跑 -> 角度变小 -> yaw_vel < 0。
  
  // 假设 yaw_vel < -0.1 (向右显著运动)
  // 如果落后（打在左边），说明 yaw 太大了（或者太小？视坐标系而定）。
  // 通常：向右跑，我们需要让 yaw 更小（更靠右）。如果打在左边，说明 yaw 还不够小。
  // 所以需要减去一个补偿值。
  
  // double dynamic_yaw_fix = 0.0;
  // if (yaw_vel < -0.1) { // 目标向右运动
  //     // 补偿 0.5度 (约 0.008 rad)
  //     // 这个值需要你实测微调：如果还是偏左，就加大；如果偏右了，就减小。
  //     dynamic_yaw_fix = -1.6 / 57.3; 
  // } else if (yaw_vel > 0.1) { // 目标向左运动
  //     // 如果向左本来就准，设为 0。如果向左打在右边(落后)，则需要加上一个正值。
  //     dynamic_yaw_fix = 0.0; 
  // }
  
  // yaw += dynamic_yaw_fix;
  // ===============================================



  // 5. 构造并返回 Command (需确保 io::Command 结构体已包含 vel/acc 字段)
  io::Command cmd;
  cmd.control = true;
  cmd.shoot = false; // shoot 标志在 shooter.cpp 中计算，这里先给 false
  cmd.yaw = yaw;
  cmd.pitch = pitch;
  cmd.yaw_vel = yaw_vel;
  cmd.pitch_vel = pitch_vel;
  // 用 yaw_acc 的异常值告诉下位机当前目标处于小陀螺状态。
  // gimbal.send(bool, ...) 会把 rad/s^2 转成 deg/s^2，因此这里先换算成弧度域的等价值。
  cmd.yaw_acc = spin_mode_ ? (999.0 / 57.29578) : yaw_acc;
  cmd.pitch_acc = pitch_acc;
  WATCH("target_yaw_deg", yaw * 57.3);
  // 填充调试用的目标状态信息 (可选)
  // cmd.target_x = x;
  // cmd.target_y = y;
  // cmd.target_z = z;
  // cmd.target_vx = vx;
  // cmd.target_vy = vy;
  // cmd.target_vz = vz;

  return cmd;
}

io::Command Aimer::aim(
  std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
  io::ShootMode shoot_mode, bool to_now)
{
  double yaw_offset;
  if (shoot_mode == io::left_shoot && left_yaw_offset_.has_value()) {
    yaw_offset = left_yaw_offset_.value();
  } else if (shoot_mode == io::right_shoot && right_yaw_offset_.has_value()) {
    yaw_offset = right_yaw_offset_.value();
  } else {
    yaw_offset = yaw_offset_;
  }

  auto command = aim(targets, timestamp, bullet_speed, to_now);

  command.yaw = command.yaw - yaw_offset_ + yaw_offset;

  return command;
}

AimPoint Aimer::choose_aim_point(const Target & target)
{
  Eigen::VectorXd ekf_x = target.ekf_x();
  std::vector<Eigen::Vector4d> armor_xyza_list = target.aim_armor_xyza_list();
  int armor_num = static_cast<int>(armor_xyza_list.size());

  // 兜底保护：预瞄逻辑至少需要两块装甲板
  if (armor_num < 2) {
    aim_preview_ = false;
    spin_mode_ = false;
    if (armor_num == 1) return {true, armor_xyza_list[0]};
    return {false, Eigen::Vector4d::Zero()};
  }
  // 如果装甲板未发生过跳变，则只有当前装甲板的位置已知
  if (target.name != ArmorName::outpost && !target.jumped) {
    aim_preview_ = false;
    spin_mode_ = false;
    return {true, armor_xyza_list[0]};
  }

  // 整车旋转中心的球坐标yaw
  auto center_yaw = std::atan2(ekf_x[2], ekf_x[0]);
  Eigen::Vector2d car_middle(ekf_x[0], ekf_x[2]);

  // 如果delta_angle为0，则该装甲板中心和整车中心的连线在世界坐标系的xy平面过原点
  std::vector<double> delta_angle_list;
  for (int i = 0; i < armor_num; i++) {
    auto delta_angle = tools::limit_rad(armor_xyza_list[i][3] - center_yaw);
    delta_angle_list.emplace_back(delta_angle);
  }

  WATCH("rad", target.ekf_x()[7]);
  double effective_rotate_speed =
    use_manual_rotate_speed_ ? manual_rotate_speed_ : ekf_x[7];


  double raw_rotate_speed_abs = std::abs(effective_rotate_speed);
  if (!filtered_rotate_speed_abs_.has_value()) {
    filtered_rotate_speed_abs_ = raw_rotate_speed_abs;
  } else {
    filtered_rotate_speed_abs_ = spin_speed_lpf_alpha_ * raw_rotate_speed_abs +
                                 (1.0 - spin_speed_lpf_alpha_) * filtered_rotate_speed_abs_.value();
  }
  double rotate_speed_abs = filtered_rotate_speed_abs_.value();

  // 前哨站静止/转动迟滞判定（极简实现：仅在瞄准层判断，不改 Target 结构）
  constexpr double OUTPOST_STATIC_OMEGA_THRESH = 0.15;
  constexpr int OUTPOST_STATIC_ENTER_COUNT = 15;
  constexpr int OUTPOST_STATIC_EXIT_COUNT = 5;
  static bool outpost_is_static = false;
  static int outpost_transition_count = 0;

  if (target.name == ArmorName::outpost) {
    if (rotate_speed_abs < OUTPOST_STATIC_OMEGA_THRESH) {
      if (!outpost_is_static) {
        outpost_transition_count++;
        if (outpost_transition_count >= OUTPOST_STATIC_ENTER_COUNT) {
          outpost_is_static = true;
          outpost_transition_count = 0;
        }
      } else {
        outpost_transition_count = 0;
      }
    } else {
      if (outpost_is_static) {
        outpost_transition_count++;
        if (outpost_transition_count >= OUTPOST_STATIC_EXIT_COUNT) {
          outpost_is_static = false;
          outpost_transition_count = 0;
        }
      } else {
        outpost_transition_count = 0;
      }
    }

    // 转动态按小陀螺处理，静止态按非小陀螺处理
    spin_mode_ = !outpost_is_static;
  } else if (spin_mode_) {
    outpost_is_static = false;
    outpost_transition_count = 0;
    if (rotate_speed_abs < spin_exit_speed_) spin_mode_ = false;
  } else {
    outpost_is_static = false;
    outpost_transition_count = 0;
    if (rotate_speed_abs > spin_enter_speed_) spin_mode_ = true;
  }

  // 前哨站旋转态下，后续策略使用固定角速度 2.5 rad/s（保留旋转方向）
  constexpr double OUTPOST_SPIN_OMEGA = 2.5;
  if (target.name == ArmorName::outpost && spin_mode_) {
    double rotate_sign = (effective_rotate_speed >= 0) ? 1.0 : -1.0;
    effective_rotate_speed = rotate_sign * OUTPOST_SPIN_OMEGA;
    rotate_speed_abs = OUTPOST_SPIN_OMEGA;
  }

  // WATCH("rotate_speed_raw", raw_rotate_speed_abs);
  // WATCH("rotate_speed_filtered", rotate_speed_abs);
  // WATCH("outpost_spin_omega_used", rotate_speed_abs);
  // WATCH("outpost_is_static", outpost_is_static ? 1 : 0);
  // WATCH("outpost_static_count", outpost_transition_count);
  // WATCH("spin_mode", spin_mode_ ? 1 : 0);

  WATCH("rad_effective", effective_rotate_speed);

  int outpost_lowest_id = -1;
  if (target.name == ArmorName::outpost) {
    outpost_lowest_id = target.lowest_plate_id();
    if (outpost_lowest_id < 0 || outpost_lowest_id >= armor_num) {
      int track_id = std::clamp(target.last_id, 0, armor_num - 1);
      return {true, armor_xyza_list[track_id], false};
    }

    if (!spin_mode_) {
      aim_preview_ = false;
      return {true, armor_xyza_list[outpost_lowest_id], true};
    }

    aim_preview_ = false;
    Eigen::Vector3d lowest_xyz = armor_xyza_list[outpost_lowest_id].head(3);
    double lowest_dist = std::hypot(lowest_xyz.x(), lowest_xyz.y());
    double center_dist = std::hypot(ekf_x[0], ekf_x[2]);
    double aim_z = lowest_xyz.z();
    if (lowest_dist > 1e-6) {
      aim_z = lowest_xyz.z() / lowest_dist * center_dist;
    }
    WATCH("outpost_center_aim_z", aim_z);
    WATCH("outpost_lowest_z", lowest_xyz.z());
    return {true, Eigen::Vector4d(ekf_x[0], ekf_x[2], aim_z, 0), true};
  }

  // ========== 策略1：非小陀螺 (转速 < 2 rad/s) ==========
  if (!spin_mode_) {
    aim_preview_ = false;
    // 选择在可射击范围内的装甲板
    std::vector<int> id_list;
    for (int i = 0; i < armor_num; i++) {
      if (std::abs(delta_angle_list[i]) > 60 / 57.3) continue;
      id_list.push_back(i);
    }
    // 绝无可能
    if (id_list.empty()) {
      tools::logger()->warn("Empty id list!");
      return {false, armor_xyza_list[0]};
    }

    // 锁定模式：防止在两个都呈45度的装甲板之间来回切换
    if (id_list.size() > 1) {
      int id0 = id_list[0], id1 = id_list[1];

      // 未处于锁定模式时，选择delta_angle绝对值较小的装甲板，进入锁定模式
      if (lock_id_ != id0 && lock_id_ != id1)
        lock_id_ = (std::abs(delta_angle_list[id0]) < std::abs(delta_angle_list[id1])) ? id0 : id1;

      return {true, armor_xyza_list[lock_id_]};
    }

    // 只有一个装甲板在可射击范围内时，退出锁定模式
    lock_id_ = -1;
    return {true, armor_xyza_list[id_list[0]]};
  }

  // ========== 策略2：小陀螺 - 启用预瞄机制 ==========
  double rotate_speed_rpm = rotate_speed_abs * 30 / CV_PI;

  // ========== 预瞄角度计算 ==========
  double track_face_angle;
  if (rotate_speed_rpm < 60) {
    // 中速：自适应计算追踪角度
    track_face_angle = adaptive_calculate_track_face_angle(rotate_speed_abs);
  } else {
    // 高速：使用静态追踪角度
    track_face_angle = static_track_face_angle_;
  }

  // 计算两块最近装甲板与枪口的角度
  // 按距离排序
  std::vector<std::pair<Eigen::Vector4d, int>> sorted_armors;
  for (int i = 0; i < armor_num; i++) {
    sorted_armors.push_back({armor_xyza_list[i], i});
  }
  std::sort(sorted_armors.begin(), sorted_armors.end(),
    [](const auto& a, const auto& b) {
      return Eigen::Vector2d(a.first[0], a.first[1]).norm() <
             Eigen::Vector2d(b.first[0], b.first[1]).norm();
    });

  double shortest_face_angle = calculate_angle_abs(
    Eigen::Vector2d::Zero(), car_middle,
    Eigen::Vector2d(sorted_armors[0].first[0], sorted_armors[0].first[1]));
  double next_face_angle = calculate_angle_abs(
    Eigen::Vector2d::Zero(), car_middle,
    Eigen::Vector2d(sorted_armors[1].first[0], sorted_armors[1].first[1]));

  // ========== 策略选择：根据配置决定使用哪种策略 ==========
  bool use_preview = false;
  bool use_shoot_middle = false;

  if (spin_strategy_ == SpinStrategy::preview) {
    use_preview = true;
  } else if (spin_strategy_ == SpinStrategy::coming_leaving) {
    use_preview = false;
  } else if (spin_strategy_ == SpinStrategy::shoot_middle) {
    // 普通小陀螺可按配置瞄车体中心
    use_shoot_middle = true;
  } else {
    // 自适应模式：根据条件自动选择
    double distance_m = car_middle.norm();  // 单位：米

    // 优先判断是否应该使用 shoot_middle
    if (should_use_shoot_middle(rotate_speed_rpm, distance_m)) {
      use_shoot_middle = true;
    } else {
      // 原有逻辑：根据角度判断
      use_preview = (track_face_angle < shortest_face_angle && track_face_angle < next_face_angle);
    }
  }

  // ========== shoot_middle 模式处理 ==========
  if (use_shoot_middle) {
    aim_preview_ = false;

    Eigen::Vector4d middle_aim = calculate_middle_aim_point(target, armor_xyza_list);

    // WATCH("shoot_middle_mode", 1);
    // WATCH("middle_aim_x", middle_aim[0]);
    // WATCH("middle_aim_y", middle_aim[1]);

    return {true, middle_aim};
  }

  if (use_preview) {
    aim_preview_ = true;

    if (target.name == ArmorName::outpost) {
      double height = armor_xyza_list[outpost_lowest_id][2];
      double radius = ekf_x[8];
      double rotate_angle = effective_rotate_speed > 0 ?
        (CV_PI - track_face_angle) : (CV_PI + track_face_angle);
      Eigen::Vector2d aim_point2d = calculate_rotate_point2d(car_middle, radius, rotate_angle);

      WATCH("aim_preview", 1);
      WATCH("track_face_angle_deg", track_face_angle * 57.3);

      return {true, Eigen::Vector4d(aim_point2d.x(), aim_point2d.y(), height, 0), true};
    }

    // 确定左右装甲板
    Eigen::Vector4d left_point, right_point;
    if (sorted_armors[0].first[0] < sorted_armors[1].first[0]) {
      left_point = sorted_armors[0].first;
      right_point = sorted_armors[1].first;
    } else {
      left_point = sorted_armors[1].first;
      right_point = sorted_armors[0].first;
    }

    // 确定高度（根据旋转方向）
    double height = effective_rotate_speed > 0 ? left_point[2] : right_point[2];
    double radius = (target.name == ArmorName::outpost) ? ekf_x[8] : (ekf_x[8] + ekf_x[9]) / 2;

    // ========== 核心：计算预瞄点位置 ==========
    // 顺时针：CV_PI - track_face_angle
    // 逆时针：CV_PI + track_face_angle
    double rotate_angle = effective_rotate_speed > 0 ?
      (CV_PI - track_face_angle) : (CV_PI + track_face_angle);
    Eigen::Vector2d aim_point2d = calculate_rotate_point2d(car_middle, radius, rotate_angle);

    WATCH("aim_preview", 1);
    WATCH("track_face_angle_deg", track_face_angle * 57.3);

    return {true, Eigen::Vector4d(aim_point2d.x(), aim_point2d.y(), height, 0)};
  }
  // ========== 不需要预瞄：直接瞄准最近装甲板 ==========
  else {
    aim_preview_ = false;
    // 使用现有的coming/leaving角度策略
    double coming_angle = comming_angle_;
    double leaving_angle = leaving_angle_;
    if (target.name == ArmorName::outpost) {
      coming_angle = 70 / 57.3;
      leaving_angle = 30 / 57.3;
      if (std::abs(delta_angle_list[outpost_lowest_id]) <= coming_angle) {
        if ((effective_rotate_speed > 0 && delta_angle_list[outpost_lowest_id] < leaving_angle) ||
            (effective_rotate_speed < 0 &&
             delta_angle_list[outpost_lowest_id] > -leaving_angle)) {
          return {true, armor_xyza_list[outpost_lowest_id], true};
        }
      }
      return {true, armor_xyza_list[outpost_lowest_id], false};
    }

    for (int i = 0; i < armor_num; i++) {
      if (std::abs(delta_angle_list[i]) > coming_angle) continue;
      if (effective_rotate_speed > 0 && delta_angle_list[i] < leaving_angle)
        return {true, armor_xyza_list[i]};
      if (effective_rotate_speed < 0 && delta_angle_list[i] > -leaving_angle)
        return {true, armor_xyza_list[i]};
    }
    return {false, armor_xyza_list[0]};
  }
}

// ========== 计算旋转后的预瞄点位置 ==========
Eigen::Vector2d Aimer::calculate_rotate_point2d(
    const Eigen::Vector2d& car_middle, double radius, double rotate_angle) const
{
  // 归一化方向向量
  double magnitude = car_middle.norm();
  if (magnitude < 1e-6) return car_middle;

  Eigen::Vector2d normalized = car_middle / magnitude;
  // 延伸到装甲板位置
  Eigen::Vector2d extended = car_middle + normalized * radius;

  // 旋转变换
  Eigen::Vector2d translate = extended - car_middle;
  double cos_theta = std::cos(rotate_angle);
  double sin_theta = std::sin(rotate_angle);

  Eigen::Vector2d rotated;
  rotated.x() = translate.x() * cos_theta - translate.y() * sin_theta;
  rotated.y() = translate.x() * sin_theta + translate.y() * cos_theta;
  rotated += car_middle;

  return rotated;
}

// ========== 计算三点角度（取绝对值）==========
double Aimer::calculate_angle_abs(
    const Eigen::Vector2d& A, const Eigen::Vector2d& B, const Eigen::Vector2d& C) const
{
  Eigen::Vector2d BA = A - B;
  Eigen::Vector2d BC = C - B;
  double dot = BA.dot(BC);
  double norm_product = BA.norm() * BC.norm();
  if (norm_product < 1e-6) return 0.0;

  double cos_theta = std::clamp(dot / norm_product, -1.0, 1.0);
  double angle = std::acos(cos_theta);
  if (angle > CV_PI / 2) angle = angle - CV_PI;
  return std::abs(angle);
}

// ========== 自适应计算追踪角度（三分法）==========
double Aimer::adaptive_calculate_track_face_angle(double rotate_speed_abs) const
{
  int iteration = 15;
  double left = 0;
  double right = CV_PI / 4; // 最大45度

  while (iteration--) {
    double m1 = left + (right - left) / 3;
    double m2 = right - (right - left) / 3;
    if (calculate_rotate_cost(m1, rotate_speed_abs) < calculate_rotate_cost(m2, rotate_speed_abs)) {
      right = m2;
    } else {
      left = m1;
    }
  }
  return (right + left) / 2;
}

// ========== 计算旋转代价函数 ==========
double Aimer::calculate_rotate_cost(double track_face_angle, double rotate_speed_abs) const
{
  // 装甲板切换时，一块装甲板到下一块的时间
  double switch_angle = CV_PI / 2 - 2 * track_face_angle;
  double switch_need_time = switch_angle / rotate_speed_abs;
  // 云台追踪需要的时间
  double track_need_time = track_face_angle * 2 / max_stability_track_rotate_speed_;
  return std::abs(switch_need_time - track_need_time);
}

// ========== 计算车辆几何中心瞄准点 ==========
Eigen::Vector4d Aimer::calculate_middle_aim_point(
    const Target& target,
    const std::vector<Eigen::Vector4d>& armor_xyza_list) const
{
  Eigen::VectorXd ekf_x = target.ekf_x();

  // 1. 获取车辆中心位置
  double car_center_x = ekf_x[0];  // 车辆中心 x
  double car_center_y = ekf_x[2];  // 车辆中心 y (世界坐标系)

  // 2. 找到最近的装甲板（用于获取高度）
  auto armor_num = armor_xyza_list.size();
  size_t nearest_idx = 0;
  double min_dist = 1e10;
  for (size_t i = 0; i < armor_num; i++) {
    double dist = Eigen::Vector2d(armor_xyza_list[i][0], armor_xyza_list[i][1]).norm();
    if (dist < min_dist) {
      min_dist = dist;
      nearest_idx = i;
    }
  }

  // 3. 瞄准点: x/y 取车辆中心，z(高度)取最近装甲板高度
  double aim_x = car_center_x;
  double aim_y = car_center_y;
  double aim_z = armor_xyza_list[nearest_idx][2];  // 使用最近装甲板的高度

  return Eigen::Vector4d(aim_x, aim_y, aim_z, 0);
}

// ========== 判断是否应该使用 shoot_middle 模式 ==========
bool Aimer::should_use_shoot_middle(double rotate_speed_rpm, double distance_m) const
{
  // 条件1: 转速在中间范围
  bool rpm_in_range = (rotate_speed_rpm >= min_shoot_middle_rpm_ &&
                       rotate_speed_rpm <= max_shoot_middle_rpm_);

  // 条件2: 距离超出最佳跟踪范围
  bool distance_out_of_range = (distance_m < min_shoot_middle_distance_ ||
                                 distance_m > max_shoot_middle_distance_);

  // 满足任一条件即启用 shoot_middle
  return rpm_in_range || distance_out_of_range;
}

}  // namespace auto_aim
