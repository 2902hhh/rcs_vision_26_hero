#include "target.hpp"

#include <numeric>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
Target::Target(
  const Armor & armor, std::chrono::steady_clock::time_point t, double radius, int armor_num,
  Eigen::VectorXd P0_dig)
: name(armor.name),
  armor_type(armor.type),
  jumped(false),
  last_id(0),
  update_count_(0),
  armor_num_(armor_num),
  t_(t),
  is_switch_(false),
  is_converged_(false),
  switch_count_(0),
  outpost_initialized(false),
  outpost_base_height(0.0),
  outpost_layer(0)
{
  auto r = radius;
  priority = armor.priority;
  const Eigen::VectorXd & xyz = armor.xyz_in_world;
  const Eigen::VectorXd & ypr = armor.ypr_in_world;

  // 旋转中心的坐标
  auto center_x = xyz[0] + r * std::cos(ypr[0]);
  auto center_y = xyz[1] + r * std::sin(ypr[0]);
  auto center_z = xyz[2];

  // x vx y vy z vz a w r l h
  // a: angle
  // w: angular velocity
  // l: r2 - r1
  // h: z2 - z1
  Eigen::VectorXd x0{{center_x, 0, center_y, 0, center_z, 0, ypr[0], 0, r, 0, 0}};  //初始化预测量
  Eigen::MatrixXd P0 = P0_dig.asDiagonal();

  // 防止夹角求和出现异常值
  auto x_add = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a + b;
    c[6] = tools::limit_rad(c[6]);
    return c;
  };

  ekf_ = tools::ExtendedKalmanFilter(x0, P0, x_add);  //初始化滤波器（预测量、预测量协方差）
}

// void Target::check_abnormal_state(const Armor & measurement, int layer)
// {
//     // 阈值设定 (根据经验值设定，可微调)
//     const double MAX_YAW_ERROR = 0.15;  // 允许的最大角度误差 (弧度, 约8.5度)
//     const double MAX_OMEGA = 5.0;       // 前哨站最大转速 (rad/s)
//     const double MAX_POS_UNCERTAINTY = 0.5; // 位置协方差最大值
//     const double EXPECTED_RADIUS = 0.2765; 
//     bool is_abnormal = false;
//     std::string warning_msg = "[Diagnose] ";
//     // ==========================================
//     // 1. 检测角度残差 (最常见抖动原因)
//     // ==========================================
//     // 预测的装甲板角度 = 车中心角度(x[6]) + ID偏移
//     double pred_armor_yaw = tools::limit_rad(ekf_.x[6] + layer * 2.0 * CV_PI / 3.0);
//     double meas_armor_yaw = measurement.ypr_in_world[0];
    
//     // 计算偏差
//     double yaw_diff = std::abs(tools::limit_rad(meas_armor_yaw - pred_armor_yaw));
//     if (yaw_diff > MAX_YAW_ERROR) {
//         is_abnormal = true;
//         warning_msg += fmt::format("Yaw Diff High({:.3f} rad); ", yaw_diff);
//     }
//     // ==========================================
//     // 2. 检测角速度异常 (导致预测过头或不足)
//     // ==========================================
//     double omega = ekf_.x[7]; // 角速度
//     if (std::abs(omega) > MAX_OMEGA) {
//         is_abnormal = true;
//         warning_msg += fmt::format("Spin Fast({:.2f}); ", omega);
//     }
//     // ==========================================
//     // 3. 检测协方差发散 (说明滤波器“迷路”了)
//     // ==========================================
//     // x[0], x[2], x[4] 分别是 x, y, z 的位置
//     // P(0,0) 是 x 的方差
//     double pos_var = ekf_.P(0, 0) + ekf_.P(2, 2); 
//     if (pos_var > MAX_POS_UNCERTAINTY) {
//         is_abnormal = true;
//         warning_msg += fmt::format("Pos Unstable(Var: {:.2f}); ", pos_var);
//     }
//     // ==========================================
//     // 4. 检测半径是否偏离理论值 (导致深度不对)
//     // ==========================================
//     // 仅针对前哨站
//     double current_radius = ekf_.x[8];
//     if (std::abs(current_radius - EXPECTED_RADIUS) > 0.05) { // 误差超过 5cm
//         is_abnormal = true;
//         warning_msg += fmt::format("Radius Bad({:.3f}); ", current_radius);
//     }
//     // ==========================================
//     // 5. 输出报警
//     // ==========================================
//     if (is_abnormal && update_count_ > 10) { // 前10帧初始化时不报错
//         tools::logger()->warn("{} | Layer: {}", warning_msg, layer);
        
//         // 可选：在这里重置滤波器，或者降低卡尔曼增益
//         // reset(); 
//     }
// }


Target::Target(double x, double vyaw, double radius, double h) : armor_num_(4)
{
  Eigen::VectorXd x0{{x, 0, 0, 0, 0, 0, 0, vyaw, radius, 0, h}};
  Eigen::VectorXd P0_dig{{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
  Eigen::MatrixXd P0 = P0_dig.asDiagonal();

  // 防止夹角求和出现异常值
  auto x_add = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a + b;
    c[6] = tools::limit_rad(c[6]);
    return c;
  };

  ekf_ = tools::ExtendedKalmanFilter(x0, P0, x_add);  //初始化滤波器（预测量、预测量协方差）
}

void Target::predict(std::chrono::steady_clock::time_point t)
{
  auto dt = tools::delta_time(t, t_);
  predict(dt);
  t_ = t;
}

void Target::print_outpost_debug_info()
{
    if (name != ArmorName::outpost) return;

    // 1. 半径检查 (Radius)
    // 理想值 0.2765，如果偏差 > 2cm 说明没锁住
    double r = ekf_.x[8];
    bool r_stable = std::abs(r - 0.2765) < 0.02;

    // 2. 角速度检查 (Omega)
    // 前哨站通常转速在 0.4 ~ 0.8 rad/s 之间
    // 如果 > 2.0 说明发散，如果 ~0 说明没跟上
    double omega = ekf_.x[7];
    
    // 3. 角度残差检查 (Innovation)
    // 我们手动算一下：预测的 Layer 0 角度 vs 观测反解的 Layer 0 角度
    // 如果这个差值很大，说明 EKF 正在剧烈修正
    // 注意：这里需要拿到最新的 armor 和 id，通常在 update 函数里算比较方便，这里只能打印状态
    
    // 4. 高度基准 (Base Height)
    double base_h = outpost_base_height;

    // 5. 当前层级 (Layer)
    int layer = outpost_layer;

    // 6. EKF 协方差 (P Matrix) - 检查收敛度
    // P(6,6) 是 Yaw 的方差，P(7,7) 是 Omega 的方差
    double p_yaw = ekf_.P(6, 6);
    double p_omega = ekf_.P(7, 7);

    // === 打印日志 ===
    // 格式: [Outpost] L:层级 | R:半径(状态) | W:角速度 | BaseZ:基准高 | Py:Yaw方差
    tools::logger()->info(
        "[Outpost] L:{} | R:{:.4f} ({}) | W:{:.3f} | BaseZ:{:.3f} | P_Yaw:{:.5f}",
        layer, 
        r, (r_stable ? "OK" : "BAD"), 
        omega, 
        base_h, 
        p_yaw
    );

    // 报警逻辑
    if (!r_stable) {
        tools::logger()->warn("  >>> RADIUS DRIFT! Force Reset Recommended.");
    }
    if (p_yaw > 0.1) { // 0.1 rad^2 很大了
        tools::logger()->warn("  >>> YAW UNSTABLE! P_Yaw > 0.1");
    }
}



void Target::predict(double dt)
{
  // 状态转移矩阵
  // clang-format off
  Eigen::MatrixXd F{
    {1, dt,  0,  0,  0,  0,  0,  0,  0,  0,  0},
    {0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0},
    {0,  0,  1, dt,  0,  0,  0,  0,  0,  0,  0},
    {0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0},
    {0,  0,  0,  0,  1, dt,  0,  0,  0,  0,  0},
    {0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0},
    {0,  0,  0,  0,  0,  0,  1, dt,  0,  0,  0},
    {0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0},
    {0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0},
    {0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0},
    {0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1}
  };
  // clang-format on

  // Piecewise White Noise Model
  // https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/07-Kalman-Filter-Math.ipynb
  double v1, v2;
  if (name == ArmorName::outpost) {
    v1 = 1e-3;   // 前哨站加速度方差
    v2 = 100;  // 前哨站角加速度方差
  } else {
    v1 = 100;  // 加速度方差
    v2 = 400;  // 角加速度方差
  }
  auto a = dt * dt * dt * dt / 4;
  auto b = dt * dt * dt / 2;
  auto c = dt * dt;
  // 预测过程噪声偏差的方差
  // clang-format off
  Eigen::MatrixXd Q{
    {a * v1, b * v1,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {b * v1, c * v1,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0, a * v1, b * v1,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0, b * v1, c * v1,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0, a * v1, b * v1,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0, b * v1, c * v1,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0, a * v2, b * v2, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0, b * v2, c * v2, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0}
  };
  // clang-format on

  // 防止夹角求和出现异常值
  auto f = [&](const Eigen::VectorXd & x) -> Eigen::VectorXd {
    Eigen::VectorXd x_prior = F * x;
    x_prior[6] = tools::limit_rad(x_prior[6]);
    return x_prior;
  };

  // 前哨站转速软饱和：仅超过阈值时平滑限制，低转速零干预
  if (this->convergened() && this->name == ArmorName::outpost) {
    constexpr double max_omega = 2.51;
    constexpr double threshold = 2.0;
    if (std::abs(ekf_.x[7]) > threshold) {
      double sign = (ekf_.x[7] > 0) ? 1.0 : -1.0;
      double excess = std::abs(ekf_.x[7]) - threshold;
      ekf_.x[7] = sign * (threshold + (max_omega - threshold) * std::tanh(excess));
    }
  }

  ekf_.predict(F, Q, f);
}

void Target::update(const Armor & armor)
{
    // === 修改：如果是前哨站，走特殊逻辑 ===
  if (this->name == ArmorName::outpost) {
      handle_outpost_update(armor);
      return; // 前哨站逻辑处理完直接返回，跳过常规流程
  }
  // ===================================

  // 装甲板匹配
  int id;
  auto min_angle_error = 1e10;
  const std::vector<Eigen::Vector4d> & xyza_list = armor_xyza_list();

  std::vector<std::pair<Eigen::Vector4d, int>> xyza_i_list;
  for (int i = 0; i < armor_num_; i++) {
    xyza_i_list.push_back({xyza_list[i], i});
  }

  std::sort(
    xyza_i_list.begin(), xyza_i_list.end(),
    [](const std::pair<Eigen::Vector4d, int> & a, const std::pair<Eigen::Vector4d, int> & b) {
      Eigen::Vector3d ypd1 = tools::xyz2ypd(a.first.head(3));
      Eigen::Vector3d ypd2 = tools::xyz2ypd(b.first.head(3));
      return ypd1[2] < ypd2[2];
    });

  // 取前3个distance最小的装甲板
  for (int i = 0; i < 3; i++) {
    const auto & xyza = xyza_i_list[i].first;
    Eigen::Vector3d ypd = tools::xyz2ypd(xyza.head(3));
    auto angle_error = std::abs(tools::limit_rad(armor.ypr_in_world[0] - xyza[3])) +
                       std::abs(tools::limit_rad(armor.ypd_in_world[0] - ypd[0]));

    if (std::abs(angle_error) < std::abs(min_angle_error)) {
      id = xyza_i_list[i].second;
      min_angle_error = angle_error;
    }
  }

  if (id != 0) jumped = true;

  if (id != last_id) {
    is_switch_ = true;
  } else {
    is_switch_ = false;
  }

  if (is_switch_) switch_count_++;

  last_id = id;
  update_count_++;

  update_ypda(armor, id);
}

// === 新增：前哨站处理逻辑 ===
// 在 target.cpp 中替换 handle_outpost_update

void Target::handle_outpost_update(const Armor & armor)
{
    double current_z = armor.xyz_in_world[2];
    double current_yaw = armor.ypr_in_world[0];

    // ==========================================
    // 1. 初始化基准高度 (保持你的原逻辑)
    // ==========================================
    if (!outpost_initialized) {
      outpost_z_history_.push_back(current_z);
      if (outpost_z_history_.size() > 30) {
        std::sort(outpost_z_history_.begin(), outpost_z_history_.end());
        // 取较小值作为基准
        outpost_base_height = outpost_z_history_[6];
        outpost_initialized = true;
        outpost_z_history_.clear();
        tools::logger()->info("Outpost Init Base: {:.3f}", outpost_base_height);

        // 初始化时强制重置半径
        ekf_.x[8] = 0.2765;
      }

      // 初始化阶段也按相位匹配当前观测板，避免硬编码 id=0 导致先验偏置
      int init_id = 0;
      double pred_yaw_base = ekf_.x[6];
      double min_yaw_diff = 1e10;
      for (int id = 0; id < 3; ++id) {
        double yaw_if_id = tools::limit_rad(current_yaw + id * 2.0 * CV_PI / 3.0);
        double diff_val = std::abs(tools::limit_rad(yaw_if_id - pred_yaw_base));
        if (diff_val < min_yaw_diff) {
          min_yaw_diff = diff_val;
          init_id = id;
        }
      }

      update_ypda(armor, init_id);
      update_count_++;
      return;
    }

    // ==========================================
    // 2. 层级判定：相位主导，高度兜底
    // ==========================================
    // 仅靠 current_z 会受“单块板自转导致的高度起伏”影响，容易在 0/1/2 层间跳变。
    // 这里改为：收敛后优先用 yaw 相位判层，Z 只用于排除明显不合理结果。

    // 先算一个高度候选层（仅做兜底）
    double diff = current_z - outpost_base_height;
    int raw_layer = std::round(diff / OUTPOST_HEIGHT_DIFF);
    int height_layer = std::max(0, std::min(2, raw_layer));

    // 再算相位候选层（主判据）
    double pred_yaw_base = ekf_.x[6];
    int yaw_layer = last_id;
    double min_yaw_diff = 1e10;
    for (int id = 0; id < 3; ++id) {
      double yaw_if_id = tools::limit_rad(current_yaw + id * 2.0 * CV_PI / 3.0);
      double diff_val = std::abs(tools::limit_rad(yaw_if_id - pred_yaw_base));
      if (diff_val < min_yaw_diff) {
        min_yaw_diff = diff_val;
        yaw_layer = id;
      }
    }

    int final_layer = height_layer;

    // 早期阶段（刚初始化）先用高度，避免相位尚未稳定时误判
    if (update_count_ > 5) {
      final_layer = yaw_layer;

      // 只在“相位结果被 Z 严重反对”时回退到高度候选
      // 0.25m 约等于 2.5 层，阈值更宽，避免 Z 抖动频繁抢回控制权。
      double z_if_yaw = current_z - final_layer * OUTPOST_HEIGHT_DIFF;
      double z_error = std::abs(z_if_yaw - outpost_base_height);
      if (z_error > 0.25) {
        final_layer = height_layer;
      }

      if (final_layer != yaw_layer && update_count_ % 20 == 0) {
        tools::logger()->debug(
          "Outpost Layer Fallback: Yaw-ID {} -> Z-ID {}, z_err={:.3f}", yaw_layer,
          height_layer, z_error);
      }
    }

    // ==========================================
    // 3.5 层级迟滞：候选层需连续若干帧一致才切换
    // ==========================================
    if (final_layer == last_id) {
      outpost_layer_candidate_ = final_layer;
      outpost_layer_candidate_count_ = 0;
    } else {
      if (outpost_layer_candidate_ != final_layer) {
        outpost_layer_candidate_ = final_layer;
        outpost_layer_candidate_count_ = 1;
      } else {
        outpost_layer_candidate_count_++;
      }

      if (outpost_layer_candidate_count_ < OUTPOST_LAYER_HYST_FRAMES) {
        final_layer = last_id;
      } else {
        final_layer = outpost_layer_candidate_;
        outpost_layer_candidate_count_ = 0;
      }
    }

    // 更新基准高度 (仅当 ID 判定可信时)
    // Layer 0 时更激进校准（直接观测基准层），其他层级慢速跟随
    if (final_layer >= 0 && final_layer <= 2) {
        double estimated_base = current_z - final_layer * OUTPOST_HEIGHT_DIFF;
        double alpha = (final_layer == 0) ? 0.05 : 0.01;
        outpost_base_height = (1.0 - alpha) * outpost_base_height + alpha * estimated_base;
    }

    // 更新状态
    this->outpost_layer = final_layer;
    if (final_layer != last_id) is_switch_ = true;
    last_id = final_layer;

    // ==========================================
    // 5. 修正观测值并送入 EKF
    // ==========================================
    Armor virtual_armor = armor;
    // 强行将 Z 轴拉回 Layer 0 的平面
    virtual_armor.xyz_in_world[2] = current_z - final_layer * OUTPOST_HEIGHT_DIFF;
    // 重新计算 ypd (距离 Distance 和 Pitch 会因此改变，这很重要)
    virtual_armor.ypd_in_world = tools::xyz2ypd(virtual_armor.xyz_in_world);

    // 强制锁定半径 (你的原逻辑)
    ekf_.x[8] = 0.2765;
    ekf_.P(8, 8) = 1e-10; 

    update_ypda(virtual_armor, final_layer);
    //check_abnormal_state(armor, final_layer);
    update_count_++;

    print_outpost_debug_info();
}


void Target::update_ypda(const Armor & armor, int id)
{
  Eigen::MatrixXd H = h_jacobian(ekf_.x, id);

  // === 1. 计算观测噪声 R ===
  Eigen::MatrixXd R;

  if (name == ArmorName::outpost) {
      // --- 前哨站专用 R ---
      // 计算角度残差
      double yaw_diff = tools::limit_rad(armor.ypr_in_world[0] - ekf_.x[6] + id * 2 * CV_PI / armor_num_);
      
      double r_yaw = 0.015;//1e-2
      double r_pitch = 1e-1; // 高度噪声大一点
      double r_dist = 1e-1;
      double r_angle = 5e-2 + std::abs(yaw_diff) * 5.0; // 自适应角度噪声

      Eigen::VectorXd R_dig{{r_yaw, r_pitch, r_dist, r_angle}};
      R = R_dig.asDiagonal();
  } 
  else {
      // --- 原有逻辑 (兼容普通装甲板) ---
      // 原代码计算 delta_angle 的方式
      auto center_yaw = std::atan2(armor.xyz_in_world[1], armor.xyz_in_world[0]);
      auto delta_angle = tools::limit_rad(armor.ypr_in_world[0] - center_yaw);
      
      Eigen::VectorXd R_dig{
        {4e-3, 4e-3, 
         log(std::abs(delta_angle) + 1) + 1,
         log(std::abs(armor.ypd_in_world[2]) + 1) / 200 + 9e-2}
      };
      R = R_dig.asDiagonal();
  }

  // === 2. 定义观测方程 h ===
  auto h = [&](const Eigen::VectorXd & x) -> Eigen::Vector4d {
    Eigen::VectorXd xyz = h_armor_xyz(x, id);
    Eigen::VectorXd ypd = tools::xyz2ypd(xyz);
    
    // 注意：这里需要区分 顺时针/逆时针 或者 通用逻辑
    // 如果普通装甲板也是顺时针排布 (120度)，用减号没问题
    // 如果普通装甲板没有顺序要求 (比如步兵4块板)，通常用加号
    // 建议：统一用减号，或者在这里再加个 if
    double angle;
    if (name == ArmorName::outpost) {
        angle = tools::limit_rad(x[6] - id * 2 * CV_PI / armor_num_); // 顺时针
    } else {
        // 假设步兵/英雄是逆时针排布 (0, 1, 2, 3)
        angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_); 
    }
    
    return {ypd[0], ypd[1], ypd[2], angle};
  };

  // === 3. 定义残差计算 z_subtract ===
  auto z_subtract = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a - b;
    c[0] = tools::limit_rad(c[0]);
    c[1] = tools::limit_rad(c[1]);
    c[3] = tools::limit_rad(c[3]);
    return c;
  };

  // === 4. 构造观测向量 z ===
  const Eigen::VectorXd & ypd = armor.ypd_in_world;
  // ypr[0] 是装甲板的世界系 Yaw
  Eigen::VectorXd z{{ypd[0], ypd[1], ypd[2], armor.ypr_in_world[0]}}; 

  // === 5. 更新 ===
  ekf_.update(z, H, R, h, z_subtract);
}

Eigen::VectorXd Target::ekf_x() const { return ekf_.x; }

const tools::ExtendedKalmanFilter & Target::ekf() const { return ekf_; }

std::vector<Eigen::Vector4d> Target::armor_xyza_list() const
{
  std::vector<Eigen::Vector4d> _armor_xyza_list;

  for (int i = 0; i < armor_num_; i++) {
    auto angle = tools::limit_rad(ekf_.x[6] - i * 2 * CV_PI / armor_num_);
    Eigen::Vector3d xyz = h_armor_xyz(ekf_.x, i);
    _armor_xyza_list.push_back({xyz[0], xyz[1], xyz[2], angle});
  }
  return _armor_xyza_list;
}

bool Target::diverged() const
{
  auto r_ok = ekf_.x[8] > 0.05 && ekf_.x[8] < 0.5;

  auto l_ok = ekf_.x[8] + ekf_.x[9] > 0.05 && ekf_.x[8] + ekf_.x[9] < 0.5;

   if (r_ok && l_ok) return false;
  //if (r_ok) return false;
  tools::logger()->debug("[Target] r={:.3f}, l={:.3f}", ekf_.x[8], ekf_.x[9]);
  return true;
}

bool Target::convergened()
{
  if (this->name != ArmorName::outpost && update_count_ > 3 && !this->diverged()) {
    is_converged_ = true;
  }

  //前哨站特殊判断
  if (this->name == ArmorName::outpost && update_count_ > 10 && !this->diverged()) {
    is_converged_ = true;
  }

  return is_converged_;
}

// 计算出装甲板中心的坐标（考虑长短轴）
Eigen::Vector3d Target::h_armor_xyz(const Eigen::VectorXd & x, int id) const
{
  auto angle = tools::limit_rad(x[6] - id * 2 * CV_PI / armor_num_);
  auto use_l_h = (armor_num_ == 4) && (id == 1 || id == 3);

  auto r = (use_l_h) ? x[8] + x[9] : x[8];
  auto armor_x = x[0] - r * std::cos(angle);
  auto armor_y = x[2] - r * std::sin(angle);
  auto armor_z = (use_l_h) ? x[4] + x[10] : x[4];

  return {armor_x, armor_y, armor_z};
}

Eigen::MatrixXd Target::h_jacobian(const Eigen::VectorXd & x, int id) const
{
  auto angle = tools::limit_rad(x[6] - id * 2 * CV_PI / armor_num_);
  auto use_l_h = (armor_num_ == 4) && (id == 1 || id == 3);

  auto r = (use_l_h) ? x[8] + x[9] : x[8];
  auto dx_da = r * std::sin(angle);
  auto dy_da = -r * std::cos(angle);

  auto dx_dr = -std::cos(angle);
  auto dy_dr = -std::sin(angle);
  auto dx_dl = (use_l_h) ? -std::cos(angle) : 0.0;
  auto dy_dl = (use_l_h) ? -std::sin(angle) : 0.0;

  auto dz_dh = (use_l_h) ? 1.0 : 0.0;

  // clang-format off
  Eigen::MatrixXd H_armor_xyza{
    {1, 0, 0, 0, 0, 0, dx_da, 0, dx_dr, dx_dl,     0},
    {0, 0, 1, 0, 0, 0, dy_da, 0, dy_dr, dy_dl,     0},
    {0, 0, 0, 0, 1, 0,     0, 0,     0,     0, dz_dh},
    {0, 0, 0, 0, 0, 0,     1, 0,     0,     0,     0}
  };
  // clang-format on

  Eigen::VectorXd armor_xyz = h_armor_xyz(x, id);
  Eigen::MatrixXd H_armor_ypd = tools::xyz2ypd_jacobian(armor_xyz);
  // clang-format off
  Eigen::MatrixXd H_armor_ypda{
    {H_armor_ypd(0, 0), H_armor_ypd(0, 1), H_armor_ypd(0, 2), 0},
    {H_armor_ypd(1, 0), H_armor_ypd(1, 1), H_armor_ypd(1, 2), 0},
    {H_armor_ypd(2, 0), H_armor_ypd(2, 1), H_armor_ypd(2, 2), 0},
    {                0,                 0,                 0, 1}
  };
  // clang-format on

  return H_armor_ypda * H_armor_xyza;
}

bool Target::checkinit() { return isinit; }

}  // namespace auto_aim
