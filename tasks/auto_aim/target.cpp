#include "target.hpp"

#include <algorithm>
#include <numeric>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
Target::Target(
  const Armor & armor, std::chrono::steady_clock::time_point t, double radius, int armor_num,
  Eigen::VectorXd P0_dig, bool use_ukf)
: name(armor.name),
  armor_type(armor.type),
  jumped(false),
  last_id(0),
  update_count_(0),
  armor_num_(armor_num),
  t_(t),
  is_switch_(false),
  is_converged_(false),
  switch_count_(0)
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
  // 前哨站：x[9]=id=0高差偏移, x[10]=id=2高差偏移, x[6]偏移+2π/3匹配首帧id=1
  double init_l = 0, init_h = 0;
  double init_angle = ypr[0];
  if (name == ArmorName::outpost) {
    init_l = -0.10;
    init_h = 0.10;
    init_angle = ypr[0] + 2.0 * CV_PI / 3.0;
  }
  Eigen::VectorXd x0{{center_x, 0, center_y, 0, center_z, 0, init_angle, 0, r, init_l, init_h}};
  Eigen::MatrixXd P0 = P0_dig.asDiagonal();

  // 防止夹角求和出现异常值
  auto x_add = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a + b;
    c[6] = tools::limit_rad(c[6]);
    return c;
  };

  ekf_ = tools::ExtendedKalmanFilter(x0, P0, x_add, use_ukf);  //初始化滤波器（预测量、预测量协方差）
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


Target::Target(double x, double vyaw, double radius, double h, bool use_ukf) : armor_num_(4)
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

  ekf_ = tools::ExtendedKalmanFilter(x0, P0, x_add, use_ukf);  //初始化滤波器（预测量、预测量协方差）
}

void Target::predict(std::chrono::steady_clock::time_point t)
{
  auto dt = tools::delta_time(t, t_);
  last_predict_dt_ = std::max(dt, 1e-3);
  predict(dt);
  t_ = t;
}





void Target::predict(double dt)
{
  last_predict_dt_ = std::max(dt, 1e-3);
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
    v1 = 10;  // 加速度方差
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
    {     0,      0,      0,      0,      0,      0, b * v2, c * v2,    0,    0,    0},
    {     0,      0,      0,      0,      0,      0,      0,      0, 1e-4,    0,    0},
    {     0,      0,      0,      0,      0,      0,      0,      0,    0, 1e-4,    0},
    {     0,      0,      0,      0,      0,      0,      0,      0,    0,    0, 1e-4}
  };
  // clang-format on

  // 防止夹角求和出现异常值
  auto f = [&](const Eigen::VectorXd & x) -> Eigen::VectorXd {
    Eigen::VectorXd x_prior = F * x;
    x_prior[6] = tools::limit_rad(x_prior[6]);
    return x_prior;
  };

  // 前哨站转速特判
  if (this->convergened() && this->name == ArmorName::outpost && std::abs(this->ekf_.x[7]) > 2)
    this->ekf_.x[7] = this->ekf_.x[7] > 0 ? 2.51 : -2.51;

  // 前哨站半径锁定（固定机械结构，不允许 EKF 估计）
  if (this->name == ArmorName::outpost) {
    this->ekf_.x[8] = 0.2765;
    this->ekf_.P(8, 8) = 1e-10;
    this->ekf_.x[5] = 0.0;
  }

  ekf_.predict(F, Q, f);
}

void Target::update(const Armor & armor)
{
  int id = 0;

  if (name == ArmorName::outpost) {
    constexpr double OUTPOST_Z_MATCH_GATE = 0.12;
    constexpr int OUTPOST_REJECT_REANCHOR_COUNT = 5;
    double min_z_error = 1e10;

    // 前哨站高度匹配：id=0 用 x[9], id=1 为基准(0), id=2 用 x[10]
    double z_offsets[3] = {ekf_.x[9], 0.0, ekf_.x[10]};
    for (int i = 0; i < 3; i++) {
      double predicted_z = ekf_.x[4] + z_offsets[i];
      double z_error = std::abs(armor.xyz_in_world[2] - predicted_z);
      if (z_error < min_z_error) {
        min_z_error = z_error;
        id = i;
      }
    }

    // z 门控：超门限不再硬拒绝，改为软更新，连续超限后重锚高度
    if (min_z_error > OUTPOST_Z_MATCH_GATE) {
      outpost_reject_count_++;
      if (outpost_reject_count_ >= OUTPOST_REJECT_REANCHOR_COUNT) {
        ekf_.x[4] = armor.xyz_in_world[2] - z_offsets[id];
        ekf_.x[5] = 0.0;
        outpost_reject_count_ = 0;
        tools::logger()->warn(
          "[Outpost] z reanchor: id={}, z_obs={:.3f}, z_anchor={:.3f}", id,
          armor.xyz_in_world[2], ekf_.x[4]);
      }
      tools::logger()->warn(
        "[Outpost] z soft-reject: id={}, z_err={:.4f}, z_obs={:.3f}, z_pred={:.3f}, cnt={}", id,
        min_z_error, armor.xyz_in_world[2], ekf_.x[4] + z_offsets[id], outpost_reject_count_);
    } else {
      outpost_reject_count_ = 0;
    }

    tools::logger()->debug(
      "[Outpost] match: id={}, z_err={:.4f}, x[9]={:.4f}, x[10]={:.4f}, x[4]={:.3f}", id,
      min_z_error, ekf_.x[9], ekf_.x[10], ekf_.x[4]);
  } else {
    auto min_angle_error = 1e10;
    const std::vector<Eigen::Vector4d> & xyza_list = armor_xyza_list();
    for (int i = 0; i < armor_num_; i++) {
      const auto & xyza = xyza_list[i];
      Eigen::Vector3d ypd = tools::xyz2ypd(xyza.head(3));
      auto angle_error = std::abs(tools::limit_rad(armor.ypr_in_world[0] - xyza[3])) +
                         std::abs(tools::limit_rad(armor.ypd_in_world[0] - ypd[0]));
      if (std::abs(angle_error) < std::abs(min_angle_error)) {
        id = i;
        min_angle_error = angle_error;
      }
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

void Target::update_ypda(const Armor & armor, int id)
{
  Eigen::MatrixXd H = h_jacobian(ekf_.x, id);
  const double w_before_update = ekf_.x[7];

  // === 1. 计算观测噪声 R ===
  Eigen::MatrixXd R;

  if (name == ArmorName::outpost) {
      // --- 前哨站专用 R ---
      double r_yaw = 0.015;
      double r_pitch = 1e-3;
      double r_dist = 1e-1;
      double r_angle = 0.1;  // 固定角度噪声，替代自适应

      Eigen::VectorXd R_dig{{r_yaw, r_pitch, r_dist, r_angle}};
      R = R_dig.asDiagonal();
  } 
  else {
      // --- 原有逻辑 (兼容普通装甲板) ---
      // 原代码计算 delta_angle 的方式
      auto center_yaw = std::atan2(armor.xyz_in_world[1], armor.xyz_in_world[0]);
      auto delta_angle = tools::limit_rad(armor.ypr_in_world[0] - center_yaw);
      // 增大 yaw/pitch 观测噪声以抑制抖动 (原值 6e-3 过小)
      Eigen::VectorXd R_dig{
        {4e-2, 4e-2,  //4e-3
        log(std::abs(delta_angle) + 1) + 1,
         log(std::abs(armor.ypd_in_world[2]) + 1) / 200 + 9e-2}
      };
      R = R_dig.asDiagonal();
  }

  // === 2. 定义观测方程 h ===
  auto h = [&](const Eigen::VectorXd & x) -> Eigen::Vector4d {
    Eigen::VectorXd xyz = h_armor_xyz(x, id);
    Eigen::VectorXd ypd = tools::xyz2ypd(xyz);
    
    // 与 h_armor_xyz / h_jacobian 保持一致，统一使用减号
    double angle = tools::limit_rad(x[6] - id * 2 * CV_PI / armor_num_);
    
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

  if (enable_w_acc_limit_) {
    const double dt = std::max(last_predict_dt_, 1e-3);
    const double max_dw = max_w_acc_ * dt;
    ekf_.x[7] = std::clamp(ekf_.x[7], w_before_update - max_dw, w_before_update + max_dw);
  }

  if (name == ArmorName::outpost) {
    constexpr double OUTPOST_DZ0_MIN = -0.20;
    constexpr double OUTPOST_DZ0_MAX = -0.03;
    constexpr double OUTPOST_DZ2_MIN = 0.03;
    constexpr double OUTPOST_DZ2_MAX = 0.20;
    constexpr double OUTPOST_TARGET_SPAN = 0.20;
    constexpr double OUTPOST_MID_RELAX_GAIN = 0.25;
    constexpr double OUTPOST_SPAN_RELAX_GAIN = 0.10;

    auto & dz0 = ekf_.x[9];
    auto & dz2 = ekf_.x[10];

    // 约束 1：中点回零（id=1 为基准层，理想情况下 dz0 + dz2 ≈ 0）
    const double mid = 0.5 * (dz0 + dz2);
    dz0 -= OUTPOST_MID_RELAX_GAIN * mid;
    dz2 -= OUTPOST_MID_RELAX_GAIN * mid;

    // 约束 2：层间距回拉（抑制 x[9]/x[10] 长时塌缩或发散）
    const double span = dz2 - dz0;
    const double span_err = OUTPOST_TARGET_SPAN - span;
    dz0 -= 0.5 * OUTPOST_SPAN_RELAX_GAIN * span_err;
    dz2 += 0.5 * OUTPOST_SPAN_RELAX_GAIN * span_err;

    // 约束 3：物理限幅
    dz0 = std::clamp(dz0, OUTPOST_DZ0_MIN, OUTPOST_DZ0_MAX);
    dz2 = std::clamp(dz2, OUTPOST_DZ2_MIN, OUTPOST_DZ2_MAX);

    // 确保上下层顺序正确
    if (dz2 <= dz0 + 0.06) {
      const double center = 0.5 * (dz0 + dz2);
      dz0 = std::clamp(center - 0.03, OUTPOST_DZ0_MIN, OUTPOST_DZ0_MAX);
      dz2 = std::clamp(center + 0.03, OUTPOST_DZ2_MIN, OUTPOST_DZ2_MAX);
    }
  }
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
  auto r_ok = ekf_.x[8] > 0.05 && ekf_.x[8] < 0.9;

  // 前哨站中 x[9]/x[10] 已用于层高差，不再参与半径合法性判据
  if (name == ArmorName::outpost) {
    if (r_ok) return false;
    tools::logger()->debug(
      "[Target][outpost] radius diverged, r={:.3f}, dz01={:.3f}, dz02={:.3f}", ekf_.x[8],
      ekf_.x[9], ekf_.x[10]);
    return true;
  }

  auto l_ok = ekf_.x[8] + ekf_.x[9] > 0.05 && ekf_.x[8] + ekf_.x[9] < 0.9;

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

  // 前哨站：高差从状态量 x[9]/x[10] 读取（在线估计）
  if (name == ArmorName::outpost) {
    double z_offsets[3] = {x[9], 0.0, x[10]};
    armor_z = x[4] + z_offsets[id];
  }

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

  // 前哨站：x[9]=id=0高差, x[10]=id=2高差
  double dz_dl_outpost = 0.0;
  double dz_dh_outpost = 0.0;
  if (name == ArmorName::outpost) {
    if (id == 0) dz_dl_outpost = 1.0;
    if (id == 2) dz_dh_outpost = 1.0;
  }

  // clang-format off
  Eigen::MatrixXd H_armor_xyza{
    {1, 0, 0, 0, 0, 0, dx_da, 0, dx_dr, dx_dl,     0},
    {0, 0, 1, 0, 0, 0, dy_da, 0, dy_dr, dy_dl,     0},
    {0, 0, 0, 0, 1, 0,     0, 0,     0, dz_dl_outpost, (name == ArmorName::outpost) ? dz_dh_outpost : dz_dh},
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
