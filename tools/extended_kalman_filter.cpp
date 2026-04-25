#include "extended_kalman_filter.hpp"

#include <numeric>

namespace tools
{
ExtendedKalmanFilter::ExtendedKalmanFilter(
  const Eigen::VectorXd & x0, const Eigen::MatrixXd & P0,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add,
  bool use_ukf)
: x(x0),
  P(P0),
  I(Eigen::MatrixXd::Identity(x0.rows(), x0.rows())),
  use_ukf_(use_ukf),
  x_add(x_add)
{
  data["residual_yaw"] = 0.0;
  data["residual_pitch"] = 0.0;
  data["residual_distance"] = 0.0;
  data["residual_angle"] = 0.0;
  data["nis"] = 0.0;
  data["nees"] = 0.0;
  data["nis_fail"] = 0.0;
  data["nees_fail"] = 0.0;
  data["recent_nis_failures"] = 0.0;
}

Eigen::VectorXd ExtendedKalmanFilter::predict(const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q)
{
  return predict(F, Q, [&](const Eigen::VectorXd & x) { return F * x; });
}

Eigen::VectorXd ExtendedKalmanFilter::predict(
  const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &)> f)
{
  if (use_ukf_) {
    return predict_ukf(Q, f);
  }

  P = F * P * F.transpose() + Q;
  x = f(x);
  return x;
}

Eigen::VectorXd ExtendedKalmanFilter::update(
  const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract)
{
  return update(z, H, R, [&](const Eigen::VectorXd & x) { return H * x; }, z_subtract);
}

Eigen::VectorXd ExtendedKalmanFilter::update(
  const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &)> h,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract)
{
  if (use_ukf_) {
    return update_ukf(z, R, h, z_subtract);
  }

  Eigen::VectorXd x_prior = x;
  Eigen::MatrixXd K = P * H.transpose() * (H * P * H.transpose() + R).inverse();

  // Stable Compution of the Posterior Covariance
  // https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/07-Kalman-Filter-Math.ipynb
  P = (I - K * H) * P * (I - K * H).transpose() + K * R * K.transpose();

  x = x_add(x, K * z_subtract(z, h(x)));

  /// 卡方检验
  Eigen::VectorXd residual = z_subtract(z, h(x));
  // 新增检验
  Eigen::MatrixXd S = H * P * H.transpose() + R;
  double nis = residual.transpose() * S.inverse() * residual;
  double nees = (x - x_prior).transpose() * P.inverse() * (x - x_prior);

  // 卡方检验阈值（上分位）：
  // NIS: 自由度=4, 95% -> 9.49
  // NEES: 状态维度=11, 95% -> 19.68
  constexpr double nis_threshold = 9.49;
  constexpr double nees_threshold = 19.68;

  if (nis > nis_threshold) nis_count_++, data["nis_fail"] = 1;
  if (nees > nees_threshold) nees_count_++, data["nees_fail"] = 1;
  total_count_++;
  last_nis = nis;

  recent_nis_failures.push_back(nis > nis_threshold ? 1 : 0);

  if (recent_nis_failures.size() > window_size) {
    recent_nis_failures.pop_front();
  }

  int recent_failures = std::accumulate(recent_nis_failures.begin(), recent_nis_failures.end(), 0);
  double recent_rate = static_cast<double>(recent_failures) / recent_nis_failures.size();

  data["residual_yaw"] = residual[0];
  data["residual_pitch"] = residual[1];
  data["residual_distance"] = residual[2];
  data["residual_angle"] = residual[3];
  data["nis"] = nis;
  data["nees"] = nees;
  data["recent_nis_failures"] = recent_rate;

  return x;
}

Eigen::MatrixXd ExtendedKalmanFilter::generate_sigma_points(
  const Eigen::VectorXd & mean, const Eigen::MatrixXd & cov) const
{
  const int n = mean.size();
  const double lambda = alpha_ * alpha_ * (n + kappa_) - n;
  const double scale = n + lambda;

  Eigen::MatrixXd sigma_points(n, 2 * n + 1);
  sigma_points.col(0) = mean;

  Eigen::MatrixXd scaled_cov = scale * cov;
  Eigen::LLT<Eigen::MatrixXd> llt(scaled_cov);
  if (llt.info() != Eigen::Success) {
    scaled_cov += 1e-6 * Eigen::MatrixXd::Identity(n, n);
    llt.compute(scaled_cov);
  }

  Eigen::MatrixXd L = llt.matrixL();
  for (int i = 0; i < n; ++i) {
    sigma_points.col(i + 1) = mean + L.col(i);
    sigma_points.col(i + 1 + n) = mean - L.col(i);
  }

  return sigma_points;
}

Eigen::VectorXd ExtendedKalmanFilter::weighted_mean(
  const Eigen::MatrixXd & sigma_points, const Eigen::VectorXd & weights) const
{
  Eigen::VectorXd mean = Eigen::VectorXd::Zero(sigma_points.rows());
  for (int i = 0; i < sigma_points.cols(); ++i) {
    mean += weights(i) * sigma_points.col(i);
  }
  return mean;
}

Eigen::VectorXd ExtendedKalmanFilter::predict_ukf(
  const Eigen::MatrixXd & Q, std::function<Eigen::VectorXd(const Eigen::VectorXd &)> f)
{
  const int n = x.size();
  const int sigma_count = 2 * n + 1;
  const double lambda = alpha_ * alpha_ * (n + kappa_) - n;

  Eigen::VectorXd Wm(sigma_count), Wc(sigma_count);
  Wm(0) = lambda / (n + lambda);
  Wc(0) = Wm(0) + (1.0 - alpha_ * alpha_ + beta_);
  for (int i = 1; i < sigma_count; ++i) {
    Wm(i) = 1.0 / (2.0 * (n + lambda));
    Wc(i) = Wm(i);
  }

  Eigen::MatrixXd sigma_points = generate_sigma_points(x, P);
  Eigen::MatrixXd predicted_sigma(n, sigma_count);
  for (int i = 0; i < sigma_count; ++i) {
    predicted_sigma.col(i) = f(sigma_points.col(i));
  }

  Eigen::VectorXd x_pred = weighted_mean(predicted_sigma, Wm);
  Eigen::MatrixXd P_pred = Q;
  for (int i = 0; i < sigma_count; ++i) {
    Eigen::VectorXd dx = predicted_sigma.col(i) - x_pred;
    P_pred += Wc(i) * (dx * dx.transpose());
  }

  x = x_pred;
  P = P_pred;
  return x;
}

Eigen::VectorXd ExtendedKalmanFilter::update_ukf(
  const Eigen::VectorXd & z, const Eigen::MatrixXd & R,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &)> h,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract)
{
  const int n = x.size();
  const int m = z.size();
  const int sigma_count = 2 * n + 1;
  const double lambda = alpha_ * alpha_ * (n + kappa_) - n;

  Eigen::VectorXd Wm(sigma_count), Wc(sigma_count);
  Wm(0) = lambda / (n + lambda);
  Wc(0) = Wm(0) + (1.0 - alpha_ * alpha_ + beta_);
  for (int i = 1; i < sigma_count; ++i) {
    Wm(i) = 1.0 / (2.0 * (n + lambda));
    Wc(i) = Wm(i);
  }

  Eigen::VectorXd x_prior = x;
  Eigen::MatrixXd sigma_points = generate_sigma_points(x, P);

  Eigen::MatrixXd Zsig(m, sigma_count);
  for (int i = 0; i < sigma_count; ++i) {
    Zsig.col(i) = h(sigma_points.col(i));
  }

  Eigen::VectorXd z_pred = weighted_mean(Zsig, Wm);

  Eigen::MatrixXd S = R;
  Eigen::MatrixXd Pxz = Eigen::MatrixXd::Zero(n, m);
  for (int i = 0; i < sigma_count; ++i) {
    Eigen::VectorXd dz = z_subtract(Zsig.col(i), z_pred);
    Eigen::VectorXd dx = sigma_points.col(i) - x;
    S += Wc(i) * (dz * dz.transpose());
    Pxz += Wc(i) * (dx * dz.transpose());
  }

  Eigen::MatrixXd K = Pxz * S.inverse();
  Eigen::VectorXd innovation = z_subtract(z, z_pred);
  x = x_add(x, K * innovation);
  P = P - K * S * K.transpose();

  Eigen::VectorXd residual = z_subtract(z, h(x));
  double nis = residual.transpose() * S.inverse() * residual;
  double nees = (x - x_prior).transpose() * P.inverse() * (x - x_prior);

  constexpr double nis_threshold = 9.49;
  constexpr double nees_threshold = 19.68;

  if (nis > nis_threshold) nis_count_++, data["nis_fail"] = 1;
  if (nees > nees_threshold) nees_count_++, data["nees_fail"] = 1;
  total_count_++;
  last_nis = nis;

  recent_nis_failures.push_back(nis > nis_threshold ? 1 : 0);
  if (recent_nis_failures.size() > window_size) {
    recent_nis_failures.pop_front();
  }

  int recent_failures = std::accumulate(recent_nis_failures.begin(), recent_nis_failures.end(), 0);
  double recent_rate = static_cast<double>(recent_failures) / recent_nis_failures.size();

  data["residual_yaw"] = residual[0];
  data["residual_pitch"] = residual[1];
  data["residual_distance"] = residual[2];
  data["residual_angle"] = residual[3];
  data["nis"] = nis;
  data["nees"] = nees;
  data["recent_nis_failures"] = recent_rate;

  return x;
}

}  // namespace tools