#include "anti_spin_utils.hpp"

#include <algorithm>

namespace auto_aim
{

std::vector<std::pair<Eigen::Vector4d, int>> sort_armors_by_distance(
    const std::vector<Eigen::Vector4d> & armor_xyza_list)
{
  std::vector<std::pair<Eigen::Vector4d, int>> sorted;
  for (size_t i = 0; i < armor_xyza_list.size(); i++) {
    sorted.push_back({armor_xyza_list[i], static_cast<int>(i)});
  }
  std::sort(sorted.begin(), sorted.end(),
    [](const auto & a, const auto & b) {
      return Eigen::Vector2d(a.first[0], a.first[1]).norm() <
             Eigen::Vector2d(b.first[0], b.first[1]).norm();
    });
  return sorted;
}

double calculate_angle(
    const Eigen::Vector2d & A,
    const Eigen::Vector2d & B,
    const Eigen::Vector2d & C)
{
  Eigen::Vector2d BA = A - B;
  Eigen::Vector2d BC = C - B;
  double dot = BA.dot(BC);
  double norm_product = BA.norm() * BC.norm();
  if (norm_product < 1e-6) return 0.0;
  double cos_theta = std::clamp(dot / norm_product, -1.0, 1.0);
  return std::acos(cos_theta);
}

double calculate_angle_abs(
    const Eigen::Vector2d & A,
    const Eigen::Vector2d & B,
    const Eigen::Vector2d & C)
{
  double angle = calculate_angle(A, B, C);
  if (angle > CV_PI / 2) angle = CV_PI - angle;  // Normalize to [0, PI/2]
  return angle;
}

}  // namespace auto_aim
