#ifndef AUTO_AIM__ANTI_SPIN_UTILS_HPP
#define AUTO_AIM__ANTI_SPIN_UTILS_HPP

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <utility>
#include <vector>

namespace auto_aim
{

// Sort armors by XY-plane distance from origin (nearest first)
// Returns vector of (armor_xyza, original_index) sorted by distance
std::vector<std::pair<Eigen::Vector4d, int>> sort_armors_by_distance(
    const std::vector<Eigen::Vector4d> & armor_xyza_list);

// Calculate the raw angle at point B formed by vectors BA and BC
// Returns acos value in [0, PI] range -- NO normalization
// Used by Shooter for precision timing (judging_precision_shoot)
double calculate_angle(
    const Eigen::Vector2d & A,
    const Eigen::Vector2d & B,
    const Eigen::Vector2d & C);

// Calculate the absolute angle at point B formed by points A-B-C
// Returns angle in [0, PI/2] range (normalizes angles > PI/2)
// Used by Aimer for face angle calculations (armor selection)
double calculate_angle_abs(
    const Eigen::Vector2d & A,
    const Eigen::Vector2d & B,
    const Eigen::Vector2d & C);

}  // namespace auto_aim

#endif  // AUTO_AIM__ANTI_SPIN_UTILS_HPP
