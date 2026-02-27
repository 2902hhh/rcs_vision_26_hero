#ifndef IO__COMMAND_HPP
#define IO__COMMAND_HPP

namespace io
{
struct Command
{
  bool control;
  bool shoot;
  double yaw;
  double pitch;

  // === 新增以下 4 个字段 ===
  double yaw_vel;   // Yaw轴目标角速度 (rad/s)
  double pitch_vel; // Pitch轴目标角速度 (rad/s)
  double yaw_acc;   // Yaw轴目标角加速度 (rad/s^2)
  double pitch_acc; // Pitch轴目标角加速度 (rad/s^2)
  // ======================
  

  double horizon_distance = 0;  //无人机专有
};

}  // namespace io

#endif  // IO__COMMAND_HPP