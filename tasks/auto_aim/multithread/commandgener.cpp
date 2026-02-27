#include "commandgener.hpp"

#include "tools/math_tools.hpp"

namespace auto_aim
{
namespace multithread
{


// === 修改 / MODIFIED ===
// 构造函数 1: 接收 CBoard
CommandGener::CommandGener(
  auto_aim::Shooter & shooter, auto_aim::Aimer & aimer, io::CBoard & cboard,
  tools::Plotter & plotter, bool debug)
: shooter_(shooter), aimer_(aimer), plotter_(plotter), stop_(false), debug_(debug)
{
  // 使用 Lambda 表达式捕获 cboard 引用，并将其包装进 std::function
  send_function_ = [&cboard](const io::Command & command) {
    cboard.send(command);
  };
  thread_ = std::thread(&CommandGener::generate_command, this);
}

// === 新增 / NEW ===
// 构造函数 2: 接收 Gimbal
CommandGener::CommandGener(
  auto_aim::Shooter & shooter, auto_aim::Aimer & aimer, io::Gimbal & gimbal,
  tools::Plotter & plotter, bool debug)
: shooter_(shooter), aimer_(aimer), plotter_(plotter), stop_(false), debug_(debug)
{
  // 使用 Lambda 表达式捕获 gimbal 引用，并将其包装进 std::function
  send_function_ = [&gimbal](const io::Command & command) {
    gimbal.send(command);
  };
  thread_ = std::thread(&CommandGener::generate_command, this);
}

CommandGener::~CommandGener()
{
  {
    std::lock_guard<std::mutex> lock(mtx_);
    stop_ = true;
  }
  cv_.notify_all();
  if (thread_.joinable()) thread_.join();
}

void CommandGener::push(
  const std::list<auto_aim::Target> & targets, const std::chrono::steady_clock::time_point & t,
  double bullet_speed, const Eigen::Vector3d & gimbal_pos)
{
  std::lock_guard<std::mutex> lock(mtx_);
  latest_ = {targets, t, bullet_speed, gimbal_pos};
  cv_.notify_one();
}

void CommandGener::generate_command()
{
  auto t0 = std::chrono::steady_clock::now();
  while (!stop_) {
    std::optional<Input> input;
    {
      std::lock_guard<std::mutex> lock(mtx_);
      if (latest_ && tools::delta_time(std::chrono::steady_clock::now(), latest_->t) < 0.2) {
        input = latest_;
      } else
        input = std::nullopt;
    }
    if (input) {
      auto command = aimer_.aim(input->targets_, input->t, input->bullet_speed);
      command.shoot = shooter_.shoot(command, aimer_, input->targets_, input->gimbal_pos);
      command.horizon_distance = input->targets_.empty()
                                   ? 0
                                   : std::sqrt(
                                       tools::square(input->targets_.front().ekf_x()[0]) +
                                       tools::square(input->targets_.front().ekf_x()[2]));
      // === 修改 / MODIFIED ===
      // 直接调用我们包装好的 send_function_，它会根据构造时传入的对象
      // 自动调用 cboard.send() 或 gimbal.send()。
      send_function_(command);
      // === 原代码 / OLD CODE ===
      // cboard_.send(command);
      if (debug_) {
        nlohmann::json data;
        data["t"] = tools::delta_time(std::chrono::steady_clock::now(), t0);
        data["cmd_yaw"] = command.yaw * 57.3;
        data["cmd_pitch"] = command.pitch * 57.3;
        data["shoot"] = command.shoot;
        data["horizon_distance"] = command.horizon_distance;
        plotter_.plot(data);
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));  //approximately 500Hz
  }
}

}  // namespace multithread

}  // namespace auto_aim