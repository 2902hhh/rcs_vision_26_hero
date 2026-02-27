#ifndef AUTO_AIM_MULTITHREAD__HPP
#define AUTO_AIM_MULTITHREAD__HPP

#include <optional>

#include <functional>

#include "io/cboard.hpp"
#include "io/gimbal/gimbal.hpp"

#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/omniperception/decider.hpp"
#include "tools/plotter.hpp"

namespace auto_aim
{
namespace multithread
{

class CommandGener
{
public:
  // === 修改 / MODIFIED ===
  // 3. 提供两个重载的构造函数，一个用于CBoard，一个用于Gimbal
  // 构造函数 for CBoard
  CommandGener(
    auto_aim::Shooter & shooter, auto_aim::Aimer & aimer, io::CBoard & cboard,
    tools::Plotter & plotter, bool debug = false);

  // 构造函数 for Gimbal
  CommandGener(
    auto_aim::Shooter & shooter, auto_aim::Aimer & aimer, io::Gimbal & gimbal,
    tools::Plotter & plotter, bool debug = false);

  ~CommandGener();

  void push(
    const std::list<auto_aim::Target> & targets, const std::chrono::steady_clock::time_point & t,
    double bullet_speed, const Eigen::Vector3d & gimbal_pos);

private:
  struct Input
  {
    std::list<auto_aim::Target> targets_;
    std::chrono::steady_clock::time_point t;
    // std::function<void()> decide;
    double bullet_speed;
    Eigen::Vector3d gimbal_pos;
  };

  // === 修改 / MODIFIED ===
  // 4. 将具体的通信对象引用替换为一个 std::function。
  //    这个函数对象将包装实际的 send 调用。
  std::function<void(const io::Command &)> send_function_;
  // === 原代码 / OLD CODE ===
  // io::CBoard & cboard_;
  auto_aim::Shooter & shooter_;
  auto_aim::Aimer & aimer_;
  tools::Plotter & plotter_;

  std::optional<Input> latest_;
  std::mutex mtx_;
  std::condition_variable cv_;
  std::thread thread_;
  bool stop_, debug_;

  void generate_command();
};

}  // namespace multithread

}  // namespace auto_aim

#endif  // AUTO_AIM_MULTITHREAD__HPP