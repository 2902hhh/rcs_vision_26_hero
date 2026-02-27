#include <fmt/format.h>

#include "io/gimbal/gimbal.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"
#include "tools/trajectory.hpp"

// 定义命令行参数
const std::string keys =
  "{help h usage ? | | 输出命令行参数说明}"
  "{@config-path   | | yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  // 读取命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  // 初始化绘图器、录制器、退出器
  tools::Plotter plotter;
  tools::Recorder recorder;
  tools::Exiter exiter;

  // 初始化云台
  io::Gimbal gimbal(config_path);
  io::VisionToGimbal plan;
  auto last_t = std::chrono::steady_clock::now();
  plan.yaw = 0;
  plan.yaw_vel = 0;
  plan.yaw_acc = 0;
  plan.pitch = 0;
  plan.pitch_vel = 0;
  plan.pitch_acc = 0;

  // -------------- 新增和修改的部分 --------------

  // 定义开火持续时间和冷却（不开火）时间
  constexpr double FIRE_DURATION = 3.0;     // 开火持续3秒
  constexpr double COOLDOWN_DURATION = 1.6; // 冷却1.6秒

  // 新增状态变量，用于标记当前是否处于开火状态
  bool is_firing = false;
  // 用于记录上一次状态切换的时间点
  auto last_state_change_t = std::chrono::steady_clock::now();

  while (!exiter.exit()) {
    auto now = std::chrono::steady_clock::now();
    auto gs = gimbal.state();

    // 计算距离上次状态切换过去的时间
    double time_since_last_change = tools::delta_time(now, last_state_change_t);

    if (is_firing) {
      // 当前是开火状态，检查开火时间是否已超过3秒
      if (time_since_last_change > FIRE_DURATION) {
        // 超过3秒，切换到冷却状态
        is_firing = false;
        last_state_change_t = now; // 更新状态切换时间点
        tools::logger()->debug("stop fire.");
      }
    } else {
      // 当前是冷却状态，检查冷却时间是否已超过1.6秒
      if (time_since_last_change > COOLDOWN_DURATION) {
        // 超过1.6秒，切换到开火状态
        is_firing = true;
        last_state_change_t = now; // 更新状态切换时间点
        tools::logger()->debug("start fire!");
      }
    }

    // 根据当前状态设置plan.mode
    if (is_firing) {
      plan.mode = 2; // 开火
    } else {
      plan.mode = 1; // 不开火
    }


  // while (!exiter.exit()) {
  //   auto now = std::chrono::steady_clock::now();
  //   auto gs = gimbal.state();
  //   if(tools::delta_time(now, last_t) > 1.600) {
  //       plan.mode = 2;
  //       tools::logger()->debug("fire!");
  //       last_t = now;
  //   } else plan.mode = 1;


    gimbal.send(plan);

    // -------------- 调试输出 --------------

    nlohmann::json data;

    if (plan.mode != 0) {
      data["shoot"] = plan.mode == 2 ? 1 : 0;
    }

    plotter.plot(data);

    auto key = cv::waitKey(1);
    if (key == 'q') break;
  }

  return 0;
}
