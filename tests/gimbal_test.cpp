#include "io/gimbal/gimbal.hpp"

#include <chrono>
#include <opencv2/opencv.hpp>
#include <thread>

#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

const std::string keys =
  "{help h usage ? | | 输出命令行参数说明}"
  "{f              | | 是否开火}"
  "{@config-path   | | yaml配置文件路径 }";

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto test_fire = cli.get<bool>("f");
  auto config_path = cli.get<std::string>("@config-path");
  if (cli.has("help") || !cli.has("@config-path")) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;
  tools::Plotter plotter;

  io::Gimbal gimbal(config_path);

  auto t0 = std::chrono::steady_clock::now();
  auto last_mode = gimbal.mode();
  uint16_t last_bullet_count = 0;

  // ===== NEW CODE 1/3: Initialize a timestamp for periodic logging =====
  auto last_log_time = std::chrono::steady_clock::now();


  auto fire = false;
  auto fire_count = 0;
  auto fire_stamp = std::chrono::steady_clock::now();
  auto first_fired = false;

  while (!exiter.exit()) {
    auto mode = gimbal.mode();

    if (mode != last_mode) {
      tools::logger()->info("Gimbal mode changed: {}", gimbal.str(mode));
      last_mode = mode;
    }

    auto t = std::chrono::steady_clock::now();
    auto state = gimbal.state();
//    tools::logger()->warn("CHECKPOINT A: About to call gimbal.q(t)..."); // <--- 添加这条日志
    auto q = gimbal.q(t);
//    tools::logger()->warn("CHECKPOINT B: Returned from gimbal.q(t).");   // <--- 添加这条日志

    // auto ypr = tools::eulers(q, 2, 1, 0);
     // 1. 只进行一次计算，并立即转换为角度制
    auto ypr_deg = tools::eulers(q, 2, 1, 0) * 57.3; // ypr_deg 现在存储了角度制的 Yaw, Pitch, Roll

    // ===== NEW CODE 2/3: Check if 1 second has passed since the last log =====
    if (tools::delta_time(t, last_log_time) >= 1.0) {
      // ===== NEW CODE 3/3: Print the received state data =====
      tools::logger()->info(
        "[Gimbal State] Yaw: {:.2f}, Pitch: {:.2f}, Bullet Speed: {:.1f}, Count: {}",
        state.yaw, state.pitch, state.bullet_speed, state.bullet_count);
      last_log_time = t; // Update the timestamp
    }

    auto fired = state.bullet_count > last_bullet_count;
    last_bullet_count = state.bullet_count;

    if (!first_fired && fired) {
      first_fired = true;
      tools::logger()->info("Gimbal first fired after: {:.3f}s", tools::delta_time(t, fire_stamp));
    }

    if (fire && fire_count > 20) {
      // 0.2 s
      fire = false;
      fire_count = 0;
    } else if (!fire && fire_count > 100) {
      // 1s
      fire = true;
      fire_count = 0;
      fire_stamp = t;
      first_fired = false;
    }
    fire_count++;

    gimbal.send(true, test_fire && fire, 1, 0, 0, 0, 0, 0);

    nlohmann::json data;
    // data["q_yaw"] = ypr[0];
    // data["q_pitch"] = ypr[1];

    data["q_yaw"] = ypr_deg[0];   // 发送角度制的 Yaw
    data["q_pitch"] = ypr_deg[1]; // 发送角度制的 Pitch
    data["q_roll"] = ypr_deg[2];  // 3. 新增：发送角度制的 Roll
    
    data["yaw"] = state.yaw;
    data["vyaw"] = state.yaw_vel;
    data["pitch"] = state.pitch;
    data["vpitch"] = state.pitch_vel;
    data["bullet_speed"] = state.bullet_speed;
    data["bullet_count"] = state.bullet_count;
    data["fired"] = fired ? 1 : 0;
    data["fire"] = test_fire && fire ? 1 : 0;
    data["t"] = tools::delta_time(t, t0);
    plotter.plot(data);

    std::this_thread::sleep_for(9ms);
  }

  gimbal.send(false, false, 0, 0, 0, 0, 0, 0);

  return 0;
}