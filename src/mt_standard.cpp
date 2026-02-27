#include <chrono>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera.hpp"
#include "io/dm_imu/dm_imu.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/multithread/commandgener.hpp"
#include "tasks/auto_aim/multithread/mt_detector.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_buff/buff_aimer.hpp"
#include "tasks/auto_buff/buff_detector.hpp"
#include "tasks/auto_buff/buff_solver.hpp"
#include "tasks/auto_buff/buff_target.hpp"
#include "tasks/auto_buff/buff_type.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"

const std::string keys =
  "{help h usage ? | | 输出命令行参数说明}"
  "{@config-path   | | yaml配置文件路径 }";

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>("@config-path");
  if (cli.has("help") || !cli.has("@config-path")) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;
  tools::Plotter plotter;
  tools::Recorder recorder;

  io::Camera camera(config_path);
// === 修改 / MODIFIED ===
  // 1. 使用新的 Gimbal 类来替代 CBoard 类进行初始化。
  io::Gimbal gimbal(config_path);
  // === 原代码 / OLD CODE ===
  // io::CBoard cboard(config_path);


  auto_aim::multithread::MultiThreadDetector detector(config_path);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);

  auto_buff::Buff_Detector buff_detector(config_path);
  auto_buff::Solver buff_solver(config_path);
  auto_buff::SmallTarget buff_small_target;
  auto_buff::BigTarget buff_big_target;
  auto_buff::Aimer buff_aimer(config_path);
  // === 修改 / MODIFIED ===
  // 2. 将 gimbal 对象传入 CommandGener。
  //    得益于我们之前的修改，CommandGener 现在可以接收 gimbal 对象。
  auto_aim::multithread::CommandGener commandgener(shooter, aimer, gimbal, plotter);
  // === 原代码 / OLD CODE ===
  // auto_aim::multithread::CommandGener commandgener(shooter, aimer, cboard, plotter);


  // === 修改 / MODIFIED ===
  // 3. 将 mode 变量的类型从 io::Mode 切换为 io::GimbalMode 以匹配新的接口。
  std::atomic<io::GimbalMode> mode{io::GimbalMode::IDLE};
  auto last_mode{io::GimbalMode::IDLE};
  // === 原代码 / OLD CODE ===
  // std::atomic<io::Mode> mode{io::Mode::idle};
  // auto last_mode{io::Mode::idle};


  auto detect_thread = std::thread([&]() {
    cv::Mat img;
    std::chrono::steady_clock::time_point t;

    while (!exiter.exit()) {
      // if (mode.load() == io::Mode::auto_aim) {
      //   camera.read(img, t);
      //   detector.push(img, t);
      // } else
      //   continue;
      // === 修改 / MODIFIED ===
      // 4. 在检测线程中，同样使用新的 GimbalMode 枚举进行模式判断。
      if (mode.load() == io::GimbalMode::AUTO_AIM) {
        camera.read(img, t);
        detector.push(img, t);
      } else
        continue;
    }
  });

  while (!exiter.exit()) {
    // === 修改 / MODIFIED ===
    // 5. 从 gimbal 对象获取模式和状态信息。
    //    将状态获取(gs)放在循环开头，以便后续的自瞄和打符逻辑都能使用。
    mode = gimbal.mode();
    auto gs = gimbal.state();
    // === 原代码 / OLD CODE ===
    // mode = cboard.mode;

    if (last_mode != mode) {
      // === 修改 / MODIFIED ===
      // 6. 使用 gimbal.str() 来获取模式的字符串表示，用于日志打印。
      tools::logger()->info("Switch to {}", gimbal.str(mode));
      last_mode = mode.load();
      // === 原代码 / OLD CODE ===
      // tools::logger()->info("Switch to {}", io::MODES[mode]);
      // last_mode = mode.load();
    }
    // === 步骤 7: 【关键修正】修正主循环中的模式判断 ===  
    /// 自瞄
    if (mode.load() == io::GimbalMode::AUTO_AIM) {
      auto [img, armors, t] = detector.debug_pop();
      // === 修改 / MODIFIED ===
      // 8. 使用 gimbal.q() 替代 cboard.imu_at() 获取姿态。
      Eigen::Quaterniond q = gimbal.q(t - 1ms);
      // === 原代码 / OLD CODE ===
      // Eigen::Quaterniond q = cboard.imu_at(t - 1ms);

      // recorder.record(img, q, t);

      solver.set_R_gimbal2world(q);

      Eigen::Vector3d ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);

      auto targets = tracker.track(armors, t);
      // === 修改 / MODIFIED ===
      // 8. 将从 gimbal 状态中获取的弹速 gs.bullet_speed 传入决策线程。
      commandgener.push(targets, t, gs.bullet_speed, ypr);  // 发送给决策线程
      // === 原代码 / OLD CODE ===
      // commandgener.push(targets, t, cboard.bullet_speed, ypr);
      // commandgener.push(targets, t, cboard.bullet_speed, ypr);  

    }
    // === 步骤 7: 【关键修正】修正主循环中的模式判断 ===
    /// 打符
    else if (mode.load() == io::GimbalMode::SMALL_BUFF || mode.load() == io::GimbalMode::BIG_BUFF) {
      cv::Mat img;
      Eigen::Quaterniond q;
      std::chrono::steady_clock::time_point t;

      camera.read(img, t);
      // === 修改 / MODIFIED ===
      // 9. 在打符逻辑中，同样使用 gimbal.q() 获取姿态。
      q = gimbal.q(t - 1ms);
      // === 原代码 / OLD CODE ===
      // q = cboard.imu_at(t - 1ms);

      // recorder.record(img, q, t);

      buff_solver.set_R_gimbal2world(q);

      auto power_runes = buff_detector.detect(img);

      buff_solver.solve(power_runes);

      io::Command buff_command;
      // if (mode.load() == io::Mode::small_buff) {
      //   buff_small_target.get_target(power_runes, t);
      //   auto target_copy = buff_small_target;
      //   buff_command = buff_aimer.aim(target_copy, t, cboard.bullet_speed, true);
      // } else if (mode.load() == io::Mode::big_buff) {
      //   buff_big_target.get_target(power_runes, t);
      //   auto target_copy = buff_big_target;
      //   buff_command = buff_aimer.aim(target_copy, t, cboard.bullet_speed, true);
      // }
      // cboard.send(buff_command);
      // === 修改 / MODIFIED ===
      // 10. 将从 gimbal 状态中获取的弹速 gs.bullet_speed 传入打符瞄准器。
      if (mode.load() == io::GimbalMode::SMALL_BUFF) {
        buff_small_target.get_target(power_runes, t);
        auto target_copy = buff_small_target;
        buff_command = buff_aimer.aim(target_copy, t, gs.bullet_speed, true);
      } else if (mode.load() == io::GimbalMode::BIG_BUFF) {
        buff_big_target.get_target(power_runes, t);
        auto target_copy = buff_big_target;
        buff_command = buff_aimer.aim(target_copy, t, gs.bullet_speed, true);
      }
      // === 原代码 / OLD CODE ===
      // ... buff_command = buff_aimer.aim(..., cboard.bullet_speed, true);
      
      // === 修改 / MODIFIED ===
      // 11. 【关键】直接使用 gimbal 对象发送打符指令。
      //     因为我们之前已为 Gimbal 类添加了 send(io::Command) 重载，所以这里可以无缝替换。
      gimbal.send(buff_command);
      // === 原代码 / OLD CODE ===
      // cboard.send(buff_command);

    } else
      continue;
  }

  detect_thread.join();

  return 0;
}