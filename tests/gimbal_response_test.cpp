#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/cboard.hpp"
#include "io/gimbal/gimbal.hpp"

#include "io/command.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? |                     | 输出命令行参数说明}"
  "{delta-angle a  |          30          | yaw轴delta角}"
  "{circle      c  |         1         | delta_angle的切片数}"
  "{signal-mode m  |     triangle_wave   | 发送信号的模式}"
  "{axis        x  |         yaw         | 发送信号的轴}"
  "{@config-path   | configs/sentry.yaml | 位置参数，yaml配置文件路径 }";

double yaw_cal(double t)
{
  double A = 7;
  double T = 1;  // s

  return A * std::sin(2 * M_PI * t / T);  // 31是云台yaw初始角度，单位为度
}

double pitch_cal(double t)
{
  double A = 7;
  double T = 1;  // s

  return A * std::sin(2 * M_PI * t / T + M_PI / 2);  // 18/0是云台pitch初始角度，单位为度
}

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  auto delta_angle = cli.get<double>("delta-angle");
  auto circle = cli.get<double>("circle");
  auto signal_mode = cli.get<std::string>("signal-mode");
  auto axis = cli.get<std::string>("axis");
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;
  tools::Plotter plotter;

  // io::CBoard cboard(config_path);
  io::Gimbal gimbal(config_path);

  auto init_angle = 0;
  double slice = circle * 100;  //切片数=周期*帧率
  auto dangle = delta_angle / slice;
  double cmd_angle = init_angle;

  int axis_index = axis == "yaw" ? 0 : 1;  // 0 for yaw, 1 for pitch

  double error = 0;
  int count = 0;

  io::Command init_command{1, 0, 0, 0};


  gimbal.send(true, false, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

  
  std::this_thread::sleep_for(5s);  //等待云台归零

  io::Command command{0};
  io::Command last_command{0};

  double t = 0;
  auto last_t = t;
  double dt = 0.005;  // 5ms, 模拟200fps

  auto t0 = std::chrono::steady_clock::now();

  while (!exiter.exit()) {
    nlohmann::json data;
    auto timestamp = std::chrono::steady_clock::now();

    std::this_thread::sleep_for(1ms);

    Eigen::Quaterniond q = gimbal.q(timestamp);

    Eigen::Vector3d eulers = tools::eulers(q, 2, 1, 0);

    if (signal_mode == "triangle_wave") {
      if (count == slice) {
        cmd_angle = init_angle;
        command = {1, 0, 0, 0};
        if (axis_index == 0)
          command.yaw = cmd_angle / 57.3;
        else
          command.pitch = cmd_angle / 57.3;
        count = 0;

      } else {
        cmd_angle += dangle;
        if (axis_index == 0)
          command.yaw = cmd_angle / 57.3;
        else
          command.pitch = cmd_angle / 57.3;
        count++;
      }


            gimbal.send(
        command.control,
        command.shoot,
        static_cast<float>(command.yaw),
        0.0f, // yaw_vel
        0.0f, // yaw_acc
        static_cast<float>(command.pitch),
        0.0f, // pitch_vel
        0.0f  // pitch_acc
      );

      if (axis_index == 0) {
        data["cmd_yaw"] = command.yaw * 57.3;
        data["last_cmd_yaw"] = last_command.yaw * 57.3;
        data["gimbal_yaw"] = eulers[0] * 57.3;
      } else {
        data["cmd_pitch"] = command.pitch * 57.3;
        data["last_cmd_pitch"] = last_command.pitch * 57.3;
        data["gimbal_pitch"] = eulers[1] * 57.3;
      }
      data["t"] = tools::delta_time(std::chrono::steady_clock::now(), t0);
      last_command = command;
      plotter.plot(data);
      std::this_thread::sleep_for(8ms);  //模拟自瞄100fps
    }

    // else if (signal_mode == "step") {
    //   if (count == 300) {
    //     cmd_angle += delta_angle;
    //     count = 0;
    //   }
    //   command = {1, 0, tools::limit_rad(cmd_angle / 57.3), 0};
    //   count++;

    //         gimbal.send(
    //     command.control,
    //     command.shoot,
    //     static_cast<float>(command.yaw),
    //     0.0f, // yaw_vel
    //     0.0f, // yaw_acc
    //     static_cast<float>(command.pitch),
    //     0.0f, // pitch_vel
    //     0.0f  // pitch_acc
    //   );
    //   data["cmd_yaw"] = command.yaw * 57.3;
    //   data["last_cmd_yaw"] = last_command.yaw * 57.3;
    //   data["gimbal_yaw"] = eulers[0] * 57.3;
    //   last_command = command;
    //   plotter.plot(data);
    //   std::this_thread::sleep_for(8ms);  //模拟自瞄100fps
    // }
    else if (signal_mode == "step") {
      if (count == 70) { // 将 300 修改为 200, 使阶跃间隔变为 2 秒
        cmd_angle += delta_angle;
        count = 0;
      }
      command = {1, 0, tools::limit_rad(cmd_angle / 57.3), 0};
      count++;

            gimbal.send(
        command.control,
        command.shoot,
        static_cast<float>(command.yaw),
        0.0f, // yaw_vel
        0.0f, // yaw_acc
        static_cast<float>(command.pitch),
        0.0f, // pitch_vel
        0.0f  // pitch_acc
      );
      data["cmd_yaw"] = command.yaw * 57.3;
      data["last_cmd_yaw"] = last_command.yaw * 57.3;
      data["gimbal_yaw"] = eulers[0] * 57.3;
      last_command = command;
      plotter.plot(data);
      std::this_thread::sleep_for(8ms);  //模拟自瞄100fps
    }


    else if (signal_mode == "circle") {
      std::cout << "t: " << t << std::endl;
      command.yaw = yaw_cal(t) / 57.3;
      command.pitch = pitch_cal(t) / 57.3;
      command.control = 1;
      command.shoot = 0;
      t += dt;
      // if (t - last_t > 2) {
      //   t += 2.4;
      //   last_t = t;
      // }
      gimbal.send(
        command.control,
        command.shoot,
        static_cast<float>(command.yaw),
        0.0f, // yaw_vel
        0.0f, // yaw_acc
        static_cast<float>(command.pitch),
        0.0f, // pitch_vel
        0.0f  // pitch_acc
      );
      data["t"] = t;
      data["cmd_yaw"] = command.yaw * 57.3;
      data["cmd_pitch"] = command.pitch * 57.3;
      data["gimbal_yaw"] = eulers[0] * 57.3;
      data["gimbal_pitch"] = eulers[1] * 57.3;
      plotter.plot(data);
      std::this_thread::sleep_for(9ms);
    }
    
    else if (signal_mode == "velocity_step") {
      // ====================================================================
      // ==        速度环阶跃信号调试模式 (内置 RPM 单位转换)            ==
      // ====================================================================
      // * 使用方法:
      //   在终端输入: ./build/gimbal_respond_test -m=velocity_step -x=[yaw|pitch] -a=[速度值] ...
      //
      // * 参数说明:
      //   -a, --delta-angle: 在此模式下代表 "目标速度的幅值"，单位仍然是物理上
      //                      方便理解的 "rad/s"。代码会自动转换为RPM再发送。
      //                      例如: -a=2.0 表示发送 ±2.0 rad/s 对应的RPM值。
      //
      // * 工作流程:
      //   1. 上位机生成 rad/s 单位的方波信号，并将其转换为RPM。
      //   2. 将RPM单位的速度指令发送给下位机。
      //   3. 在下位机中观察 "目标速度(RPM)" 和 "电机实际速度(RPM)" 的曲线。
      // ====================================================================

      // 定义 rad/s 到 RPM 的转换系数
      const double RAD_S_TO_RPM = 60.0 / (2.0 * M_PI); // 约等于 9.5493

      auto target_velocity_rad_s = cli.get<double>("delta-angle");
      
      // 每4秒为一个周期，生成方波信号
      double period = 4.0; // 方波周期 (秒)
      double current_time_sec = tools::delta_time(std::chrono::steady_clock::now(), t0);
      double time_in_period = fmod(current_time_sec, period);

      float target_velocity_command_rad_s = 0.0f;
      if (time_in_period < period / 2.0) {
        target_velocity_command_rad_s = static_cast<float>(target_velocity_rad_s);
      } else {
        target_velocity_command_rad_s = -static_cast<float>(target_velocity_rad_s);
      }
      // float target_velocity_command_rad_s = static_cast<float>(target_velocity_rad_s);

      // --- 单位转换 ---
      // 在发送前，将 rad/s 转换为 RPM
      float target_velocity_command_rpm = target_velocity_command_rad_s * RAD_S_TO_RPM;

      // 根据-x参数，只发送指定轴的速度指令
      if (axis == "yaw") {
        gimbal.send(
          true,     // control mode on
          false,    // shoot mode off
          0.0f,     // yaw_angle
          target_velocity_command_rpm, // yaw_vel (单位: RPM)
          0.0f,     // yaw_acc
          0.0f,     // pitch_angle
          0.0f,     // pitch_vel
          0.0f      // pitch_acc
        );
      } else { // pitch
        gimbal.send(
          true,     // control mode on
          false,    // shoot mode off
          0.0f,     // yaw_angle
          0.0f,     // yaw_vel
          0.0f,     // yaw_acc
          0.0f,     // pitch_angle
          target_velocity_command_rpm, // pitch_vel (单位: RPM)
          0.0f      // pitch_acc
        );
      }
      
      // 绘图部分只显示我们发送的指令值，方便确认
      data["t"] = current_time_sec;
      // 为了方便，我们在图上同时显示 rad/s 和转换后的 RPM 值
      data["target_velocity_rad_s"] = target_velocity_command_rad_s;
      data["target_velocity_rpm_sent"] = target_velocity_command_rpm;
      plotter.plot(data);

      std::this_thread::sleep_for(8ms); // 维持发送频率
    }

  }
  return 0;
}