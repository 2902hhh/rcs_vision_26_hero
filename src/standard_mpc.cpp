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

// 1. 修改参数定义，增加 display 选项
const std::string keys =
  "{help h usage ? |      | 输出命令行参数说明}"
  "{display d      | true | 是否显示视频流}"
  "{@config-path   |      | yaml配置文件路径 }";

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>("@config-path");
  bool enable_display = cli.get<bool>("display"); // 获取显示开关

  if (cli.has("help") || !cli.has("@config-path")) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;
  tools::Plotter plotter;
  tools::Recorder recorder;

  io::Gimbal gimbal(config_path);
  io::Camera camera(config_path);

  auto_aim::YOLO yolo(config_path, true);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Planner planner(config_path);

  tools::ThreadSafeQueue<std::optional<auto_aim::Target>, true> target_queue(1);
  target_queue.push(std::nullopt);

  auto_buff::Buff_Detector buff_detector(config_path);
  auto_buff::Solver buff_solver(config_path);
  auto_buff::SmallTarget buff_small_target;
  auto_buff::BigTarget buff_big_target;
  auto_buff::Aimer buff_aimer(config_path);

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;

  std::atomic<bool> quit = false;

  std::atomic<io::GimbalMode> mode{io::GimbalMode::IDLE};
  auto last_mode{io::GimbalMode::IDLE};

  // FPS 计算变量
  int frame_count = 0;
  auto last_fps_time = std::chrono::steady_clock::now();
  double fps = 0.0;

  // MPC 规划线程
  auto plan_thread = std::thread([&]() {
    auto t0 = std::chrono::steady_clock::now();
    
    while (!quit) {
      if (!target_queue.empty() && mode == io::GimbalMode::AUTO_AIM) {
        auto target = target_queue.front();
        auto gs = gimbal.state();
        auto plan = planner.plan(target, gs.bullet_speed);

        gimbal.send(
          plan.control, plan.fire, plan.yaw, plan.yaw_vel, plan.yaw_acc, plan.pitch, plan.pitch_vel,
          plan.pitch_acc);

        std::this_thread::sleep_for(10ms);
      } else
        std::this_thread::sleep_for(200ms);
    }
  });

  while (!exiter.exit()) {
    // FPS 计算
    frame_count++;
    auto now = std::chrono::steady_clock::now();
    if (tools::delta_time(now, last_fps_time) >= 1.0) {
        fps = frame_count / tools::delta_time(now, last_fps_time);
        frame_count = 0;
        last_fps_time = now;
    }

    mode = gimbal.mode();

    if (last_mode != mode) {
      tools::logger()->info("Switch to {}", gimbal.str(mode));
      last_mode = mode.load();
    }

    camera.read(img, t);
    if (img.empty()) continue; // 安全检查

    auto q = gimbal.q(t);
    auto gs = gimbal.state();
    recorder.record(img, q, t);
    solver.set_R_gimbal2world(q);

    // 2. 将 armors 和 targets 提到外部作用域，以便可视化代码访问
    std::list<auto_aim::Armor> armors;
    std::list<auto_aim::Target> targets;

    /// 自瞄
    if (mode.load() == io::GimbalMode::AUTO_AIM) {
      armors = yolo.detect(img);
      targets = tracker.track(armors, t);
      if (!targets.empty())
        target_queue.push(targets.front());
      else
        target_queue.push(std::nullopt);
    }

    /// 打符
    else if (mode.load() == io::GimbalMode::SMALL_BUFF || mode.load() == io::GimbalMode::BIG_BUFF) {
      // 打符的可视化比较复杂，这里暂不处理，只处理逻辑
      buff_solver.set_R_gimbal2world(q);
      auto power_runes = buff_detector.detect(img);
      buff_solver.solve(power_runes);

      auto_aim::Plan buff_plan;
      if (mode.load() == io::GimbalMode::SMALL_BUFF) {
        buff_small_target.get_target(power_runes, t);
        auto target_copy = buff_small_target;
        buff_plan = buff_aimer.mpc_aim(target_copy, t, gs, true);
      } else if (mode.load() == io::GimbalMode::BIG_BUFF) {
        buff_big_target.get_target(power_runes, t);
        auto target_copy = buff_big_target;
        buff_plan = buff_aimer.mpc_aim(target_copy, t, gs, true);
      }
      gimbal.send(
        buff_plan.control, buff_plan.fire, buff_plan.yaw, buff_plan.yaw_vel, buff_plan.yaw_acc,
        buff_plan.pitch, buff_plan.pitch_vel, buff_plan.pitch_acc);

    } else {
      gimbal.send(false, false, 0, 0, 0, 0, 0, 0);
    }

    // ==================== 可视化代码开始 ====================
    if (enable_display) {
        cv::Mat vis_img = img.clone();

        // A. 绘制识别到的装甲板 (绿色框) - 自瞄模式下有效
        for (const auto & armor : armors) {
            for (int i = 0; i < 4; i++) {
                cv::line(vis_img, armor.points[i], armor.points[(i + 1) % 4], cv::Scalar(0, 255, 0), 3);
            }
            cv::circle(vis_img, armor.center, 3, cv::Scalar(0, 255, 0), -1);
            tools::draw_text(vis_img, auto_aim::ARMOR_NAMES[armor.name], armor.points[0], cv::Scalar(0, 255, 0), 0.8, 2);
        }

        // B. 绘制追踪预测结果 (黄色框)
        if (!targets.empty()) {
            auto target = targets.front();
            std::vector<Eigen::Vector4d> predicted_armors = target.armor_xyza_list();
            
            for (const auto & xyza : predicted_armors) {
                // 重投影
                auto image_points = solver.reproject_armor(
                    xyza.head(3), xyza[3], target.armor_type, target.name
                );
                
                // 绘制预测框
                tools::draw_points(vis_img, image_points, {0, 255, 255}, 2);

                // 显示距离
                double distance = xyza.head(3).norm();
                if (!image_points.empty()) {
                    std::string dist_text = fmt::format("{:.2f}m", distance);
                    cv::Point text_pos = image_points[0];
                    text_pos.y -= 20; 
                    tools::draw_text(vis_img, dist_text, text_pos, {0, 255, 255}, 0.6, 2);
                }
            }
            
            // 追踪状态
            std::string state_info = fmt::format("State: {} | ID: {}", tracker.state(), auto_aim::ARMOR_NAMES[target.name]);
            tools::draw_text(vis_img, state_info, {20, 80}, {0, 255, 255}, 1.0, 2);
        }

        // C. 绘制 MPC 规划的瞄准点 (红色十字)
        // 注意：这里读取的是 Planner 内部记录的 debug_xyza
        // 它是 Planner 在生成轨迹时计算出的理想击打点
        Eigen::Vector4d plan_aim = planner.debug_xyza;
        if (plan_aim.norm() > 0.1) { // 简单判断是否有有效值
             auto aim_xyz = plan_aim.head(3);
             // 借用 reproject 投影中心点
             auto aim_proj_points = solver.reproject_armor(aim_xyz, 0, auto_aim::ArmorType::small, auto_aim::ArmorName::outpost);
             cv::Point2f aim_center = (aim_proj_points[0] + aim_proj_points[2]) / 2;

             // 使用 cv::Point2f 避免窄化转换警告
             cv::line(vis_img, cv::Point2f(aim_center.x - 15, aim_center.y), cv::Point2f(aim_center.x + 15, aim_center.y), {0, 0, 255}, 2);
             cv::line(vis_img, cv::Point2f(aim_center.x, aim_center.y - 15), cv::Point2f(aim_center.x, aim_center.y + 15), {0, 0, 255}, 2);
        }

        // D. 绘制 UI
        tools::draw_text(vis_img, fmt::format("FPS: {:.1f}", fps), {20, 40}, {255, 255, 255}, 1.0, 2);
        tools::draw_text(vis_img, fmt::format("Mode: {}", gimbal.str(mode)), {20, 120}, {255, 255, 255}, 1.0, 2);

        // E. 显示
        cv::resize(vis_img, vis_img, {}, 0.5, 0.5);
        cv::imshow("Standard MPC Debug", vis_img);
        
        if (cv::waitKey(1) == 'q') {
            break;
        }
    }
    // ==================== 可视化代码结束 ====================
  }

  quit = true;
  if (plan_thread.joinable()) plan_thread.join();
  gimbal.send(false, false, 0, 0, 0, 0, 0, 0);

  return 0;
}