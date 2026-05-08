#include <fmt/core.h>

#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/multithread/commandgener.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/debug_monitor.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"

using namespace std::chrono;

// 1. 命令行参数配置
const std::string keys =
  "{help h usage ? |      | 输出命令行参数说明}"
  "{display d      | true | 是否显示视频流}" 
  "{@config-path   | configs/standard3.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  
  // 获取是否显示的标志
  bool enable_display = cli.get<bool>("display");

  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;
  tools::Plotter plotter;
  tools::Recorder recorder;

  io::Gimbal gimbal(config_path);
  io::Camera camera(config_path);
  
  auto_aim::YOLO detector(config_path, false);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;

  auto mode = io::GimbalMode::IDLE;
  auto last_mode = io::GimbalMode::IDLE;

  // 用于计算FPS
  int frame_count = 0;
  auto last_fps_time = std::chrono::steady_clock::now();
  double fps = 0.0;
  std::chrono::steady_clock::time_point last_frame_timestamp;
  bool has_last_frame_timestamp = false;

  while (!exiter.exit()) {
    // 简单计算FPS
    frame_count++;
    auto now = std::chrono::steady_clock::now();
    if (tools::delta_time(now, last_fps_time) >= 1.0) {
        fps = frame_count / tools::delta_time(now, last_fps_time);
        frame_count = 0;
        last_fps_time = now;
    }

    camera.read(img, t);
    // 增加空图检查，防止程序崩溃
    if (img.empty()) continue;

    auto t_loop = std::chrono::steady_clock::now();

    if (has_last_frame_timestamp) {
      double frame_dt = tools::delta_time(t, last_frame_timestamp);
      if (frame_dt > 0.0 && frame_dt < 0.2) {
        shooter.update_frame_time(frame_dt);
      }
    }
    last_frame_timestamp = t;
    has_last_frame_timestamp = true;

    auto gs = gimbal.state();
    q = gimbal.q(t - 1ms);
    mode = gimbal.mode();

    // 根据下位机发送的 enemy_color 动态设置敌方颜色，0 时回退 yaml 默认值
    if (gs.enemy_color == 101) {
      tracker.set_enemy_color(auto_aim::Color::red);
    } else if (gs.enemy_color == 1) {
      tracker.set_enemy_color(auto_aim::Color::blue);
    }
    //tools::logger()->warn("enemy_color: {}", gs.enemy_color);
    //tools::logger()->warn("speed: {}", gs.bullet_speed);
    //tools::logger()->warn("enemy_color: {}", gs.enemy_color);
    //tools::logger()->warn("enemy_color: {}", gs.enemy_color);
    if (last_mode != mode) {
      tools::logger()->info("Switch to {}", gimbal.str(mode));
      last_mode = mode;
    }

    // recorder.record(img, q, t);

    solver.set_R_gimbal2world(q);

    Eigen::Vector3d ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);

    // 1. 识别
    auto t_detect = std::chrono::steady_clock::now();
    auto armors = detector.detect(img);
    auto t_after_detect = std::chrono::steady_clock::now();

    // 2. 追踪
    auto targets = tracker.track(armors, t);
    auto t_after_track = std::chrono::steady_clock::now();

    // 3. 瞄准
    auto command = aimer.aim(targets, t, gs.bullet_speed);
    auto t_after_aim = std::chrono::steady_clock::now();

    command.shoot = shooter.shoot(command, aimer, targets, ypr);
    auto t_after_shoot = std::chrono::steady_clock::now(); 
    
    WATCH("fire", command.shoot);

    gimbal.send(command);
    auto t_after_send = std::chrono::steady_clock::now();

    // ==================== 可视化代码开始 ====================
    if (enable_display) {
        // 克隆一份图像用于绘制，避免影响原图处理
        cv::Mat vis_img = img.clone();

        // A. 绘制识别到的装甲板 (绿色框)
        for (const auto & armor : armors) {
            // 绘制四点连线
            for (int i = 0; i < 4; i++) {
                cv::line(vis_img, armor.points[i], armor.points[(i + 1) % 4], cv::Scalar(0, 255, 0), 3);
            }
            // 绘制中心点
            cv::circle(vis_img, armor.center, 3, cv::Scalar(0, 255, 0), -1);
            
            // 显示装甲板名称 / 前哨站层级
            std::string text_info = auto_aim::ARMOR_NAMES[armor.name];
            
            // === 前哨站层级显示 ===
            if (armor.name == auto_aim::ArmorName::outpost && !targets.empty()) {
                auto t = targets.front();
                // 用 EKF 状态中的 z (ID 0 基准高度) 计算层级
                double base_z = t.ekf_x()[4];
                double diff = armor.xyz_in_world[2] - base_z;
                int layer = std::round(diff / 0.10);
                layer = std::max(0, std::min(2, layer));
                text_info += fmt::format(" L{}", layer);
            }
            // ============================

            tools::draw_text(vis_img, text_info, armor.points[0], cv::Scalar(0, 255, 0), 0.8, 2);
        }

        // B. 绘制追踪预测结果 (黄色框)
        if (!targets.empty()) {
            auto target = targets.front();
            
            // 获取目标所有装甲板的预测位置 (世界坐标系)
            // 前哨站的 aim_armor_xyza_list() 会在瞄准/显示阶段恢复层高
            std::vector<Eigen::Vector4d> predicted_armors = target.aim_armor_xyza_list();
            
            for (const auto & xyza : predicted_armors) {
                // 关键步骤：重投影 (Reprojection)
                auto image_points = solver.reproject_armor(
                    xyza.head(3), xyza[3], target.armor_type, target.name
                );
                
                // 绘制预测框
                tools::draw_points(vis_img, image_points, {0, 255, 255}, 2);
            }

            // 在左上角显示追踪状态
            std::string state_info = fmt::format("State: {} | ID: {}", tracker.state(), auto_aim::ARMOR_NAMES[target.name]);
            tools::draw_text(vis_img, state_info, {20, 80}, {0, 255, 255}, 1.0, 2);
            
            // 显示前哨站调试信息
            if (target.name == auto_aim::ArmorName::outpost) {
                 tools::draw_text(vis_img, fmt::format("EKF Z: {:.2f}m", target.ekf_x()[4]), {20, 110}, {0, 255, 255}, 0.8, 2);
            }
        }

        // C. 绘制最终打击点 (红色十字)
        // aimer.debug_aim_point 存储了经过反小陀螺/前哨站策略选择后的最终击打位置
        if (aimer.debug_aim_point.valid) {
            Eigen::Vector3d aim_xyz = aimer.debug_aim_point.xyza.head(3);
            
            // 同样使用重投影将其画在图上
            auto aim_proj_points = solver.reproject_armor(aim_xyz, 0, auto_aim::ArmorType::small, auto_aim::ArmorName::outpost);
            
            // 计算投影矩形的中心
            cv::Point2f aim_center = (aim_proj_points[0] + aim_proj_points[2]) / 2;
            
            // 画红十字
            cv::drawMarker(vis_img, aim_center, {0, 0, 255}, cv::MARKER_CROSS, 20, 3);
        }

        // D. 绘制 UI 信息
        tools::draw_text(vis_img, fmt::format("FPS: {:.1f}", fps), {20, 40}, {255, 255, 255}, 1.0, 2);
        tools::draw_text(vis_img, fmt::format("Mode: {}", gimbal.str(mode)), {20, 140}, {255, 255, 255}, 1.0, 2);

        // E. 显示图像 (缩小一半显示，防止超出屏幕)
        cv::resize(vis_img, vis_img, {}, 0.5, 0.5);
        cv::imshow("Auto Aim Debug", vis_img);
        
        // 必须加 waitKey，否则窗口不刷新
        if (cv::waitKey(1) == 'q') {
            break;
        }
    }
    // ==================== 可视化代码结束 ====================

    // 帧率计时输出
    double dt_detect = tools::delta_time(t_after_detect, t_detect) * 1000;
    double dt_track  = tools::delta_time(t_after_track, t_after_detect) * 1000;
    double dt_aim    = tools::delta_time(t_after_aim, t_after_track) * 1000;
    double dt_shoot  = tools::delta_time(t_after_shoot, t_after_aim) * 1000;
    double dt_send   = tools::delta_time(t_after_send, t_after_shoot) * 1000;
    double dt_total  = tools::delta_time(std::chrono::steady_clock::now(), t_loop) * 1000;
    WATCH("time_detect_ms", dt_detect);
    WATCH("time_track_ms", dt_track);
    WATCH("time_aim_ms", dt_aim);
    WATCH("time_shoot_ms", dt_shoot);
    WATCH("time_send_ms", dt_send);
    WATCH("time_total_ms", dt_total);
    WATCH("fps", fps);

    FLUSH_DEBUG();


  }

  return 0;
}
