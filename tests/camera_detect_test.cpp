#include <fmt/core.h>

#include <chrono>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "tasks/auto_aim/detector.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tasks/auto_aim/solver.hpp" // [新增] 引入解算器
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp" // [新增] 引入绘图工具
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明 }"
  "{@config-path   | configs/sentry.yaml    | yaml配置文件的路径}"
  "{tradition t    |  false                 | 是否使用传统方法识别}";

int main(int argc, char * argv[])
{
  // 读取命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto config_path = cli.get<std::string>(0);
  auto use_tradition = cli.get<bool>("tradition");

  tools::Exiter exiter;

  io::Camera camera(config_path);
  
  // [修改] 将 debug 参数设为 false，关闭内部弹窗，由我们在 main 中手动绘制
  auto_aim::Detector detector(config_path, false);
  auto_aim::YOLO yolo(config_path, false);
  
  // [新增] 初始化 PnP 解算器
  auto_aim::Solver solver(config_path);

  std::chrono::steady_clock::time_point timestamp;

  while (!exiter.exit()) {
    cv::Mat img;
    std::list<auto_aim::Armor> armors;

    camera.read(img, timestamp);

    if (img.empty()) break;

    auto last = std::chrono::steady_clock::now();

    // 1. 识别
    if (use_tradition)
      armors = detector.detect(img);
    else
      armors = yolo.detect(img);

    // 2. 解算位姿 (PnP)
    for (auto & armor : armors) {
        // 由于没有 IMU 数据，这里只能解算出相对于云台/相机的坐标
        // R_gimbal2world 默认为单位阵 (假设云台不动)
        solver.solve(armor);
    }

    auto now = std::chrono::steady_clock::now();
    auto dt = tools::delta_time(now, last);
    
    // 3. 可视化绘制
    cv::Mat vis_img = img.clone();
    
    for (const auto & armor : armors) {
        // 绘制装甲板四点
        tools::draw_points(vis_img, armor.points, {0, 255, 0}, 2);
        
        // 绘制中心点
        cv::circle(vis_img, armor.center, 4, {0, 255, 0}, -1);
        
        // 绘制类别和颜色
        std::string name_info = fmt::format("{} {}", 
            auto_aim::COLORS[armor.color], 
            auto_aim::ARMOR_NAMES[armor.name]);
        tools::draw_text(vis_img, name_info, armor.points[0], {0, 255, 255}, 0.8, 2);

        // [核心] 绘制位姿信息 (XYZ)
        // xyz_in_gimbal: 相对于云台中心的坐标 (单位: 米)
        auto xyz = armor.xyz_in_gimbal; 
        std::string dist_info = fmt::format("X:{:.2f} Y:{:.2f} Z:{:.2f}m", xyz[0], xyz[1], xyz[2]);
        
        // 在装甲板下方显示距离
        cv::Point2f text_pos = armor.points[3]; 
        text_pos.y += 30;
        tools::draw_text(vis_img, dist_info, text_pos, {0, 100, 255}, 0.6, 2);
    }

    // 显示 FPS
    tools::draw_text(vis_img, fmt::format("FPS: {:.2f}", 1 / dt), {20, 40}, {255, 255, 255});
    tools::logger()->info("{:.2f} fps | Armors: {}", 1 / dt, armors.size());

    // 缩小显示 (防止图片太大)
    cv::resize(vis_img, vis_img, {}, 0.5, 0.5);
    cv::imshow("Camera PnP Test", vis_img);

    auto key = cv::waitKey(33); // 33ms ~ 30fps
    if (key == 'q') break;
  }

  return 0;
}