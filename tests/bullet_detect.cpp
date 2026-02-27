#include <fmt/core.h>
#include <opencv2/opencv.hpp>
#include <deque>

#include "io/camera.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

// 定义一个简单的点结构，用于画轨迹
struct BulletPoint {
    cv::Point2f pt;
    std::chrono::steady_clock::time_point t;
};

const std::string keys =
  "{help h usage ? |                     | 输出命令行参数说明 }"
  "{@config-path   | configs/bullet_test.yaml | yaml配置文件的路径}";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto config_path = cli.get<std::string>(0);

  tools::Exiter exiter;
  
  // 1. 初始化相机 (使用你现有的相机配置，比如 hero.yaml 里的相机参数)
  // 如果 bullet.yaml 里没有相机参数，这里可以直接传 configs/hero.yaml
  // 但为了加载弹丸模型，我们需要 bullet.yaml。
  // 建议把弹丸模型参数合并到 configs/hero.yaml 里，或者这里硬编码一下路径。
  io::Camera camera("configs/hero.yaml"); 

  // 2. 初始化弹丸检测器 (加载 bullet.yaml)
  // 注意：这里 debug=false，我们自己画图
  auto_aim::YOLO bullet_detector(config_path, false); 

  std::chrono::steady_clock::time_point timestamp;
  std::deque<BulletPoint> trajectory; // 存储历史弹丸点

  while (!exiter.exit()) {
    cv::Mat img;
    camera.read(img, timestamp);
    if (img.empty()) continue;

    // 3. 推理
    auto bullets = bullet_detector.detect(img);

    // 4. 记录轨迹
    // 假设一帧里只有一个弹丸（或者取置信度最高的）
    if (!bullets.empty()) {
        // 按置信度排序，取最高的
        bullets.sort([](const auto& a, const auto& b){ return a.confidence > b.confidence; });
        
        auto best_bullet = bullets.front();
        trajectory.push_back({best_bullet.center, timestamp});
    }

    // 5. 绘制
    cv::Mat vis_img = img.clone();
    
    // 画当前识别框
    for (const auto& b : bullets) {
        cv::rectangle(vis_img, b.box, {0, 0, 255}, 2); // 红色框
        cv::putText(vis_img, fmt::format("{:.2f}", b.confidence), b.points[0], 
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, {0, 0, 255}, 1);
    }

    // 画历史轨迹 (尾迹效果)
    for (size_t i = 1; i < trajectory.size(); ++i) {
        // 根据时间衰减颜色或粗细
        cv::line(vis_img, trajectory[i-1].pt, trajectory[i].pt, {0, 255, 255}, 2);
    }

    // 清理太久远的轨迹 (比如 1秒前的)
    while (!trajectory.empty() && 
           tools::delta_time(timestamp, trajectory.front().t) > 1.0) {
        trajectory.pop_front();
    }

    cv::imshow("Bullet Detector", vis_img);
    if (cv::waitKey(1) == 'q') break;
  }

  return 0;
}