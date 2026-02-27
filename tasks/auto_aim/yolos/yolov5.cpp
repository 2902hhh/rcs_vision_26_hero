#include "yolov5.hpp"

#include <fmt/chrono.h>
#include <yaml-cpp/yaml.h>

#include <filesystem>

#include "tools/img_tools.hpp"
#include "tools/logger.hpp"

namespace auto_aim
{
YOLOV5::YOLOV5(const std::string & config_path, bool debug)
: debug_(debug), detector_(config_path, false)
{
  auto yaml = YAML::LoadFile(config_path);

  model_path_ = yaml["yolov5_model_path"].as<std::string>();
  device_ = yaml["device"].as<std::string>();
  
  // 兼容不同配置文件的参数读取
  if (yaml["threshold"].IsDefined()) binary_threshold_ = yaml["threshold"].as<double>();
  if (yaml["min_confidence"].IsDefined()) min_confidence_ = yaml["min_confidence"].as<double>();
  
  int x = 0, y = 0, width = 0, height = 0;
  if (yaml["roi"].IsDefined()) {
      x = yaml["roi"]["x"].as<int>();
      y = yaml["roi"]["y"].as<int>();
      width = yaml["roi"]["width"].as<int>();
      height = yaml["roi"]["height"].as<int>();
  }
  
  if (yaml["use_roi"].IsDefined()) use_roi_ = yaml["use_roi"].as<bool>();
  if (yaml["use_traditional"].IsDefined()) use_traditional_ = yaml["use_traditional"].as<bool>();
  
  roi_ = cv::Rect(x, y, width, height);
  offset_ = cv::Point2f(x, y);

  save_path_ = "imgs";
  std::filesystem::create_directory(save_path_);
  auto model = core_.read_model(model_path_);
  ov::preprocess::PrePostProcessor ppp(model);
  auto & input = ppp.input();

  // 这里的 640 必须与你的模型导出尺寸一致
  input.tensor()
    .set_element_type(ov::element::u8)
    .set_shape({1, 640, 640, 3}) 
    .set_layout("NHWC")
    .set_color_format(ov::preprocess::ColorFormat::BGR);

  input.model().set_layout("NCHW");

  input.preprocess()
    .convert_element_type(ov::element::f32)
    .convert_color(ov::preprocess::ColorFormat::RGB)
    .scale(255.0);

  model = ppp.build();
  compiled_model_ = core_.compile_model(
    model, device_, ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY));
}

std::list<Armor> YOLOV5::detect(const cv::Mat & raw_img, int frame_count)
{
  if (raw_img.empty()) {
    tools::logger()->warn("Empty img!, camera drop!");
    return std::list<Armor>();
  }

  cv::Mat bgr_img;
  if (use_roi_) {
    if (roi_.width == -1) roi_.width = raw_img.cols;
    if (roi_.height == -1) roi_.height = raw_img.rows;
    bgr_img = raw_img(roi_);
  } else {
    bgr_img = raw_img;
  }

  // 必须与模型输入尺寸一致 (640)
  auto x_scale = static_cast<double>(640) / bgr_img.rows;
  auto y_scale = static_cast<double>(640) / bgr_img.cols;
  auto scale = std::min(x_scale, y_scale);
  auto h = static_cast<int>(bgr_img.rows * scale);
  auto w = static_cast<int>(bgr_img.cols * scale);

  // preprocess
  auto input = cv::Mat(640, 640, CV_8UC3, cv::Scalar(0, 0, 0));
  auto roi = cv::Rect(0, 0, w, h);
  cv::resize(bgr_img, input(roi), {w, h});
  
  ov::Tensor input_tensor(ov::element::u8, {1, 640, 640, 3}, input.data);

  // infer
  auto infer_request = compiled_model_.create_infer_request();
  infer_request.set_input_tensor(input_tensor);
  infer_request.infer();

  // postprocess
  auto output_tensor = infer_request.get_output_tensor();
  auto output_shape = output_tensor.get_shape();
  cv::Mat output(output_shape[1], output_shape[2], CV_32F, output_tensor.data());

  return parse(scale, output, raw_img, frame_count);
}

std::list<Armor> YOLOV5::parse(
  double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count)
{
  std::vector<int> color_ids, num_ids;
  std::vector<float> confidences;
  std::vector<cv::Rect> boxes;
  std::vector<std::vector<cv::Point2f>> armors_key_points;

  // === 关键修改：自动判断模型类型 ===
  // 标准 YOLO 检测模型 (xywh+conf+cls) 通常列数较少 (6~10)
  // RM 装甲板模型 (xy*4 + conf + colors + nums) 通常列数 > 20
  bool is_standard_detect = output.cols < 15; 

  for (int r = 0; r < output.rows; r++) {
    if (is_standard_detect) {
        // --- 逻辑分支 A: 弹丸/标准检测模型 ---
        // 格式通常为: [cx, cy, w, h, conf, class_score...]
        // 注意：OpenVINO 导出的 raw output 可能没做 sigmoid，视导出参数而定。
        // 这里假设是标准的 flat output
        
        float conf = output.at<float>(r, 4);
        if (conf < min_confidence_) continue; // 使用配置文件的置信度

        float cx = output.at<float>(r, 0);
        float cy = output.at<float>(r, 1);
        float w = output.at<float>(r, 2);
        float h = output.at<float>(r, 3);

        // 还原坐标
        float left = (cx - w / 2) / scale;
        float top = (cy - h / 2) / scale;
        float width = w / scale;
        float height = h / scale;

        cv::Rect rect(left, top, width, height);
        boxes.emplace_back(rect);
        confidences.emplace_back(conf);
        
        // 伪造装甲板数据以通过后续过滤器
        color_ids.push_back(0); // Blue (0)
        num_ids.push_back(2);   // ID 2 (Three), type=Small -> 这是一个"安全"的ID，不容易被过滤

        // 伪造4个关键点 (用矩形角点代替)
        std::vector<cv::Point2f> kps;
        kps.push_back(cv::Point2f(left, top));           // TL
        kps.push_back(cv::Point2f(left + width, top));   // TR
        kps.push_back(cv::Point2f(left + width, top + height)); // BR
        kps.push_back(cv::Point2f(left, top + height));  // BL
        armors_key_points.emplace_back(kps);

    } else {
        // --- 逻辑分支 B: 原有装甲板 Pose 模型 ---
        double score = output.at<float>(r, 8);
        score = sigmoid(score);

        if (score < score_threshold_) continue;

        std::vector<cv::Point2f> armor_key_points;

        //颜色和类别独热向量
        cv::Mat color_scores = output.row(r).colRange(9, 13);     //color
        cv::Mat classes_scores = output.row(r).colRange(13, 22);  //num
        cv::Point class_id, color_id;
        double score_color, score_num;
        cv::minMaxLoc(classes_scores, NULL, &score_num, NULL, &class_id);
        cv::minMaxLoc(color_scores, NULL, &score_color, NULL, &color_id);
        
        armor_key_points.push_back(
          cv::Point2f(output.at<float>(r, 0) / scale, output.at<float>(r, 1) / scale));
        armor_key_points.push_back(
          cv::Point2f(output.at<float>(r, 6) / scale, output.at<float>(r, 7) / scale));
        armor_key_points.push_back(
          cv::Point2f(output.at<float>(r, 4) / scale, output.at<float>(r, 5) / scale));
        armor_key_points.push_back(
          cv::Point2f(output.at<float>(r, 2) / scale, output.at<float>(r, 3) / scale));

        // 计算包围盒
        float min_x = armor_key_points[0].x, max_x = armor_key_points[0].x;
        float min_y = armor_key_points[0].y, max_y = armor_key_points[0].y;
        for (const auto& p : armor_key_points) {
            min_x = std::min(min_x, p.x); max_x = std::max(max_x, p.x);
            min_y = std::min(min_y, p.y); max_y = std::max(max_y, p.y);
        }

        cv::Rect rect(min_x, min_y, max_x - min_x, max_y - min_y);

        color_ids.emplace_back(color_id.x);
        num_ids.emplace_back(class_id.x);
        boxes.emplace_back(rect);
        confidences.emplace_back(score);
        armors_key_points.emplace_back(armor_key_points);
    }
  }

  std::vector<int> indices;
  cv::dnn::NMSBoxes(boxes, confidences, score_threshold_, nms_threshold_, indices);

  std::list<Armor> armors;
  for (const auto & i : indices) {
    if (use_roi_) {
      armors.emplace_back(
        color_ids[i], num_ids[i], confidences[i], boxes[i], armors_key_points[i], offset_);
    } else {
      armors.emplace_back(color_ids[i], num_ids[i], confidences[i], boxes[i], armors_key_points[i]);
    }
  }

  tmp_img_ = bgr_img;
  
  // 过滤逻辑
  for (auto it = armors.begin(); it != armors.end();) {
    // 如果是弹丸模式(标准检测)，可能需要跳过 check_name/check_type，或者确保伪造的数据能通过
    // 这里保留检查，因为我们上面伪造了 ID=2 (ArmorName::three)
    if (!check_name(*it)) {
      it = armors.erase(it);
      continue;
    }

    if (!check_type(*it)) {
      it = armors.erase(it);
      continue;
    }
    
    // 传统视觉矫正只对装甲板有效，弹丸不需要
    if (use_traditional_ && !is_standard_detect) detector_.detect(*it, bgr_img);

    it->center_norm = get_center_norm(bgr_img, it->center);
    ++it;
  }

  if (debug_) draw_detections(bgr_img, armors, frame_count);

  return armors;
}

bool YOLOV5::check_name(const Armor & armor) const
{
  auto name_ok = armor.name != ArmorName::not_armor;
  auto confidence_ok = armor.confidence > min_confidence_;
  return name_ok && confidence_ok;
}

bool YOLOV5::check_type(const Armor & armor) const
{
  auto name_ok = (armor.type == ArmorType::small)
                   ? (armor.name != ArmorName::one && armor.name != ArmorName::base)
                   : (armor.name != ArmorName::two && armor.name != ArmorName::sentry &&
                      armor.name != ArmorName::outpost);
  return name_ok;
}

cv::Point2f YOLOV5::get_center_norm(const cv::Mat & bgr_img, const cv::Point2f & center) const
{
  auto h = bgr_img.rows;
  auto w = bgr_img.cols;
  return {center.x / w, center.y / h};
}

void YOLOV5::draw_detections(
  const cv::Mat & img, const std::list<Armor> & armors, int frame_count) const
{
  auto detection = img.clone();
  tools::draw_text(detection, fmt::format("[{}]", frame_count), {10, 30}, {255, 255, 255});
  for (const auto & armor : armors) {
    auto info = fmt::format(
      "{:.2f} {} {} {}", armor.confidence, COLORS[armor.color], ARMOR_NAMES[armor.name],
      ARMOR_TYPES[armor.type]);
    tools::draw_points(detection, armor.points, {0, 255, 0});
    tools::draw_text(detection, info, armor.center, {0, 255, 0});
  }

  if (use_roi_) {
    cv::Scalar green(0, 255, 0);
    cv::rectangle(detection, roi_, green, 2);
  }
  cv::resize(detection, detection, {}, 0.5, 0.5); 
  cv::imshow("detection", detection);
}

void YOLOV5::save(const Armor & armor) const
{
  auto file_name = fmt::format("{:%Y-%m-%d_%H-%M-%S}", std::chrono::system_clock::now());
  auto img_path = fmt::format("{}/{}_{}.jpg", save_path_, armor.name, file_name);
  cv::imwrite(img_path, tmp_img_);
}

double YOLOV5::sigmoid(double x)
{
  if (x > 0)
    return 1.0 / (1.0 + exp(-x));
  else
    return exp(x) / (1.0 + exp(x));
}

std::list<Armor> YOLOV5::postprocess(
  double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count)
{
  return parse(scale, output, bgr_img, frame_count);
}

}  // namespace auto_aim