#include "yolov5.hpp"

#include <fmt/chrono.h>
#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <cmath> // 引入 exp 用于 sigmoid

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
  
  if (yaml["threshold"].IsDefined()) binary_threshold_ = yaml["threshold"].as<double>();
  if (yaml["min_confidence"].IsDefined()) min_confidence_ = yaml["min_confidence"].as<double>();
  
  nms_threshold_ = 0.45; 
  if (yaml["nms_threshold"].IsDefined()) nms_threshold_ = yaml["nms_threshold"].as<double>();
  
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
    
  // 提前创建 infer_request_，避免每帧重复创建开辟内存
  infer_request_ = compiled_model_.create_infer_request();
}

std::list<Armor> YOLOV5::detect(const cv::Mat & raw_img, int frame_count)
{
  if (raw_img.empty()) {
    tools::logger()->warn("Empty img!, camera drop!");
    return std::list<Armor>();
  }

  cv::Mat bgr_img;
  cv::Rect current_roi = roi_; // 防止异形帧污染全局 ROI 配置
  if (use_roi_) {
    if (current_roi.width == -1) current_roi.width = raw_img.cols;
    if (current_roi.height == -1) current_roi.height = raw_img.rows;
    bgr_img = raw_img(current_roi);
  } else {
    bgr_img = raw_img;
  }

  // Letterbox 等比缩放补黑边
  auto x_scale = static_cast<double>(640) / bgr_img.rows;
  auto y_scale = static_cast<double>(640) / bgr_img.cols;
  auto scale = std::min(x_scale, y_scale);
  auto h = static_cast<int>(bgr_img.rows * scale);
  auto w = static_cast<int>(bgr_img.cols * scale);

  auto input = cv::Mat(640, 640, CV_8UC3, cv::Scalar(0, 0, 0));
  auto roi = cv::Rect(0, 0, w, h);
  cv::resize(bgr_img, input(roi), {w, h});
  
  ov::Tensor input_tensor(ov::element::u8, {1, 640, 640, 3}, input.data);

  // 推理
  infer_request_.set_input_tensor(input_tensor);
  infer_request_.infer();

  auto output_tensor = infer_request_.get_output_tensor();
  auto output_shape = output_tensor.get_shape();
  
  // 兼容不同模型导出张量排布 ([1, 25200, 22] vs [1, 22, 25200])
  cv::Mat output;
  if (output_shape[1] > output_shape[2]) {
      output = cv::Mat(output_shape[1], output_shape[2], CV_32F, output_tensor.data());
  } else {
      cv::Mat tmp(output_shape[1], output_shape[2], CV_32F, output_tensor.data());
      cv::transpose(tmp, output);
  }

  return parse(scale, output, raw_img, frame_count);
}

std::list<Armor> YOLOV5::parse(
  double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count)
{
  std::vector<int> color_ids, num_ids;
  std::vector<float> confidences;
  std::vector<cv::Rect> boxes;
  std::vector<std::vector<cv::Point2f>> armors_key_points;

  for (int r = 0; r < output.rows; r++) {
    // 1. 获取并处理置信度
    double score = output.at<float>(r, 8);
    // 如果模型没有做 sigmoid 激活，则手动激活
    if (score < 0.0 || score > 1.0) {
        score = sigmoid(score);
    }
    if (score < min_confidence_) continue;

    // 2. 解析颜色和类别
    cv::Mat color_scores = output.row(r).colRange(9, 13);     
    cv::Mat classes_scores = output.row(r).colRange(13, 22);  
    cv::Point class_id, color_id;
    double score_color, score_num;
    cv::minMaxLoc(classes_scores, NULL, &score_num, NULL, &class_id);
    cv::minMaxLoc(color_scores, NULL, &score_color, NULL, &color_id);
    
    // 3. 解析四个角点并按照顺时针 (TopLeft, TopRight, BottomRight, BottomLeft) 放入
    std::vector<cv::Point2f> armor_key_points;
    armor_key_points.push_back(cv::Point2f(output.at<float>(r, 0) / scale, output.at<float>(r, 1) / scale));
    armor_key_points.push_back(cv::Point2f(output.at<float>(r, 6) / scale, output.at<float>(r, 7) / scale));
    armor_key_points.push_back(cv::Point2f(output.at<float>(r, 4) / scale, output.at<float>(r, 5) / scale));
    armor_key_points.push_back(cv::Point2f(output.at<float>(r, 2) / scale, output.at<float>(r, 3) / scale));

    // 4. 根据角点计算 Bounding Box
    float min_x = armor_key_points[0].x, max_x = armor_key_points[0].x;
    float min_y = armor_key_points[0].y, max_y = armor_key_points[0].y;
    for (const auto& p : armor_key_points) {
        min_x = std::min(min_x, p.x); max_x = std::max(max_x, p.x);
        min_y = std::min(min_y, p.y); max_y = std::max(max_y, p.y);
    }
    cv::Rect rect(min_x, min_y, max_x - min_x, max_y - min_y);

    // 5. 颜色映射修正 (模型: 0蓝, 1红) -> 框架 (0蓝, 1红)
    int mapped_color = 2; // 默认为灰
    if (color_id.x == 0) mapped_color = 0;      // 蓝
    else if (color_id.x == 1) mapped_color = 1; // 红

    // 6. 类别映射修正 (模型标签从 1 开始排: 0代表1号)
    int mapped_num = 0;
    switch(class_id.x) {
        case 0: mapped_num = 0; break; // 1号 -> 映射给框架 ArmorName::one 
        case 1: mapped_num = 1; break; // 2号 -> 映射给框架 ArmorName::two 
        case 2: mapped_num = 2; break; // 3号 -> 映射给框架 ArmorName::three 
        case 3: mapped_num = 3; break; // 4号 -> 映射给框架 ArmorName::four 
        case 4: mapped_num = 4; break; // 5号 -> 映射给框架 ArmorName::five 
        case 5: mapped_num = 5; break; // 前哨站 -> 映射给框架 ArmorName::outpost
        case 6: mapped_num = 6; break; // 哨兵 -> 映射给框架 ArmorName::sentry
        case 7: mapped_num = 7; break; // 基地 -> 映射给框架 ArmorName::base
        case 8: mapped_num = 7; break; // 基地大 -> 映射给框架 ArmorName::base
        default: mapped_num = 0; break;
    }

    color_ids.emplace_back(mapped_color);
    num_ids.emplace_back(mapped_num);
    boxes.emplace_back(rect);
    confidences.emplace_back(score); 
    armors_key_points.emplace_back(armor_key_points);
  }

  // 7. NMS 非极大值抑制
  std::vector<int> indices;
  cv::dnn::NMSBoxes(boxes, confidences, min_confidence_, nms_threshold_, indices);

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
  
  // 8. 校验过滤与后处理
  for (auto it = armors.begin(); it != armors.end();) {
    if (!check_name(*it)) {
      it = armors.erase(it);
      continue;
    }

    if (!check_type(*it)) {
      it = armors.erase(it);
      continue;
    }
    
    // 如果需要传统灯条修正 (提升 PnP 精度)，通过 yaml 开关控制
    if (use_traditional_) detector_.detect(*it, bgr_img);

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