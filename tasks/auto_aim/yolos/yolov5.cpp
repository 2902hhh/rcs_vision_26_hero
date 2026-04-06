#include "yolov5.hpp"

#include <fmt/chrono.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
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
  constexpr int OUTPUT_DIMS = 23;

  if (output.cols != OUTPUT_DIMS) {
    if (output.rows == OUTPUT_DIMS) {
      cv::transpose(output, output);
    } else {
      tools::logger()->warn(
        "YOLOV5 output shape mismatch: rows={}, cols={}, expected cols={}", output.rows,
        output.cols, OUTPUT_DIMS);
      return std::list<Armor>();
    }
  }

  std::vector<int> color_ids, num_ids;
  std::vector<float> confidences;
  std::vector<cv::Rect> boxes;
  std::vector<std::vector<cv::Point2f>> armors_key_points;

  const float conf_threshold = std::max(score_threshold_, static_cast<float>(min_confidence_));

  for (int r = 0; r < output.rows; r++) {
    double score = output.at<float>(r, 8);
    if (score < 0.0 || score > 1.0) {
      score = sigmoid(score);
    }

    if (score < conf_threshold) continue;

    std::vector<cv::Point2f> armor_key_points;

    // YOLOV5 输出定义：
    // 0~7: 四个关键点坐标，顺序 TL->BL->BR->TR（从左上角逆时针）
    // 8: 置信度
    // 9~12: 颜色（红、蓝、灰、紫）
    // 13~22: 数字/类型（G,1,2,3,4,5,O,Bs,Bb,other）
    cv::Mat color_scores = output.row(r).colRange(9, 13);
    cv::Mat classes_scores = output.row(r).colRange(13, 23);
    cv::Point class_id, color_id;
    double score_color, score_num;
    cv::minMaxLoc(classes_scores, nullptr, &score_num, nullptr, &class_id);
    cv::minMaxLoc(color_scores, nullptr, &score_color, nullptr, &color_id);

    // 转换为 Armor 构造函数期望顺序：(TL, TR, BR, BL)
    armor_key_points.push_back(
      cv::Point2f(output.at<float>(r, 0) / scale, output.at<float>(r, 1) / scale));
    armor_key_points.push_back(
      cv::Point2f(output.at<float>(r, 6) / scale, output.at<float>(r, 7) / scale));
    armor_key_points.push_back(
      cv::Point2f(output.at<float>(r, 4) / scale, output.at<float>(r, 5) / scale));
    armor_key_points.push_back(
      cv::Point2f(output.at<float>(r, 2) / scale, output.at<float>(r, 3) / scale));

    float min_x = armor_key_points[0].x;
    float max_x = armor_key_points[0].x;
    float min_y = armor_key_points[0].y;
    float max_y = armor_key_points[0].y;
    for (const auto & p : armor_key_points) {
      min_x = std::min(min_x, p.x);
      max_x = std::max(max_x, p.x);
      min_y = std::min(min_y, p.y);
      max_y = std::max(max_y, p.y);
    }

    cv::Rect rect(min_x, min_y, max_x - min_x, max_y - min_y);

    // 模型输出颜色: 0红, 1蓝, 2灰, 3紫
    // Armor(color_id) 约定: 0蓝, 1红, 2灭(灰/紫)
    int mapped_color = 0;
    if (color_id.x == 0) {
      mapped_color = 1;
    } else if (color_id.x == 1) {
      mapped_color = 0;
    } else {
      mapped_color = 2;
    }

    // 模型输出类别: 0:G, 1:1, 2:2, 3:3, 4:4, 5:5, 6:O, 7:Bs, 8:Bb, 9:other
    // Armor(num_id) 约定: 0->sentry, 1..5->one..five, 6->outpost, 7->base
    int mapped_num = 0;
    switch (class_id.x) {
      case 0:
        mapped_num = 0;
        break;
      case 1:
        mapped_num = 1;
        break;
      case 2:
        mapped_num = 2;
        break;
      case 3:
        mapped_num = 3;
        break;
      case 4:
        mapped_num = 4;
        break;
      case 5:
        mapped_num = 5;
        break;
      case 6:
        mapped_num = 6;
        break;
      case 7:
        mapped_num = 7;
        break;
      case 8:
        mapped_num = 7;
        break;
      default:
        mapped_num = 0;
        break;
    }

    color_ids.emplace_back(mapped_color);
    num_ids.emplace_back(mapped_num);
    boxes.emplace_back(rect);
    confidences.emplace_back(score);
    armors_key_points.emplace_back(armor_key_points);
  }

  std::vector<int> indices;
  cv::dnn::NMSBoxes(boxes, confidences, conf_threshold, nms_threshold_, indices);

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
    if (!check_name(*it)) {
      it = armors.erase(it);
      continue;
    }

    if (!check_type(*it)) {
      it = armors.erase(it);
      continue;
    }
    
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