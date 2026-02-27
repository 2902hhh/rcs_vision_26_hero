#include "gimbal.hpp"

#include "tools/crc.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"
#include <cstring>  // for std::memcpy

namespace io
{
Gimbal::Gimbal(const std::string & config_path)
// 使用成员初始化列表来确保 mode_ 被正确初始化

: mode_(GimbalMode::IDLE) 
//陈庆年
{
  auto yaml = tools::load(config_path);
  auto com_port = tools::read<std::string>(yaml, "com_port");
  auto baudrate = tools::read<int>(yaml, "serial_baudrate");

  try {
    // === MODIFIED / 新增 ===
    // 在操作串口前加锁
    std::lock_guard<std::mutex> lock(mutex_);
    // =======================

    serial_.setPort(com_port);
    serial_.setBaudrate(baudrate);

        // === 新增代码：设置超时 ===
    // 创建一个 Timeout 对象.
    // inter_byte_timeout: 两个字节之间的最大间隔时间
    // read_timeout_constant: 固定的读取超时时间
    // read_timeout_multiplier: 每请求一个字节增加的超时时间
    // 这里的设置表示：读取操作最多等待 20ms。
    // 如果20ms内一个字节都没收到，serial_.read()就会返回0。
    serial::Timeout timeout = serial::Timeout::simpleTimeout(20); // 20毫秒
    serial_.setTimeout(timeout);
    // ========================

    serial_.open();
        tools::logger()->info("[Gimbal] Serial port {} opened with baudrate {}.", com_port, baudrate);

  } catch (const std::exception & e) {
    tools::logger()->error("[Gimbal] Failed to open serial: {}", e.what());
    exit(1);
  }

  thread_ = std::thread(&Gimbal::read_thread, this);

  queue_.pop();
  tools::logger()->info("[Gimbal] First q received.");
}

Gimbal::~Gimbal()
{
  quit_ = true;
  if (thread_.joinable()) thread_.join();
   // === MODIFIED / 新增 ===
  // 在操作串口前加锁
  std::lock_guard<std::mutex> lock(mutex_);
  // =======================
  serial_.close();
}

GimbalMode Gimbal::mode() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return mode_;
}

GimbalState Gimbal::state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

std::string Gimbal::str(GimbalMode mode) const
{
  switch (mode) {
    case GimbalMode::IDLE:
      return "IDLE";
    case GimbalMode::AUTO_AIM:
      return "AUTO_AIM";
    case GimbalMode::SMALL_BUFF:
      return "SMALL_BUFF";
    case GimbalMode::BIG_BUFF:
      return "BIG_BUFF";
    case GimbalMode::OUTPOST:
      return "OUTPOST";
    default:
      return "INVALID";
  }
}

Eigen::Quaterniond Gimbal::q(std::chrono::steady_clock::time_point t)
{
  while (true) {

    auto [q_a, t_a] = queue_.pop();
    auto [q_b, t_b] = queue_.front();
    auto t_ab = tools::delta_time(t_a, t_b);
    auto t_ac = tools::delta_time(t_a, t);
    auto k = t_ac / t_ab;
    Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();
    if (t < t_a) return q_c;
    if (!(t_a < t && t <= t_b)) continue;

    return q_c;
  }
}

void Gimbal::send(io::VisionToGimbal VisionToGimbal)
{
  // === MODIFIED / 新增 ===
  // 在写入串口前加锁
  std::lock_guard<std::mutex> lock(mutex_);
  // =======================
  tx_data_.mode = VisionToGimbal.mode;
  tx_data_.yaw = VisionToGimbal.yaw;  
  tx_data_.yaw_vel = VisionToGimbal.yaw_vel;
  tx_data_.yaw_acc = VisionToGimbal.yaw_acc;
  tx_data_.pitch = VisionToGimbal.pitch;  
  tx_data_.pitch_vel = VisionToGimbal.pitch_vel;
  tx_data_.pitch_acc = VisionToGimbal.pitch_acc;
  tx_data_.crc16 = tools::get_crc16(
    reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_) - sizeof(tx_data_.crc16));

  try {
    serial_.write(reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_));
  } catch (const std::exception & e) {
    tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
  }
}

void Gimbal::send(
  bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
  float pitch_acc)
{
  // === MODIFIED / 新增 ===
  // 在写入串口前加锁
  std::lock_guard<std::mutex> lock(mutex_);
  // =======================
  tx_data_.mode = control ? (fire ? 2 : 1) : 0;
  tx_data_.yaw = yaw*57.29578f;  // 转换为角度
  tx_data_.yaw_vel = yaw_vel*57.29578f;
  tx_data_.yaw_acc = yaw_acc*57.29578f;
  tx_data_.pitch = pitch*57.29578f;  // 转换为角度
  tx_data_.pitch_vel = pitch_vel;
  tx_data_.pitch_acc = pitch_acc;
  tx_data_.crc16 = tools::get_crc16(
    reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_) - sizeof(tx_data_.crc16));

  try {
    serial_.write(reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_));
  } catch (const std::exception & e) {
    tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
  }
}

void Gimbal::send(const io::Command &command)
{
  // 这个函数直接调用已有的、更详细的 send 函数
  // 将 Command 结构体的数据映射到对应的参数上
  // 对于 Command 中没有的速度和加速度信息，我们传递 0.0f
  this->send(
    command.control,
    command.shoot,
    // === 修改这里 ===
    // 原代码: 0.0f, 0.0f, ...
    static_cast<float>(command.yaw),
    static_cast<float>(command.yaw_vel), // 发送计算出的速度
    static_cast<float>(command.yaw_acc), // 发送加速度 (0)
    
    static_cast<float>(command.pitch),
    
    // === 修改这里 ===
    static_cast<float>(command.pitch_vel),
    static_cast<float>(command.pitch_acc)
  );
}

// bool Gimbal::read(uint8_t * buffer, size_t size)
// {
//   try {
//     return serial_.read(buffer, size) == size;
//   } catch (const std::exception & e) {
//     // tools::logger()->warn("[Gimbal] Failed to read serial: {}", e.what());
//     return false;
//   }
// }


bool Gimbal::read(uint8_t * buffer, size_t size)
{
  // === MODIFIED / 新增 ===
  // 在读取串口前加锁
  std::lock_guard<std::mutex> lock(mutex_);
  // =======================
  size_t got = 0;
  try {
    while (got < size) {
      size_t n = serial_.read(buffer + got, size - got);
      if (n == 0) {
        // 到达串口超时窗口（由 setTimeout 控制），认为这次读取失败
        tools::logger()->warn("[Gimbal::read] Read timed out waiting for data."); // <--- 第一条日志
        return false;
      }
      got += n;
    }
    return true;
  } catch (const std::exception & e) {
    // tools::logger()->warn("[Gimbal] Failed to read serial: {}", e.what());
    return false;
  }
}


// void Gimbal::read_thread()
// {
//   tools::logger()->info("[Gimbal] read_thread started.");
//   int error_count = 0;

//   while (!quit_) {
//     if (error_count > 5000) {
//       error_count = 0;
//       tools::logger()->warn("[Gimbal] Too many errors, attempting to reconnect...");
//       reconnect();
//       continue;
//     }
// //    tools::logger()->debug("[read_thread] CHECKPOINT 1: Attempting to read frame header..."); // <--- 第二条日志
//     if (!read(reinterpret_cast<uint8_t *>(&rx_data_), sizeof(rx_data_.head))) {
//       error_count++;
//       continue;
//     }

//     if (rx_data_.head[0] != 'S' || rx_data_.head[1] != 'P') continue;

//     auto t = std::chrono::steady_clock::now();
// //    tools::logger()->debug("[read_thread] CHECKPOINT 2: Header OK. Attempting to read frame body..."); // <--- 第三条日志
//     if (!read(
//           reinterpret_cast<uint8_t *>(&rx_data_) + sizeof(rx_data_.head),
//           sizeof(rx_data_) - sizeof(rx_data_.head))) {
//       error_count++;
//       continue;
//     }

//     if (!tools::check_crc16(reinterpret_cast<uint8_t *>(&rx_data_), sizeof(rx_data_))) {
//       tools::logger()->debug("[Gimbal] CRC16 check failed.");
//       continue;
//     }

//     error_count = 0;
//     Eigen::Quaterniond q(rx_data_.q[0], rx_data_.q[1], rx_data_.q[2], rx_data_.q[3]);
//     queue_.push({q, t});

//     std::lock_guard<std::mutex> lock(mutex_);

//     state_.yaw = rx_data_.yaw;
//     state_.yaw_vel = rx_data_.yaw_vel;
//     state_.pitch = rx_data_.pitch;
//     state_.pitch_vel = rx_data_.pitch_vel;
//     state_.bullet_speed = rx_data_.bullet_speed;
//     state_.bullet_count = rx_data_.bullet_count;

//     switch (rx_data_.mode) {
//       case 0:
//         mode_ = GimbalMode::IDLE;
//         break;
//       case 1:
//         mode_ = GimbalMode::AUTO_AIM;
//         break;
//       case 2:
//         mode_ = GimbalMode::SMALL_BUFF;
//         break;
//       case 3:
//         mode_ = GimbalMode::BIG_BUFF;
//         break;
//       case 4:
//         mode_ = GimbalMode::OUTPOST;
//         break;
//       default:
//         mode_ = GimbalMode::IDLE;
//         tools::logger()->warn("[Gimbal] Invalid mode: {}", rx_data_.mode);
//         break;
//     }
//   }

//   tools::logger()->info("[Gimbal] read_thread stopped.");
// }

// ==================== 替换为这个新函数 ====================
void Gimbal::read_thread()
{
  tools::logger()->info("[Gimbal] read_thread started.");
  
  uint8_t frame_buffer[sizeof(GimbalToVision)];
  
  enum class ParseState {
    WAITING_FOR_S,
    WAITING_FOR_P
  };
  ParseState state = ParseState::WAITING_FOR_S;

  uint8_t single_byte;

  while (!quit_) {
    try {
      if (serial_.read(&single_byte, 1) == 0) {
        continue;
      }
    } catch (const std::exception& e) {
      tools::logger()->warn("[Gimbal] Exception during single byte read: {}. Attempting to reconnect...", e.what());
      reconnect();
      state = ParseState::WAITING_FOR_S;
      continue;
    }

    if (state == ParseState::WAITING_FOR_S) {
      if (single_byte == 'S') {
        state = ParseState::WAITING_FOR_P;
      }
    } else if (state == ParseState::WAITING_FOR_P) {
      if (single_byte == 'P') {
        frame_buffer[0] = 'S';
        frame_buffer[1] = 'P';

        size_t bytes_to_read = sizeof(GimbalToVision) - 2;
        
        if (read(frame_buffer + 2, bytes_to_read)) {
            if (tools::check_crc16(frame_buffer, sizeof(GimbalToVision))) {
                auto t = std::chrono::steady_clock::now();
                
                std::memcpy(&rx_data_, frame_buffer, sizeof(GimbalToVision));

                Eigen::Quaterniond q(rx_data_.q[0], rx_data_.q[1], rx_data_.q[2], rx_data_.q[3]);
                queue_.push({q, t});

                // ============== 这里是修正点 ==============
                std::lock_guard<std::mutex> lock(mutex_); // 使用 :: 而不是 .
                // =======================================

                state_.yaw = rx_data_.yaw;
                state_.yaw_vel = rx_data_.yaw_vel;
                state_.pitch = rx_data_.pitch;
                state_.pitch_vel = rx_data_.pitch_vel;
                state_.bullet_speed = rx_data_.bullet_speed;
                state_.bullet_count = rx_data_.bullet_count;
                
                 switch (rx_data_.mode) {
                    case 0: mode_ = GimbalMode::IDLE; break;
                    case 1: mode_ = GimbalMode::AUTO_AIM; break;
                    case 2: mode_ = GimbalMode::SMALL_BUFF; break;
                    case 3: mode_ = GimbalMode::BIG_BUFF; break;
                    case 4: mode_ = GimbalMode::OUTPOST; break;
                    default:
                        mode_ = GimbalMode::IDLE;
                        tools::logger()->warn("[Gimbal] Invalid mode: {}", rx_data_.mode);
                        break;
                }
            } else {
                tools::logger()->debug("[Gimbal] CRC16 check failed after finding header.");
            }
        }
        state = ParseState::WAITING_FOR_S;

      } else {
        state = (single_byte == 'S') ? ParseState::WAITING_FOR_P : ParseState::WAITING_FOR_S;
      }
    }
  }
  tools::logger()->info("[Gimbal] read_thread stopped.");
}

void Gimbal::reconnect()
{
   // === MODIFIED / 新增 ===
  // 锁住整个重连过程
  //std::lock_guard<std::mutex> lock(mutex_);
  // =======================
  int max_retry_count = 10;
  for (int i = 0; i < max_retry_count && !quit_; ++i) {
    tools::logger()->warn("[Gimbal] Reconnecting serial, attempt {}/{}...", i + 1, max_retry_count);
    try {
      serial_.close();
      std::this_thread::sleep_for(std::chrono::seconds(1));
    } catch (...) {
    }

    try {
      serial_.open();  // 尝试重新打开
      queue_.clear();
      tools::logger()->info("[Gimbal] Reconnected serial successfully.");
      break;
    } catch (const std::exception & e) {
      tools::logger()->warn("[Gimbal] Reconnect failed: {}", e.what());
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }
}

}  // namespace io