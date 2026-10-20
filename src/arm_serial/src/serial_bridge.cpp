#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cerrno>
#include <condition_variable>
#include <cstring>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "arm_interfaces/msg/mcu_status.hpp"
#include "arm_interfaces/srv/send_joints.hpp"
#include "arm_serial/frame_codec.hpp"

namespace
{

constexpr size_t kStatusPayloadSize = 38;

uint16_t read_u16(const std::vector<uint8_t> & data, size_t offset)
{
  return static_cast<uint16_t>(
    data[offset] | (static_cast<uint16_t>(data[offset + 1]) << 8));
}

uint32_t read_u32(const std::vector<uint8_t> & data, size_t offset)
{
  return static_cast<uint32_t>(data[offset]) |
         (static_cast<uint32_t>(data[offset + 1]) << 8) |
         (static_cast<uint32_t>(data[offset + 2]) << 16) |
         (static_cast<uint32_t>(data[offset + 3]) << 24);
}

float read_f32(const std::vector<uint8_t> & data, size_t offset)
{
  const uint32_t raw = read_u32(data, offset);
  float value = 0.0F;
  std::memcpy(&value, &raw, sizeof(value));
  return value;
}

void append_f32(std::vector<uint8_t> & data, float value)
{
  uint32_t raw = 0;
  std::memcpy(&raw, &value, sizeof(raw));
  for (int i = 0; i < 4; ++i) {
    data.push_back(static_cast<uint8_t>((raw >> (8 * i)) & 0xFF));
  }
}

speed_t baud_to_speed(int baud)
{
  switch (baud) {
    case 9600: return B9600;
    case 57600: return B57600;
    case 115200: return B115200;
    default: return 0;
  }
}

}  // namespace

class SerialBridge : public rclcpp::Node
{
public:
  SerialBridge()
  : Node("serial_bridge")
  {
    port_ = declare_parameter<std::string>("port", "/tmp/ttyROS");
    const int baud = declare_parameter<int>("baud", 115200);
    ack_timeout_ms_ = declare_parameter<int>("ack_timeout_ms", 500);
    retries_ = declare_parameter<int>("retries", 3);
    if (retries_ < 1) {
      retries_ = 1;
    }

    status_pub_ = create_publisher<arm_interfaces::msg::McuStatus>("/mcu/status", 10);
    joints_pub_ = create_publisher<sensor_msgs::msg::JointState>("/mcu/joint_states", 10);
    send_joints_srv_ = create_service<arm_interfaces::srv::SendJoints>(
      "/serial/send_joints",
      std::bind(&SerialBridge::handle_send_joints, this,
      std::placeholders::_1, std::placeholders::_2));
    ping_srv_ = create_service<std_srvs::srv::Trigger>(
      "/serial/ping",
      std::bind(&SerialBridge::handle_ping, this,
      std::placeholders::_1, std::placeholders::_2));

    fd_ = open_serial(port_, baud);
    if (fd_ < 0) {
      RCLCPP_ERROR(get_logger(), "无法打开串口 %s（socat 是否已启动？）",
        port_.c_str());
      return;
    }
    RCLCPP_INFO(get_logger(), "串口 %s 已打开，baud=%d，帧头 AA55", port_.c_str(), baud);
    running_ = true;
    reader_ = std::thread(&SerialBridge::reader_loop, this);
  }

  ~SerialBridge() override
  {
    running_ = false;
    if (reader_.joinable()) {
      reader_.join();
    }
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

private:
  int open_serial(const std::string & port, int baud)
  {
    const speed_t speed = baud_to_speed(baud);
    if (speed == 0) {
      RCLCPP_ERROR(get_logger(), "不支持的波特率 %d（可用 9600/57600/115200）", baud);
      return -1;
    }
    const int fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
      return -1;
    }
    termios tio{};
    if (::tcgetattr(fd, &tio) != 0) {
      ::close(fd);
      return -1;
    }
    ::cfmakeraw(&tio);
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CRTSCTS;
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;
    ::cfsetispeed(&tio, speed);
    ::cfsetospeed(&tio, speed);
    if (::tcsetattr(fd, TCSANOW, &tio) != 0) {
      ::close(fd);
      return -1;
    }
    ::tcflush(fd, TCIOFLUSH);
    return fd;
  }

  void reader_loop()
  {
    arm_serial::FrameParser parser;
    arm_serial::Frame frame;
    uint8_t buffer[256];
    uint32_t last_crc_errors = 0;

    while (running_ && rclcpp::ok()) {
      pollfd pfd{};
      pfd.fd = fd_;
      pfd.events = POLLIN;
      const int rc = ::poll(&pfd, 1, 100);
      if (rc < 0) {
        if (errno == EINTR) {
          continue;
        }
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
          "poll 失败：%s", std::strerror(errno));
        break;
      }
      if (rc == 0) {
        continue;
      }
      if (pfd.revents & POLLNVAL) {
        RCLCPP_ERROR(get_logger(), "串口 fd 已失效");
        break;
      }
      if (!(pfd.revents & POLLIN)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        continue;
      }
      const ssize_t n = ::read(fd_, buffer, sizeof(buffer));
      if (n <= 0) {
        continue;
      }
      for (ssize_t i = 0; i < n; ++i) {
        if (parser.push(buffer[i], frame)) {
          handle_frame(frame, parser);
        }
      }
      if (parser.crc_errors() != last_crc_errors) {
        last_crc_errors = parser.crc_errors();
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "CRC 校验失败累计 %u 帧", last_crc_errors);
      }
    }
  }

  void handle_frame(const arm_serial::Frame & frame,
    const arm_serial::FrameParser & parser)
  {
    if (frame.cmd == arm_serial::cmd::kStatus) {
      if (frame.payload.size() != kStatusPayloadSize) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "状态帧长度 %zu != %zu，已丢弃",
          frame.payload.size(), kStatusPayloadSize);
        return;
      }
      arm_interfaces::msg::McuStatus status;
      status.seq = frame.seq;
      status.uptime_ms = read_u32(frame.payload, 0);
      status.temperature_c = read_f32(frame.payload, 4);
      status.error_code = read_u16(frame.payload, 8);
      for (size_t i = 0; i < 6; ++i) {
        status.encoders[i] = read_f32(frame.payload, 10 + 4 * i);
      }
      status.mcu_crc_errors = read_u32(frame.payload, 34);
      status.bridge_crc_errors = parser.crc_errors();
      status_pub_->publish(status);

      sensor_msgs::msg::JointState joints;
      joints.header.stamp = now();
      joints.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
      joints.position.assign(status.encoders.begin(), status.encoders.end());
      joints_pub_->publish(joints);
      return;
    }

    if (frame.cmd == arm_serial::cmd::kAck && frame.payload.size() >= 3) {
      std::lock_guard<std::mutex> lock(ack_mutex_);
      ack_cmd_ = frame.payload[0];
      ack_seq_ = frame.payload[1];
      ack_result_ = frame.payload[2];
      ack_valid_ = true;
      ack_cv_.notify_all();
      return;
    }

    if (frame.cmd == arm_serial::cmd::kError && frame.payload.size() >= 2) {
      std::lock_guard<std::mutex> lock(ack_mutex_);
      ack_cmd_ = arm_serial::cmd::kError;
      ack_seq_ = frame.payload[0];
      ack_result_ = frame.payload[1];
      ack_valid_ = true;
      ack_cv_.notify_all();
    }
  }

  bool write_all(const std::vector<uint8_t> & bytes)
  {
    size_t offset = 0;
    while (offset < bytes.size()) {
      const ssize_t n = ::write(fd_, bytes.data() + offset, bytes.size() - offset);
      if (n > 0) {
        offset += static_cast<size_t>(n);
        continue;
      }
      if (n < 0 && errno == EINTR) {
        continue;
      }
      if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        pollfd pfd{};
        pfd.fd = fd_;
        pfd.events = POLLOUT;
        if (::poll(&pfd, 1, 200) <= 0) {
          return false;
        }
        continue;
      }
      return false;
    }
    return true;
  }

  bool transact(uint8_t command, const std::vector<uint8_t> & payload,
    std::string & message, uint8_t & seq_out)
  {
    if (fd_ < 0) {
      message = "串口未打开";
      return false;
    }
    std::lock_guard<std::mutex> tx_lock(transact_mutex_);
    for (int attempt = 1; attempt <= retries_; ++attempt) {
      const uint8_t seq = next_seq_++;
      seq_out = seq;
      std::vector<uint8_t> bytes;
      try {
        bytes = arm_serial::encode(command, seq, payload);
      } catch (const std::invalid_argument & error) {
        message = error.what();
        return false;
      }
      if (!write_all(bytes)) {
        message = "串口写失败";
        continue;
      }
      std::unique_lock<std::mutex> lock(ack_mutex_);
      const bool got = ack_cv_.wait_for(
        lock, std::chrono::milliseconds(ack_timeout_ms_), [this, command, seq]() {
          return ack_valid_ && ack_seq_ == seq &&
                 (ack_cmd_ == command || ack_cmd_ == arm_serial::cmd::kError);
        });
      if (!got) {
        message = "等待 ACK 超时（第 " + std::to_string(attempt) + "/" +
                  std::to_string(retries_) + " 次）";
        continue;
      }
      const bool is_error = ack_cmd_ == arm_serial::cmd::kError;
      const uint8_t result = ack_result_;
      ack_valid_ = false;
      lock.unlock();
      if (is_error) {
        message = "MCU 返回错误码 " + std::to_string(result);
        return false;
      }
      if (result != 0) {
        message = "MCU 拒绝请求，result=" + std::to_string(result);
        return false;
      }
      message = "ok";
      return true;
    }
    return false;
  }

  void handle_send_joints(
    const arm_interfaces::srv::SendJoints::Request::SharedPtr request,
    arm_interfaces::srv::SendJoints::Response::SharedPtr response)
  {
    std::vector<uint8_t> payload;
    payload.reserve(24);
    for (double position : request->positions) {
      append_f32(payload, static_cast<float>(position));
    }
    std::string message;
    uint8_t seq = 0;
    const bool ok = transact(arm_serial::cmd::kSetJoints, payload, message, seq);
    response->success = ok;
    response->message = message;
    response->seq = seq;
    if (ok) {
      RCLCPP_INFO(get_logger(), "SET_JOINTS 已确认，seq=%u", seq);
    } else {
      RCLCPP_WARN(get_logger(), "SET_JOINTS 失败：%s", message.c_str());
    }
  }

  void handle_ping(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    std::string message;
    uint8_t seq = 0;
    const bool ok = transact(arm_serial::cmd::kPing, {}, message, seq);
    response->success = ok;
    response->message = ok ? "pong, seq=" + std::to_string(seq) : message;
  }

  std::string port_;
  int fd_{-1};
  int ack_timeout_ms_{500};
  int retries_{3};
  std::atomic<bool> running_{false};
  std::thread reader_;

  std::mutex transact_mutex_;
  uint8_t next_seq_{1};

  std::mutex ack_mutex_;
  std::condition_variable ack_cv_;
  bool ack_valid_{false};
  uint8_t ack_cmd_{0};
  uint8_t ack_seq_{0};
  uint8_t ack_result_{0};

  rclcpp::Publisher<arm_interfaces::msg::McuStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joints_pub_;
  rclcpp::Service<arm_interfaces::srv::SendJoints>::SharedPtr send_joints_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr ping_srv_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SerialBridge>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
