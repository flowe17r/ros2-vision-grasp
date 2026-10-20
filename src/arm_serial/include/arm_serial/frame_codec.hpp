#ifndef ARM_SERIAL__FRAME_CODEC_HPP_
#define ARM_SERIAL__FRAME_CODEC_HPP_

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace arm_serial
{

// 帧格式：AA 55 | version | cmd | seq | len | payload[0..64] | crc_lo crc_hi
// CRC16-CCITT-FALSE 覆盖 version..payload，低字节在前。
constexpr uint8_t kSync0 = 0xAA;
constexpr uint8_t kSync1 = 0x55;
constexpr uint8_t kVersion = 0x01;
constexpr size_t kMaxPayload = 64;

namespace cmd
{
constexpr uint8_t kPing = 0x01;
constexpr uint8_t kSetJoints = 0x02;
constexpr uint8_t kGetStatus = 0x03;
constexpr uint8_t kAck = 0x10;
constexpr uint8_t kStatus = 0x11;
constexpr uint8_t kError = 0x7F;
}  // namespace cmd

struct Frame
{
  uint8_t version{0};
  uint8_t cmd{0};
  uint8_t seq{0};
  std::vector<uint8_t> payload;
};

inline uint16_t crc16(const uint8_t * data, size_t length)
{
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                           : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

inline std::vector<uint8_t> encode(uint8_t command, uint8_t seq,
  const std::vector<uint8_t> & payload)
{
  if (payload.size() > kMaxPayload) {
    throw std::invalid_argument("payload exceeds 64 bytes");
  }
  std::vector<uint8_t> out;
  out.reserve(payload.size() + 8);
  out.push_back(kSync0);
  out.push_back(kSync1);
  out.push_back(kVersion);
  out.push_back(command);
  out.push_back(seq);
  out.push_back(static_cast<uint8_t>(payload.size()));
  out.insert(out.end(), payload.begin(), payload.end());
  const uint16_t crc = crc16(out.data() + 2, out.size() - 2);
  out.push_back(static_cast<uint8_t>(crc & 0xFF));
  out.push_back(static_cast<uint8_t>(crc >> 8));
  return out;
}

class FrameParser
{
public:
  // 返回 true 时 out 是一帧通过 CRC 校验的数据。
  bool push(uint8_t byte, Frame & out)
  {
    switch (state_) {
      case State::kWaitSync0:
        if (byte == kSync0) {
          state_ = State::kWaitSync1;
        }
        break;
      case State::kWaitSync1:
        if (byte == kSync1) {
          state_ = State::kVersion;
        } else if (byte != kSync0) {
          state_ = State::kWaitSync0;
        }
        break;
      case State::kVersion:
        if (byte != kVersion) {
          ++version_errors_;
          reset_frame();
          break;
        }
        frame_.version = byte;
        state_ = State::kCmd;
        break;
      case State::kCmd:
        frame_.cmd = byte;
        state_ = State::kSeq;
        break;
      case State::kSeq:
        frame_.seq = byte;
        state_ = State::kLength;
        break;
      case State::kLength:
        if (byte > kMaxPayload) {
          ++length_errors_;
          reset_frame();
          break;
        }
        length_ = byte;
        frame_.payload.clear();
        frame_.payload.reserve(length_);
        state_ = (length_ == 0) ? State::kCrcLo : State::kPayload;
        break;
      case State::kPayload:
        frame_.payload.push_back(byte);
        if (frame_.payload.size() == length_) {
          state_ = State::kCrcLo;
        }
        break;
      case State::kCrcLo:
        crc_lo_ = byte;
        state_ = State::kCrcHi;
        break;
      case State::kCrcHi: {
          uint8_t raw[6 + kMaxPayload];
          raw[0] = frame_.version;
          raw[1] = frame_.cmd;
          raw[2] = frame_.seq;
          raw[3] = length_;
          for (size_t i = 0; i < frame_.payload.size(); ++i) {
            raw[4 + i] = frame_.payload[i];
          }
          const uint16_t expected = crc16(raw, 4 + length_);
          const uint16_t actual =
            static_cast<uint16_t>(crc_lo_ | (static_cast<uint16_t>(byte) << 8));
          if (expected != actual) {
            ++crc_errors_;
            reset_frame();
            break;
          }
          out = frame_;
          reset_frame();
          return true;
        }
    }
    return false;
  }

  void reset()
  {
    reset_frame();
  }

  bool mid_frame() const
  {
    return state_ != State::kWaitSync0;
  }

  uint32_t crc_errors() const {return crc_errors_;}
  uint32_t length_errors() const {return length_errors_;}
  uint32_t version_errors() const {return version_errors_;}

private:
  enum class State
  {
    kWaitSync0,
    kWaitSync1,
    kVersion,
    kCmd,
    kSeq,
    kLength,
    kPayload,
    kCrcLo,
    kCrcHi,
  };

  void reset_frame()
  {
    state_ = State::kWaitSync0;
    frame_ = Frame{};
    length_ = 0;
    crc_lo_ = 0;
  }

  State state_{State::kWaitSync0};
  Frame frame_;
  uint8_t length_{0};
  uint8_t crc_lo_{0};
  uint32_t crc_errors_{0};
  uint32_t length_errors_{0};
  uint32_t version_errors_{0};
};

}  // namespace arm_serial

#endif  // ARM_SERIAL__FRAME_CODEC_HPP_
