#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "arm_serial/frame_codec.hpp"

namespace
{

std::vector<arm_serial::Frame> feed(
  arm_serial::FrameParser & parser, const std::vector<uint8_t> & bytes)
{
  std::vector<arm_serial::Frame> frames;
  arm_serial::Frame frame;
  for (uint8_t byte : bytes) {
    if (parser.push(byte, frame)) {
      frames.push_back(frame);
    }
  }
  return frames;
}

}  // namespace

TEST(FrameCodec, CrcKnownVector)
{
  const uint8_t text[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  EXPECT_EQ(arm_serial::crc16(text, sizeof(text)), 0x29B1);
}

TEST(FrameCodec, RoundTrip)
{
  const std::vector<uint8_t> payload{0x01, 0x02, 0x03, 0xFE};
  const auto bytes = arm_serial::encode(arm_serial::cmd::kSetJoints, 7, payload);
  ASSERT_EQ(bytes.size(), payload.size() + 8);
  EXPECT_EQ(bytes[0], 0xAA);
  EXPECT_EQ(bytes[1], 0x55);
  EXPECT_EQ(bytes[2], arm_serial::kVersion);

  arm_serial::FrameParser parser;
  const auto frames = feed(parser, bytes);
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_EQ(frames[0].cmd, arm_serial::cmd::kSetJoints);
  EXPECT_EQ(frames[0].seq, 7);
  EXPECT_EQ(frames[0].payload, payload);
  EXPECT_EQ(parser.crc_errors(), 0u);
}

TEST(FrameCodec, TwoFramesSticky)
{
  const auto first = arm_serial::encode(arm_serial::cmd::kPing, 1, {});
  const auto second = arm_serial::encode(arm_serial::cmd::kStatus, 2, {9, 8, 7});
  std::vector<uint8_t> stream = first;
  stream.insert(stream.end(), second.begin(), second.end());

  arm_serial::FrameParser parser;
  const auto frames = feed(parser, stream);
  ASSERT_EQ(frames.size(), 2u);
  EXPECT_EQ(frames[0].cmd, arm_serial::cmd::kPing);
  EXPECT_TRUE(frames[0].payload.empty());
  EXPECT_EQ(frames[1].cmd, arm_serial::cmd::kStatus);
  EXPECT_EQ(frames[1].payload, (std::vector<uint8_t>{9, 8, 7}));
}

TEST(FrameCodec, GarbageThenFrame)
{
  const auto valid = arm_serial::encode(arm_serial::cmd::kPing, 3, {});
  std::vector<uint8_t> stream{0x00, 0x12, 0xFF};
  stream.insert(stream.end(), valid.begin(), valid.end());

  arm_serial::FrameParser parser;
  const auto frames = feed(parser, stream);
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_EQ(frames[0].seq, 3);
}

TEST(FrameCodec, OversizeLengthRejected)
{
  arm_serial::FrameParser parser;
  const std::vector<uint8_t> bad{
    0xAA, 0x55, arm_serial::kVersion, arm_serial::cmd::kStatus, 0x01, 0x41};
  EXPECT_TRUE(feed(parser, bad).empty());
  EXPECT_EQ(parser.length_errors(), 1u);

  const auto valid = arm_serial::encode(arm_serial::cmd::kPing, 4, {});
  EXPECT_EQ(feed(parser, valid).size(), 1u);
}

TEST(FrameCodec, BadCrcCountedThenRecovers)
{
  auto corrupted = arm_serial::encode(arm_serial::cmd::kPing, 5, {0x42});
  corrupted.back() ^= 0xFF;

  arm_serial::FrameParser parser;
  EXPECT_TRUE(feed(parser, corrupted).empty());
  EXPECT_EQ(parser.crc_errors(), 1u);

  const auto valid = arm_serial::encode(arm_serial::cmd::kPing, 6, {0x42});
  EXPECT_EQ(feed(parser, valid).size(), 1u);
  EXPECT_EQ(parser.crc_errors(), 1u);
}

TEST(FrameCodec, WrongVersionRejected)
{
  arm_serial::FrameParser parser;
  const std::vector<uint8_t> bad{0xAA, 0x55, 0x02, 0x01, 0x00};
  EXPECT_TRUE(feed(parser, bad).empty());
  EXPECT_EQ(parser.version_errors(), 1u);
}

TEST(FrameCodec, MidFrameReset)
{
  arm_serial::FrameParser parser;
  arm_serial::Frame frame;
  EXPECT_FALSE(parser.push(0xAA, frame));
  EXPECT_TRUE(parser.mid_frame());
  parser.reset();
  EXPECT_FALSE(parser.mid_frame());
}
