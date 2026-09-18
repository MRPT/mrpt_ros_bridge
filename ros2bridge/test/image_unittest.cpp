/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2026, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/ros2bridge/image.h>

#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/header.hpp>

namespace
{
sensor_msgs::msg::Image makeImage(
    const std::string& encoding,
    uint32_t width,
    uint32_t height,
    uint32_t bytesPerPixel,
    const std::vector<uint8_t>& data)
{
  sensor_msgs::msg::Image msg;
  msg.encoding = encoding;
  msg.width = width;
  msg.height = height;
  msg.step = width * bytesPerPixel;
  msg.data = data;
  return msg;
}
}  // namespace

// CImage keeps color channels in RGB order, so a bgr8 message must be
// swapped on the way in, and an RGB CImage must be announced as rgb8.
TEST(Image, bgr8RoundTrip)
{
  const auto msg = makeImage(sensor_msgs::image_encodings::BGR8, 2, 1, 3, {10, 20, 30, 40, 50, 60});

  const auto img = mrpt::ros2bridge::fromROS(msg);
  ASSERT_TRUE(img.isColor());
  EXPECT_EQ(img.at<uint8_t>(0, 0, 0), 30);
  EXPECT_EQ(img.at<uint8_t>(0, 0, 1), 20);
  EXPECT_EQ(img.at<uint8_t>(0, 0, 2), 10);
  EXPECT_EQ(img.at<uint8_t>(1, 0, 0), 60);

  const auto out = mrpt::ros2bridge::toROS(img, std_msgs::msg::Header());
  EXPECT_EQ(out.encoding, sensor_msgs::image_encodings::RGB8);
  EXPECT_EQ(out.width, 2U);
  EXPECT_EQ(out.height, 1U);
  EXPECT_EQ(out.step, 6U);
  EXPECT_EQ(out.data[0], 30);
  EXPECT_EQ(out.data[2], 10);
}

TEST(Image, rgb8KeepsChannelOrder)
{
  const auto msg = makeImage(sensor_msgs::image_encodings::RGB8, 1, 1, 3, {10, 20, 30});

  const auto img = mrpt::ros2bridge::fromROS(msg);
  EXPECT_EQ(img.at<uint8_t>(0, 0, 0), 10);
  EXPECT_EQ(img.at<uint8_t>(0, 0, 2), 30);
}

TEST(Image, mono8RoundTrip)
{
  const auto msg = makeImage(sensor_msgs::image_encodings::MONO8, 2, 2, 1, {1, 2, 3, 4});

  const auto img = mrpt::ros2bridge::fromROS(msg);
  ASSERT_FALSE(img.isColor());
  EXPECT_EQ(img.getWidth(), 2U);
  EXPECT_EQ(img.getHeight(), 2U);
  EXPECT_EQ(img.at<uint8_t>(0, 0, 0), 1);
  EXPECT_EQ(img.at<uint8_t>(1, 1, 0), 4);

  const auto out = mrpt::ros2bridge::toROS(img, std_msgs::msg::Header());
  EXPECT_EQ(out.encoding, sensor_msgs::image_encodings::MONO8);
  EXPECT_EQ(out.step, 2U);
  EXPECT_EQ(out.data, std::vector<uint8_t>({1, 2, 3, 4}));
}

// CImage is 8-bit, so mono16 keeps the most significant byte of each pixel:
TEST(Image, mono16Narrowing)
{
  auto msg = makeImage(sensor_msgs::image_encodings::MONO16, 1, 1, 2, {0x34, 0x12});

  msg.is_bigendian = 0;
  EXPECT_EQ(mrpt::ros2bridge::fromROS(msg).at<uint8_t>(0, 0, 0), 0x12);

  msg.is_bigendian = 1;
  EXPECT_EQ(mrpt::ros2bridge::fromROS(msg).at<uint8_t>(0, 0, 0), 0x34);
}

TEST(Image, rgba8DropsAlpha)
{
  const auto msg = makeImage(sensor_msgs::image_encodings::RGBA8, 1, 1, 4, {10, 20, 30, 255});

  const auto img = mrpt::ros2bridge::fromROS(msg);
  ASSERT_TRUE(img.isColor());
  EXPECT_EQ(img.at<uint8_t>(0, 0, 0), 10);
  EXPECT_EQ(img.at<uint8_t>(0, 0, 2), 30);
}

// Unsupported encodings must be rejected instead of being read as if they
// had three bytes per pixel:
TEST(Image, rejectsUnsupportedEncoding)
{
  const auto msg = makeImage("yuv422", 4, 1, 2, std::vector<uint8_t>(8, 0));
  EXPECT_THROW(mrpt::ros2bridge::fromROS(msg), std::exception);
}

TEST(Image, rejectsTruncatedData)
{
  auto msg = makeImage(sensor_msgs::image_encodings::RGB8, 2, 2, 3, std::vector<uint8_t>(6, 0));
  EXPECT_THROW(mrpt::ros2bridge::fromROS(msg), std::exception);
}
