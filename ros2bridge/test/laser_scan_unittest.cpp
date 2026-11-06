/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2026, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/ros2bridge/laser_scan.h>

#include <cmath>
#include <limits>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <vector>

namespace
{
// Every MRPT ray must be the ROS ray with the same index and direction.
// If angle_increment is zero, it is derived from angle_min, angle_max and N.
void check_scan(float angle_min, float angle_max, size_t N, float angle_increment = 0)
{
  sensor_msgs::msg::LaserScan msg;
  msg.angle_min = angle_min;
  msg.angle_max = angle_max;
  msg.angle_increment =
      angle_increment != 0 ? angle_increment : (angle_max - angle_min) / static_cast<float>(N - 1);
  msg.range_min = 0.01f;
  msg.range_max = 100.0f;
  msg.ranges.resize(N);
  for (size_t i = 0; i < N; i++)
  {
    msg.ranges[i] = 1.0f + 0.01f * static_cast<float>(i);
  }

  const mrpt::poses::CPose3D sensorPose(0.1, 0.2, 0.3, 0.4, 0, 0);
  mrpt::obs::CObservation2DRangeScan obs;
  ASSERT_TRUE(mrpt::ros2bridge::fromROS(msg, sensorPose, obs));
  ASSERT_EQ(obs.getScanSize(), N);

  for (size_t i = 0; i < N; i++)
  {
    EXPECT_EQ(obs.getScanRange(i), msg.ranges[i]) << "ray " << i;

    const double angRos = angle_min + static_cast<double>(i) * msg.angle_increment;
    const double r = msg.ranges[i];
    const auto ptRos = sensorPose.composePoint({r * std::cos(angRos), r * std::sin(angRos), 0});
    const double angMrpt = obs.getScanAngle(i);
    const auto ptMrpt =
        obs.sensorPose.composePoint({r * std::cos(angMrpt), r * std::sin(angMrpt), 0});
    EXPECT_NEAR((ptRos - ptMrpt).norm(), 0, 1e-4) << "ray " << i;
  }
}
}  // namespace

TEST(LaserScanConversions, fromROS_centered_scans)
{
  check_scan(-2.35619449f, 2.35619449f, 1081);
  check_scan(-1.5707964f, 1.5707964f, 181);
  check_scan(-2.0943951f, 2.0943951f, 682);
}

TEST(LaserScanConversions, fromROS_off_center_scan)
{
  check_scan(0.0f, 3.14159265f, 181);
  check_scan(-1.0f, 2.0f, 301);
}

// angle_max may only be an upper bound of the last ray angle.
TEST(LaserScanConversions, fromROS_explicit_angle_increment)
{
  check_scan(-1.5f, 1.5f, 100, 0.03f);
  check_scan(-3.14159265f, 3.14159265f, 360, 0.0174532925f);
}

TEST(LaserScanConversions, fromROS_invalid_ranges)
{
  sensor_msgs::msg::LaserScan msg;
  msg.angle_min = -1.0f;
  msg.angle_max = 1.0f;
  msg.angle_increment = 0.4f;
  msg.range_min = 0.1f;
  msg.range_max = 10.0f;
  msg.ranges = {1.0f,
                std::numeric_limits<float>::quiet_NaN(),
                std::numeric_limits<float>::infinity(),
                0.05f,
                20.0f,
                5.0f};
  const std::vector<bool> expectedValid = {true, false, false, false, false, true};

  mrpt::obs::CObservation2DRangeScan obs;
  ASSERT_TRUE(mrpt::ros2bridge::fromROS(msg, mrpt::poses::CPose3D(), obs));
  ASSERT_EQ(obs.getScanSize(), msg.ranges.size());
  for (size_t i = 0; i < msg.ranges.size(); i++)
  {
    EXPECT_EQ(obs.getScanRangeValidity(i), expectedValid[i]) << "ray " << i;
  }
}

TEST(LaserScanConversions, toROS_validity_direction_intensity)
{
  mrpt::obs::CObservation2DRangeScan obs;
  obs.aperture = static_cast<float>(M_PI);
  obs.maxRange = 30.0f;
  obs.resizeScan(4);
  obs.setScanHasIntensity(true);
  for (size_t i = 0; i < 4; i++)
  {
    obs.setScanRange(i, 1.0f + static_cast<float>(i));
    obs.setScanRangeValidity(i, i != 1);
    obs.setScanIntensity(i, static_cast<int>(10 * i));
  }

  sensor_msgs::msg::LaserScan msg;
  ASSERT_TRUE(mrpt::ros2bridge::toROS(obs, msg));
  ASSERT_EQ(msg.ranges.size(), 4u);
  ASSERT_EQ(msg.intensities.size(), 4u);
  EXPECT_EQ(msg.ranges[0], 1.0f);
  EXPECT_TRUE(std::isinf(msg.ranges[1]));
  EXPECT_EQ(msg.intensities[3], 30.0f);

  // Clockwise MRPT scans are reversed, so ROS rays still go counterclockwise:
  obs.rightToLeft = false;
  ASSERT_TRUE(mrpt::ros2bridge::toROS(obs, msg));
  EXPECT_EQ(msg.ranges[0], 4.0f);
  EXPECT_TRUE(std::isinf(msg.ranges[2]));
  EXPECT_EQ(msg.intensities[0], 30.0f);
}
