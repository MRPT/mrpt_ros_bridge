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
#include <sensor_msgs/msg/laser_scan.hpp>

namespace
{
// Every MRPT ray must be the ROS ray with the same index and direction.
void check_scan(float angle_min, float angle_max, size_t N)
{
  sensor_msgs::msg::LaserScan msg;
  msg.angle_min = angle_min;
  msg.angle_max = angle_max;
  msg.angle_increment = (angle_max - angle_min) / static_cast<float>(N - 1);
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
