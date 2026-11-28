/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2026, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

/*
 * test_time.cpp
 *
 *  Created on: July 15, 2014
 *      Author: Markus Bader
 */

#include <gtest/gtest.h>
#include <mrpt/core/Clock.h>
#include <mrpt/ros2bridge/time.h>

#include <cstdint>

TEST(Time, basicTest)
{
  const auto org_time = mrpt::Clock::now();

  rclcpp::Time ros_tim = mrpt::ros2bridge::toROS(org_time);
  mrpt::system::TTimeStamp mrpt_tim = mrpt::ros2bridge::fromROS(ros_tim);

  EXPECT_NEAR(mrpt::system::timeDifference(org_time, mrpt_tim), .0, 1e-6);
}

// Conversions keep the full resolution of mrpt::Clock (100 ns), with no floating-point
// rounding at current dates.
TEST(Time, exactRoundTrip)
{
  const auto t0 = mrpt::Clock::fromDouble(1305031453.0);
  for (int64_t k = 0; k < 20000; k += 7)
  {
    const auto t = t0 + mrpt::Clock::duration(k);
    const rclcpp::Time r = mrpt::ros2bridge::toROS(t);
    EXPECT_EQ(r.nanoseconds() % 100, 0);
    EXPECT_EQ(mrpt::ros2bridge::fromROS(r), t);
  }
  // ROS nanoseconds round to the nearest 100 ns tick:
  const int64_t ns = INT64_C(1305031453366898049);
  EXPECT_EQ(
      mrpt::ros2bridge::toROS(mrpt::ros2bridge::fromROS(rclcpp::Time(ns))).nanoseconds(),
      INT64_C(1305031453366898000));
  EXPECT_EQ(
      mrpt::ros2bridge::toROS(mrpt::ros2bridge::fromROS(rclcpp::Time(ns + 2))).nanoseconds(),
      INT64_C(1305031453366898100));
  // Same instant as the floating-point conversion, within its resolution:
  EXPECT_NEAR(
      mrpt::Clock::toDouble(mrpt::ros2bridge::fromROS(rclcpp::Time(ns))), 1305031453.366898, 1e-6);
  // Invalid (default) time points map to zero:
  EXPECT_EQ(mrpt::ros2bridge::toROS(mrpt::Clock::time_point()).nanoseconds(), 0);
}
