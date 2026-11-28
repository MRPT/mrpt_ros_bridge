/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2026, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/core/Clock.h>
#include <mrpt/ros2bridge/time.h>

#include <cstdint>

// Integer arithmetic: going through floating-point seconds since 1970 would lose
// the 100 ns resolution of mrpt::Clock (doubles resolve ~0.2 us at current dates).
namespace
{
// Offset between the 1601-01-01 epoch of mrpt::Clock and the 1970-01-01 UNIX epoch,
// in 100-nanosecond ticks.
constexpr int64_t UNIX_EPOCH_OFFSET_TICKS = INT64_C(116444736) * INT64_C(1000000000);
constexpr int64_t NS_PER_TICK = 100;

// Integer division rounding to the nearest integer, also for negative numbers.
int64_t roundedDivision(int64_t num, int64_t den)
{
  return num >= 0 ? (num + den / 2) / den : -((-num + den / 2) / den);
}
}  // namespace

mrpt::system::TTimeStamp mrpt::ros2bridge::fromROS(const rclcpp::Time& src)
{
  return mrpt::Clock::time_point(mrpt::Clock::duration(
      UNIX_EPOCH_OFFSET_TICKS + roundedDivision(src.nanoseconds(), NS_PER_TICK)));
}

rclcpp::Time mrpt::ros2bridge::toROS(const mrpt::system::TTimeStamp& src)
{
  if (src == mrpt::Clock::time_point())
  {
    return rclcpp::Time(0, 0);  // invalid time point
  }
  const int64_t ticks = src.time_since_epoch().count() - UNIX_EPOCH_OFFSET_TICKS;
  return rclcpp::Time(ticks * NS_PER_TICK, RCL_SYSTEM_TIME);
}
