/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2026, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/ros2bridge/laser_scan.h>
#include <mrpt/ros2bridge/pose.h>
#include <mrpt/ros2bridge/time.h>
#include <mrpt/version.h>

#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

bool mrpt::ros2bridge::fromROS(
    const sensor_msgs::msg::LaserScan& msg,
    const mrpt::poses::CPose3D& pose,
    mrpt::obs::CObservation2DRangeScan& obj)
{
  obj.timestamp = fromROS(msg.header.stamp);
  obj.rightToLeft = true;
  obj.sensorLabel = msg.header.frame_id;
  obj.aperture = msg.angle_max - msg.angle_min;
  obj.maxRange = msg.range_max;
  obj.sensorPose = pose;

#if MRPT_VERSION >= 0x020f06
  obj.sweepDuration = msg.scan_time;
#endif

  ASSERT_GT_(msg.ranges.size(), 2);

  const size_t N = msg.ranges.size();
  const auto N_f = static_cast<float>(N);
  const float ang_step = obj.aperture / (N_f - 1);
  const float fov05 = 0.5f * obj.aperture;
  const float inv_ang_step = (N_f - 1) / obj.aperture;

  obj.resizeScan(N);
  for (std::size_t i_mrpt = 0; i_mrpt < N; i_mrpt++)
  {
    // ROS indices go from msg.angle_min to msg.angle_max, while
    // in MRPT they go from -FOV/2 to +FOV/2.
    auto i_ros = static_cast<int>(
        inv_ang_step * (-fov05 - msg.angle_min + ang_step * static_cast<float>(i_mrpt)));
    if (i_ros < 0)
    {
      i_ros += static_cast<int>(N);
    }
    else if (i_ros >= static_cast<int>(N))
    {
      i_ros -= static_cast<int>(N);  // wrap around 2PI...
    }

    // set the scan
    const float r = msg.ranges[i_ros];
    obj.setScanRange(i_mrpt, r);

    // set the validity of the scan
    const auto ri = obj.getScanRange(i_mrpt);
    const bool r_valid = ((ri < (msg.range_max * 0.99)) && (ri > msg.range_min));
    obj.setScanRangeValidity(i_mrpt, r_valid);
  }

  return true;
}

bool mrpt::ros2bridge::toROS(
    const mrpt::obs::CObservation2DRangeScan& obj, sensor_msgs::msg::LaserScan& msg)
{
  const size_t nRays = obj.getScanSize();
  const auto nRays_f = static_cast<float>(nRays);
  if (!nRays)
  {
    return false;
  }

  msg.angle_min = -0.5f * obj.aperture;
  msg.angle_max = 0.5f * obj.aperture;
  msg.angle_increment = obj.aperture / (nRays_f - 1.0f);

#if MRPT_VERSION >= 0x020f06
  msg.time_increment = obj.sweepDuration / nRays_f;
  msg.scan_time = obj.sweepDuration;
#else
  msg.time_increment = 0.0;
  msg.scan_time = 0.0;
#endif

  msg.range_min = 0.02f;
  msg.range_max = obj.maxRange;

  msg.ranges.resize(nRays);
  for (size_t i = 0; i < nRays; i++)
  {
    msg.ranges[i] = obj.getScanRange(i);
  }

  // Set header data:
  msg.header.stamp = toROS(obj.timestamp);
  msg.header.frame_id = obj.sensorLabel;

  return true;
}

bool mrpt::ros2bridge::toROS(
    const mrpt::obs::CObservation2DRangeScan& obj,
    sensor_msgs::msg::LaserScan& msg,
    geometry_msgs::msg::Pose& pose)
{
  toROS(obj, msg);
  pose = toROS_Pose(obj.sensorPose);
  return true;
}
