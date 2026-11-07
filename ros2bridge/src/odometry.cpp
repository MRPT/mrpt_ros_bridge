/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2026, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/ros2bridge/odometry.h>
#include <mrpt/ros2bridge/pose.h>

nav_msgs::msg::Odometry mrpt::ros2bridge::toROS(
    const mrpt::obs::CObservationOdometry& obj,
    const std_msgs::msg::Header& msg_header,
    const std::string& child_frame_id)
{
  nav_msgs::msg::Odometry msg;
  msg.header = msg_header;
  msg.child_frame_id = child_frame_id;
  msg.pose.pose = toROS_Pose(obj.odometry);

  if (obj.hasVelocities)
  {
    msg.twist.twist.linear.x = obj.velocityLocal.vx;
    msg.twist.twist.linear.y = obj.velocityLocal.vy;
    msg.twist.twist.angular.z = obj.velocityLocal.omega;
  }
  return msg;
}
