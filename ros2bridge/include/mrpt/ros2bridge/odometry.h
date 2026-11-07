/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2026, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/obs/CObservationOdometry.h>

#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/header.hpp>
#include <string>

namespace mrpt::ros2bridge
{
/** \addtogroup mrpt_ros2bridge_grp
 * @{ */

/** MRPT->ROS: a 2D odometry reading to nav_msgs/Odometry.
 *  The caller fills the header (stamp, odometry frame) and the child (robot) frame.
 *  Covariances are left as zeros (unknown), since CObservationOdometry has none.
 *  The opposite conversion is odometryToObservation().
 */
nav_msgs::msg::Odometry toROS(
    const mrpt::obs::CObservationOdometry& obj,
    const std_msgs::msg::Header& msg_header,
    const std::string& child_frame_id);

/** @} */

}  // namespace mrpt::ros2bridge
