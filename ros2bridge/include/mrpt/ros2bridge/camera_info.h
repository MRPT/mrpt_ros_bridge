/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2026, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/img/TCamera.h>

#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/header.hpp>

namespace mrpt::ros2bridge
{
/** \addtogroup mrpt_ros2bridge_grp
 * @{ */

/** MRPT->ROS: camera intrinsics and distortion to sensor_msgs/CameraInfo.
 *
 * Distortion models: `none` -> "plumb_bob" with zero coefficients, `plumb_bob` ->
 * "plumb_bob" (5 coefficients) or "rational_polynomial" (8, if k4..k6 are used),
 * `kannala_brandt` -> "equidistant". The rectification matrix is the identity and the
 * projection matrix is K with a zero fourth column (monocular camera).
 */
sensor_msgs::msg::CameraInfo toROS(
    const mrpt::img::TCamera& cam, const std_msgs::msg::Header& msg_header);

/** ROS->MRPT: sensor_msgs/CameraInfo to camera intrinsics and distortion.
 *  Unknown distortion models with non-zero coefficients are returned as `none`.
 *  \sa toROS
 */
mrpt::img::TCamera fromROS(const sensor_msgs::msg::CameraInfo& info);

/** @} */

}  // namespace mrpt::ros2bridge
