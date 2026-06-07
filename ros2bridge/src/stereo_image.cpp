/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2026, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

/*---------------------------------------------------------------
  APPLICATION: mrpt_ros bridge
  FILE: stereo_image.cpp
  AUTHOR: Raghavender Sahdev <raghavendersahdev@gmail.com>
  ---------------------------------------------------------------*/

#include <mrpt/ros2bridge/image.h>
#include <mrpt/ros2bridge/stereo_image.h>

#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

bool mrpt::ros2bridge::toROS(
    const mrpt::obs::CObservationStereoImages& obj,
    const std_msgs::msg::Header& msg_header,
    sensor_msgs::msg::Image& left,
    sensor_msgs::msg::Image& right,
    stereo_msgs::msg::DisparityImage& disparity)
{
  left = mrpt::ros2bridge::toROS(obj.imageLeft, msg_header);
  right = mrpt::ros2bridge::toROS(obj.imageRight, msg_header);

  if (obj.hasImageDisparity)
  {
    disparity.image = mrpt::ros2bridge::toROS(obj.imageDisparity, msg_header);
  }

  return true;
}
