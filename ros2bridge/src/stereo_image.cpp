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

#include <cstring>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace
{
/** stereo_msgs/DisparityImage mandates a 32FC1 image, so the MRPT 8-bit
 *  disparity image cannot be forwarded through the generic CImage converter.
 *  Note that MRPT does not define the scale of its disparity values.
 */
sensor_msgs::msg::Image disparityToROS(
    const mrpt::img::CImage& img, const std_msgs::msg::Header& msg_header)
{
  sensor_msgs::msg::Image msg;
  msg.header = msg_header;

  const int32_t w = static_cast<int32_t>(img.getWidth());
  const int32_t h = static_cast<int32_t>(img.getHeight());

  msg.height = static_cast<uint32_t>(h);
  msg.width = static_cast<uint32_t>(w);
  msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  msg.is_bigendian = 0;
  msg.step = static_cast<uint32_t>(w * sizeof(float));
  msg.data.resize(static_cast<size_t>(h) * msg.step);

  for (int32_t row = 0; row < h; row++)
  {
    uint8_t* dstRow = msg.data.data() + static_cast<size_t>(row) * msg.step;
    for (int32_t col = 0; col < w; col++)
    {
      const float d = static_cast<float>(img.at<uint8_t>(col, row, 0));
      std::memcpy(dstRow + col * sizeof(float), &d, sizeof(float));
    }
  }

  return msg;
}
}  // namespace

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
    disparity.image = disparityToROS(obj.imageDisparity, msg_header);
  }

  return true;
}
