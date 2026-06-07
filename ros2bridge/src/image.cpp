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
  FILE: image.cpp
  AUTHOR: Raghavender Sahdev <raghavendersahdev@gmail.com>
  ---------------------------------------------------------------*/

#include <mrpt/ros2bridge/image.h>

#include <cstring>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

using namespace mrpt::img;

mrpt::img::CImage mrpt::ros2bridge::fromROS(const sensor_msgs::msg::Image& msg)
{
  const bool isColor = (msg.encoding != sensor_msgs::image_encodings::MONO8);
  const int32_t w = static_cast<int32_t>(msg.width);
  const int32_t h = static_cast<int32_t>(msg.height);
  const TImageChannels nCh = isColor ? CH_RGB : CH_GRAY;

  mrpt::img::CImage img(w, h, nCh);

  const uint32_t step = msg.step;
  const uint8_t* src = msg.data.data();

  for (int32_t row = 0; row < h; row++)
  {
    const uint8_t* srcRow = src + row * step;
    if (isColor)
    {
      for (int32_t col = 0; col < w; col++)
      {
        // ROS image_encodings::BGR8 or RGB8 -> store as-is
        img.at<uint8_t>(col, row, 0) = srcRow[col * 3 + 0];
        img.at<uint8_t>(col, row, 1) = srcRow[col * 3 + 1];
        img.at<uint8_t>(col, row, 2) = srcRow[col * 3 + 2];
      }
    }
    else
    {
      for (int32_t col = 0; col < w; col++)
      {
        img.at<uint8_t>(col, row, 0) = srcRow[col];
      }
    }
  }

  return img;
}

sensor_msgs::msg::Image mrpt::ros2bridge::toROS(
    const mrpt::img::CImage& img, const std_msgs::msg::Header& msg_header)
{
  sensor_msgs::msg::Image msg;
  msg.header = msg_header;

  const int32_t w = img.getWidth();
  const int32_t h = img.getHeight();
  const bool isColor = img.isColor();
  const int32_t nCh = isColor ? 3 : 1;

  msg.height = static_cast<uint32_t>(h);
  msg.width = static_cast<uint32_t>(w);
  msg.encoding = isColor ? sensor_msgs::image_encodings::BGR8 : sensor_msgs::image_encodings::MONO8;
  msg.is_bigendian = 0;
  msg.step = static_cast<uint32_t>(w * nCh);
  msg.data.resize(static_cast<size_t>(h) * msg.step);

  uint8_t* dst = msg.data.data();

  for (int32_t row = 0; row < h; row++)
  {
    uint8_t* dstRow = dst + row * msg.step;
    if (isColor)
    {
      for (int32_t col = 0; col < w; col++)
      {
        dstRow[col * 3 + 0] = img.at<uint8_t>(col, row, 0);
        dstRow[col * 3 + 1] = img.at<uint8_t>(col, row, 1);
        dstRow[col * 3 + 2] = img.at<uint8_t>(col, row, 2);
      }
    }
    else
    {
      for (int32_t col = 0; col < w; col++)
      {
        dstRow[col] = img.at<uint8_t>(col, row, 0);
      }
    }
  }

  return msg;
}
