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

#include <mrpt/core/exceptions.h>
#include <mrpt/ros1bridge/image.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <cstring>

using namespace mrpt::img;

mrpt::img::CImage mrpt::ros1bridge::fromROS(const sensor_msgs::Image& msg)
{
  namespace enc = sensor_msgs::image_encodings;

  // CImage holds 8-bit pixels, with color channels in RGB order:
  uint32_t srcBytesPerPixel = 0;
  bool isColor = false;
  bool swapRedBlue = false;
  bool is16bit = false;

  if (msg.encoding == enc::MONO8)
  {
    srcBytesPerPixel = 1;
  }
  else if (msg.encoding == enc::MONO16)
  {
    srcBytesPerPixel = 2;
    is16bit = true;
  }
  else if (msg.encoding == enc::RGB8)
  {
    srcBytesPerPixel = 3;
    isColor = true;
  }
  else if (msg.encoding == enc::BGR8)
  {
    srcBytesPerPixel = 3;
    isColor = true;
    swapRedBlue = true;
  }
  else if (msg.encoding == enc::RGBA8)
  {
    srcBytesPerPixel = 4;
    isColor = true;
  }
  else if (msg.encoding == enc::BGRA8)
  {
    srcBytesPerPixel = 4;
    isColor = true;
    swapRedBlue = true;
  }
  else
  {
    THROW_EXCEPTION_FMT("Unsupported ROS image encoding: '%s'", msg.encoding.c_str());
  }

  ASSERT_GE_(msg.step, msg.width * srcBytesPerPixel);
  ASSERT_GE_(msg.data.size(), static_cast<size_t>(msg.step) * msg.height);

  const int32_t w = static_cast<int32_t>(msg.width);
  const int32_t h = static_cast<int32_t>(msg.height);

  mrpt::img::CImage img(w, h, isColor ? CH_RGB : CH_GRAY);

  for (int32_t row = 0; row < h; row++)
  {
    const uint8_t* srcRow = msg.data.data() + static_cast<size_t>(row) * msg.step;
    for (int32_t col = 0; col < w; col++)
    {
      const uint8_t* px = srcRow + static_cast<size_t>(col) * srcBytesPerPixel;
      if (isColor)
      {
        img.at<uint8_t>(col, row, 0) = swapRedBlue ? px[2] : px[0];
        img.at<uint8_t>(col, row, 1) = px[1];
        img.at<uint8_t>(col, row, 2) = swapRedBlue ? px[0] : px[2];
      }
      else if (is16bit)
      {
        // Narrowed down to the 8 most significant bits:
        img.at<uint8_t>(col, row, 0) = msg.is_bigendian ? px[0] : px[1];
      }
      else
      {
        img.at<uint8_t>(col, row, 0) = px[0];
      }
    }
  }

  return img;
}

sensor_msgs::Image mrpt::ros1bridge::toROS(
    const mrpt::img::CImage& img, const std_msgs::Header& msg_header)
{
  sensor_msgs::Image msg;
  msg.header = msg_header;

  const int32_t w = static_cast<int32_t>(img.getWidth());
  const int32_t h = static_cast<int32_t>(img.getHeight());
  const bool isColor = img.isColor();
  const int32_t nCh = isColor ? 3 : 1;

  msg.height = static_cast<uint32_t>(h);
  msg.width = static_cast<uint32_t>(w);
  // CImage keeps color channels in RGB order, so no channel swap is needed here:
  msg.encoding = isColor ? sensor_msgs::image_encodings::RGB8 : sensor_msgs::image_encodings::MONO8;
  msg.is_bigendian = 0;
  msg.step = static_cast<uint32_t>(w * nCh);
  msg.data.resize(static_cast<size_t>(h) * msg.step);

  for (int32_t row = 0; row < h; row++)
  {
    uint8_t* dstRow = msg.data.data() + static_cast<size_t>(row) * msg.step;
    for (int32_t col = 0; col < w; col++)
    {
      if (isColor)
      {
        dstRow[col * 3 + 0] = img.at<uint8_t>(col, row, 0);
        dstRow[col * 3 + 1] = img.at<uint8_t>(col, row, 1);
        dstRow[col * 3 + 2] = img.at<uint8_t>(col, row, 2);
      }
      else
      {
        dstRow[col] = img.at<uint8_t>(col, row, 0);
      }
    }
  }

  return msg;
}
