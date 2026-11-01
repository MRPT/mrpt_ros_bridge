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
#include <mrpt/ros2bridge/image.h>

#include <cstring>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

using namespace mrpt::img;

namespace
{
/** Bilinear demosaicing of an 8-bit Bayer image into an RGB CImage.
 *  (r_row, r_col): position of the red pixel within each 2x2 tile. */
mrpt::img::CImage demosaicBayer8(const sensor_msgs::msg::Image& msg, int r_row, int r_col)
{
  ASSERT_GE_(msg.width, 2U);
  ASSERT_GE_(msg.height, 2U);
  ASSERT_GE_(msg.step, msg.width);
  ASSERT_GE_(msg.data.size(), static_cast<size_t>(msg.step) * msg.height);

  const int32_t w = static_cast<int32_t>(msg.width);
  const int32_t h = static_cast<int32_t>(msg.height);
  const int b_row = 1 - r_row;
  const int b_col = 1 - r_col;

  // Out-of-bounds neighbors are reflected (-1 -> 1), which keeps them on the
  // same color site of the pattern, unlike clamping (-1 -> 0).
  const auto at = [&](int32_t y, int32_t x) -> int
  {
    y = y < 0 ? -y : (y >= h ? 2 * h - 2 - y : y);
    x = x < 0 ? -x : (x >= w ? 2 * w - 2 - x : x);
    return msg.data[static_cast<size_t>(y) * msg.step + static_cast<size_t>(x)];
  };

  mrpt::img::CImage img(w, h, CH_RGB);
  for (int32_t y = 0; y < h; y++)
  {
    for (int32_t x = 0; x < w; x++)
    {
      const int ty = y % 2;
      const int tx = x % 2;
      const int cross = (at(y - 1, x) + at(y + 1, x) + at(y, x - 1) + at(y, x + 1)) / 4;
      const int diag =
          (at(y - 1, x - 1) + at(y - 1, x + 1) + at(y + 1, x - 1) + at(y + 1, x + 1)) / 4;
      const int horz = (at(y, x - 1) + at(y, x + 1)) / 2;
      const int vert = (at(y - 1, x) + at(y + 1, x)) / 2;

      int r = 0;
      int g = 0;
      int b = 0;
      if (ty == r_row && tx == r_col)
      {
        r = at(y, x);
        g = cross;
        b = diag;
      }
      else if (ty == b_row && tx == b_col)
      {
        r = diag;
        g = cross;
        b = at(y, x);
      }
      else if (ty == r_row)
      {
        // Green on a red row: red to the left/right, blue above/below.
        r = horz;
        g = at(y, x);
        b = vert;
      }
      else
      {
        // Green on a blue row: blue to the left/right, red above/below.
        r = vert;
        g = at(y, x);
        b = horz;
      }
      img.at<uint8_t>(x, y, 0) = static_cast<uint8_t>(r);
      img.at<uint8_t>(x, y, 1) = static_cast<uint8_t>(g);
      img.at<uint8_t>(x, y, 2) = static_cast<uint8_t>(b);
    }
  }
  return img;
}
}  // namespace

mrpt::img::CImage mrpt::ros2bridge::fromROS(const sensor_msgs::msg::Image& msg)
{
  namespace enc = sensor_msgs::image_encodings;

  // Raw 8-bit Bayer mosaics (e.g. machine-vision cameras), demosaiced to RGB:
  if (msg.encoding == enc::BAYER_RGGB8)
  {
    return demosaicBayer8(msg, 0, 0);
  }
  if (msg.encoding == enc::BAYER_BGGR8)
  {
    return demosaicBayer8(msg, 1, 1);
  }
  if (msg.encoding == enc::BAYER_GRBG8)
  {
    return demosaicBayer8(msg, 0, 1);
  }
  if (msg.encoding == enc::BAYER_GBRG8)
  {
    return demosaicBayer8(msg, 1, 0);
  }

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
  // CImage keeps color channels in RGB order, so no channel swap is needed here:
  msg.encoding = isColor ? sensor_msgs::image_encodings::RGB8 : sensor_msgs::image_encodings::MONO8;
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
