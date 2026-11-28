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
#include <mrpt/core/format.h>
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/io/vector_loadsave.h>
#include <mrpt/ros2bridge/image.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/string_utils.h>

#include <cstring>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

using namespace mrpt::img;

std::optional<mrpt::img::BayerPattern> mrpt::ros2bridge::bayerPatternFromROS(
    const std::string& encoding)
{
  namespace enc = sensor_msgs::image_encodings;
  if (encoding == enc::BAYER_RGGB8)
  {
    return BayerPattern::RGGB;
  }
  if (encoding == enc::BAYER_BGGR8)
  {
    return BayerPattern::BGGR;
  }
  if (encoding == enc::BAYER_GBRG8)
  {
    return BayerPattern::GBRG;
  }
  if (encoding == enc::BAYER_GRBG8)
  {
    return BayerPattern::GRBG;
  }
  return std::nullopt;
}

mrpt::img::CImage mrpt::ros2bridge::fromROS(const sensor_msgs::msg::Image& msg)
{
  namespace enc = sensor_msgs::image_encodings;

  // Raw 8-bit Bayer mosaics (e.g. machine-vision cameras), demosaiced to RGB:
  if (const auto pattern = bayerPatternFromROS(msg.encoding); pattern)
  {
    ASSERT_GE_(msg.data.size(), static_cast<size_t>(msg.step) * msg.height);
    mrpt::img::CImage img;
    img.loadFromBayerBuffer(
        static_cast<int32_t>(msg.width), static_cast<int32_t>(msg.height), msg.data.data(),
        msg.step, *pattern);
    return img;
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

  // Row pointers, not per-pixel accessors: this runs on every camera frame.
  const size_t dstRowBytes = static_cast<size_t>(w) * (isColor ? 3 : 1);
  const bool sameLayout = !is16bit && !swapRedBlue && srcBytesPerPixel == (isColor ? 3 : 1);
  for (int32_t row = 0; row < h; row++)
  {
    const uint8_t* src = msg.data.data() + static_cast<size_t>(row) * msg.step;
    auto* dst = img.ptrLine<uint8_t>(row);
    if (sameLayout)
    {
      std::memcpy(dst, src, dstRowBytes);
      continue;
    }
    for (int32_t col = 0; col < w; col++, src += srcBytesPerPixel)
    {
      if (isColor)
      {
        dst[0] = swapRedBlue ? src[2] : src[0];
        dst[1] = src[1];
        dst[2] = swapRedBlue ? src[0] : src[2];
        dst += 3;
      }
      else
      {
        // 16 bit: narrowed down to the 8 most significant bits
        *dst++ = is16bit ? (msg.is_bigendian ? src[0] : src[1]) : src[0];
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

  for (int32_t row = 0; row < h; row++)
  {
    std::memcpy(
        msg.data.data() + static_cast<size_t>(row) * msg.step, img.ptrLine<uint8_t>(row), msg.step);
  }

  return msg;
}

namespace
{
// Format string of a CompressedImage, from a file name, or empty if the file is
// neither a JPEG nor a PNG.
std::string compressedFormatFromFileName(const std::string& fileName)
{
  const auto ext = mrpt::system::lowerCase(mrpt::system::extractFileExtension(fileName));
  if (ext == "jpg" || ext == "jpeg")
  {
    return "jpeg";
  }
  if (ext == "png")
  {
    return "png";
  }
  return {};
}
}  // namespace

sensor_msgs::msg::CompressedImage mrpt::ros2bridge::toROS_Compressed(
    const mrpt::img::CImage& img, const std_msgs::msg::Header& msg_header, int jpeg_quality)
{
  sensor_msgs::msg::CompressedImage msg;
  msg.header = msg_header;

  if (img.isExternallyStored())
  {
    const std::string file = img.getExternalStorageFileAbsolutePath();
    const std::string format = compressedFormatFromFileName(file);
    if (!format.empty())
    {
      ASSERTMSG_(
          mrpt::io::loadBinaryFile(msg.data, file),
          mrpt::format("Cannot read external image file: '%s'", file.c_str()));
      msg.format = format;
      return msg;
    }
  }

  mrpt::io::CMemoryStream buf;
  img.saveToStreamAsJPEG(buf, jpeg_quality);
  const auto* p = static_cast<const uint8_t*>(buf.getRawBufferData());
  msg.data.assign(p, p + buf.getTotalBytesCount());
  msg.format = "jpeg";
  return msg;
}

mrpt::img::CImage mrpt::ros2bridge::fromROS(const sensor_msgs::msg::CompressedImage& i)
{
  mrpt::img::CImage img;
  if (!img.loadFromEncodedBuffer(i.data.data(), i.data.size()))
  {
    img.clear();
  }
  return img;
}
