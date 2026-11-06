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
  FILE: image.h
  AUTHOR: Raghavender Sahdev <raghavendersahdev@gmail.com>
  ---------------------------------------------------------------*/

#pragma once

#include <mrpt/img/CImage.h>
#include <mrpt/obs/CObservationImage.h>

#include <cstring>  // size_t
#include <optional>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>

namespace mrpt::ros2bridge
{
/** \addtogroup mrpt_ros2bridge_grp
 * @{ */

/** Makes a deep copy of the image data */
/** Converts a ROS image into an 8-bit CImage. Raw 8-bit Bayer mosaics
 *  (bayer_*8) are demosaiced into RGB with CImage::loadFromBayerBuffer(). */
mrpt::img::CImage fromROS(const sensor_msgs::msg::Image& i);

/** The color filter layout of a raw 8-bit Bayer ROS encoding (bayer_rggb8,
 *  ...), or nullopt for any other encoding. */
std::optional<mrpt::img::BayerPattern> bayerPatternFromROS(const std::string& encoding);

/** Makes a deep copy of the image data */
sensor_msgs::msg::Image toROS(const mrpt::img::CImage& i, const std_msgs::msg::Header& msg_header);

/** MRPT->ROS: an image as sensor_msgs/CompressedImage.
 *  Externally stored JPEG or PNG images are copied byte by byte from their file, without
 *  decoding nor re-encoding them. Any other image is encoded as JPEG with the given quality.
 */
sensor_msgs::msg::CompressedImage toROS_Compressed(
    const mrpt::img::CImage& i, const std_msgs::msg::Header& msg_header, int jpeg_quality = 95);

/** ROS->MRPT: decodes a sensor_msgs/CompressedImage (JPEG or PNG).
 *  eturn An empty image on decoding errors.
 */
mrpt::img::CImage fromROS(const sensor_msgs::msg::CompressedImage& i);
/** @} */

}  // namespace mrpt::ros2bridge
