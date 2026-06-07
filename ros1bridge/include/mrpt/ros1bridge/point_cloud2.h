/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/maps/CGenericPointsMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/obs/CObservationRotatingScan.h>
#include <sensor_msgs/PointCloud2.h>

#include <set>
#include <string>

namespace mrpt::ros1bridge
{
/** \addtogroup mrpt_ros1bridge_grp
 * @{ */

/** @name sensor_msgs::PointCloud2: ROS <-> MRPT
 *  @{ */

/** Convert sensor_msgs/PointCloud2 -> mrpt::slam::CSimplePointsMap
 *  Only (x,y,z) data is converted. To use extra channels, see CGenericPointsMap overload.
 * Requires point cloud fields: x,y,z.
 * \return true on successful conversion, false on any error.
 */
bool fromROS(const sensor_msgs::PointCloud2& msg, mrpt::maps::CSimplePointsMap& obj);

/** Convert sensor_msgs/PointCloud2 to mrpt::maps::CGenericPointsMap.
 * Auto-detects all fields in the message (x,y,z,intensity,ring,time,...).
 * \return true on successful conversion, false on any error.
 */
bool fromROS(const sensor_msgs::PointCloud2& msg, mrpt::maps::CGenericPointsMap& obj);

/** Convert sensor_msgs/PointCloud2 -> mrpt::obs::CObservationRotatingScan.
 * Requires point cloud fields: x,y,z,ring
 */
bool fromROS(
    const sensor_msgs::PointCloud2& m,
    mrpt::obs::CObservationRotatingScan& o,
    const mrpt::poses::CPose3D& sensorPoseOnRobot,
    unsigned int num_azimuth_divisions = 360,
    float max_intensity = 1000.0f);

/** Extract a list of fields found in the point cloud.
 * Typically: {"x","y","z","intensity"}
 */
std::set<std::string> extractFields(const sensor_msgs::PointCloud2& msg);

/** Convert mrpt::slam::CSimplePointsMap -> sensor_msgs/PointCloud2
 * Generated sensor_msgs::PointCloud2::channels: `x`, `y`, `z`.
 * \return true on successful conversion, false on any error.
 */
bool toROS(
    const mrpt::maps::CSimplePointsMap& obj,
    const std_msgs::Header& msg_header,
    sensor_msgs::PointCloud2& msg);

/** Convert mrpt::maps::CGenericPointsMap -> sensor_msgs/PointCloud2
 * \return true on successful conversion, false on any error.
 */
bool toROS(
    const mrpt::maps::CGenericPointsMap& obj,
    const std_msgs::Header& msg_header,
    sensor_msgs::PointCloud2& msg);

/** @} */
/** @} */

}  // namespace mrpt::ros1bridge
