/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2026, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

/**
 * @file   rosbag2_to_mrpt.h
 * @brief  Deserialize rosbag2 serialized messages and convert to MRPT
 *         observations. Requires rosbag2_cpp at build time.
 * @author Jose Luis Blanco Claraco
 * @date   2025
 */
#pragma once

#include <mrpt/obs/CObservation.h>
#include <mrpt/poses/CPose3D.h>

#include <optional>
#include <string>
#include <string_view>
#include <vector>

// Forward declarations
namespace tf2
{
class BufferCore;
}
namespace rosbag2_storage
{
class SerializedBagMessage;
}

namespace mrpt::ros2bridge
{
/** \addtogroup mrpt_ros2bridge_grp
 * @{ */

/// Return type for all rosbag2-to-MRPT converters.
using ObsVector = std::vector<mrpt::obs::CObservation::Ptr>;

// -------------------------------------------------------------------
//  Rosbag2 serialized-message → MRPT observation converters
//
//  Each function deserialises the raw CDR bytes, converts the ROS
//  message to the corresponding MRPT observation, and resolves the
//  sensor pose via tf2 (or via the user-supplied fixed pose).
// -------------------------------------------------------------------

/** PointCloud2 → CObservationPointCloud.
 *  Includes the Livox nanosecond-timestamp fix.
 */
ObsVector rosbag2ToPointCloud2(
    std::string_view sensorLabel,
    const rosbag2_storage::SerializedBagMessage& rosmsg,
    tf2::BufferCore& tfBuffer,
    const std::string& base_link_frame,
    const std::optional<mrpt::poses::CPose3D>& fixedSensorPose);

/** LaserScan → CObservation2DRangeScan. */
ObsVector rosbag2ToLidar2D(
    std::string_view sensorLabel,
    const rosbag2_storage::SerializedBagMessage& rosmsg,
    tf2::BufferCore& tfBuffer,
    const std::string& base_link_frame,
    const std::optional<mrpt::poses::CPose3D>& fixedSensorPose);

/** PointCloud2 (with ring field) → CObservationRotatingScan. */
ObsVector rosbag2ToRotatingScan(
    std::string_view sensorLabel,
    const rosbag2_storage::SerializedBagMessage& rosmsg,
    tf2::BufferCore& tfBuffer,
    const std::string& base_link_frame,
    const std::optional<mrpt::poses::CPose3D>& fixedSensorPose);

/** Imu → CObservationIMU. */
ObsVector rosbag2ToIMU(
    std::string_view sensorLabel,
    const rosbag2_storage::SerializedBagMessage& rosmsg,
    tf2::BufferCore& tfBuffer,
    const std::string& base_link_frame,
    const std::optional<mrpt::poses::CPose3D>& fixedSensorPose);

/** NavSatFix → CObservationGPS. */
ObsVector rosbag2ToGPS(
    std::string_view sensorLabel,
    const rosbag2_storage::SerializedBagMessage& rosmsg,
    tf2::BufferCore& tfBuffer,
    const std::string& base_link_frame,
    const std::optional<mrpt::poses::CPose3D>& fixedSensorPose);

/** Odometry → CObservationOdometry. */
ObsVector rosbag2ToOdometry(
    std::string_view sensorLabel, const rosbag2_storage::SerializedBagMessage& rosmsg);

/** Image → CObservationImage (uses cv_bridge). */
ObsVector rosbag2ToImage(
    std::string_view sensorLabel,
    const rosbag2_storage::SerializedBagMessage& rosmsg,
    tf2::BufferCore& tfBuffer,
    const std::string& base_link_frame,
    const std::optional<mrpt::poses::CPose3D>& fixedSensorPose);

/** Deserialize a /tf or /tf_static message and feed it into a
 *  tf2::BufferCore.
 *  \tparam isStatic  true for /tf_static, false for /tf.
 */
template <bool isStatic>
void rosbag2ToTf(tf2::BufferCore& tfBuffer, const rosbag2_storage::SerializedBagMessage& rosmsg);

/** @} */

}  // namespace mrpt::ros2bridge
