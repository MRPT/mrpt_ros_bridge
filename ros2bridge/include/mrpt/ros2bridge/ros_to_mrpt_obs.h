/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2026, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

/**
 * @file   ros_to_mrpt_obs.h
 * @brief  Convert already-deserialized ROS 2 messages to MRPT observations,
 *         plus shared helpers (tf2 lookups, Livox timestamp fix).
 * @author Jose Luis Blanco Claraco
 * @date   2025
 */
#pragma once

#include <mrpt/maps/CPointsMap.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/system/COutputLogger.h>

#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>

// Forward declarations
namespace tf2
{
class BufferCore;
}

namespace mrpt::ros2bridge
{
/** \addtogroup mrpt_ros2bridge_grp
 * @{ */

// -------------------------------------------------------------------
//  tf2 sensor-pose lookup  (shared by BridgeROS2, Rosbag2Dataset,
//  and rosbag2rawlog)
// -------------------------------------------------------------------

/** Look up a transform in a tf2::BufferCore and store the result as a
 *  mrpt::poses::CPose3D.
 *
 *  If \a fixedSensorPose is set, it is returned directly and tf2 is not
 *  queried.
 *
 * \param[out] des              The resulting pose.
 * \param[in]  tfBuffer         The tf2 buffer to query.
 * \param[in]  target_frame     Frame of the sensor (child frame).
 * \param[in]  reference_frame  Reference frame (e.g. base_link).
 * \param[in]  fixedSensorPose  If provided, returned as-is (bypass tf2).
 *
 * \return true on success, false if the transform could not be resolved.
 */
bool lookupSensorPose(
    mrpt::poses::CPose3D& des,
    tf2::BufferCore& tfBuffer,
    const std::string& target_frame,
    const std::string& reference_frame,
    const std::optional<mrpt::poses::CPose3D>& fixedSensorPose = std::nullopt,
    mrpt::system::COutputLogger* logger = nullptr);

// -------------------------------------------------------------------
//  Livox timestamp fix  (shared by BridgeROS2 and Rosbag2Dataset)
// -------------------------------------------------------------------

/** If the per-point timestamps in a point map look like nanoseconds
 *  (span > 1e5), convert them to seconds in-place.
 *
 *  This handles a quirk of the Livox ROS driver that publishes
 *  double-typed timestamps that are actually in nanoseconds.
 */
void fixLivoxTimestampsIfNeeded(mrpt::maps::CPointsMap& pts);

// -------------------------------------------------------------------
//  Deserialized-ROS-message  →  MRPT observation helpers
// -------------------------------------------------------------------

/** Convert a PointCloud2 message to CObservationPointCloud.
 *
 *  - Picks the richest available point-map type
 *    (CGenericPointsMap > CPointsMapXYZIRT > CPointsMapXYZI > CSimplePointsMap)
 *  - Applies fixLivoxTimestampsIfNeeded() automatically.
 *  - The caller is responsible for filling sensorPose afterwards.
 *
 * \return nullptr if the point cloud lacks x/y/z fields.
 */
mrpt::obs::CObservationPointCloud::Ptr pointCloud2ToObservation(
    const sensor_msgs::msg::PointCloud2& pts, const std::string& sensorLabel);

/** Convert a LaserScan message to CObservation2DRangeScan.
 *
 * \param sensorPose  The sensor pose in the robot frame (used by
 *                    mrpt::ros2bridge::fromROS to project rays).
 */
mrpt::obs::CObservation2DRangeScan::Ptr laserScanToObservation(
    const sensor_msgs::msg::LaserScan& scan,
    const mrpt::poses::CPose3D& sensorPose,
    const std::string& sensorLabel);

/** Convert an Imu message to CObservationIMU.
 *  The caller is responsible for filling sensorPose afterwards.
 */
mrpt::obs::CObservationIMU::Ptr imuToObservation(
    const sensor_msgs::msg::Imu& imu, const std::string& sensorLabel);

/** Convert a NavSatFix message to CObservationGPS.
 *  The caller is responsible for filling sensorPose afterwards.
 */
mrpt::obs::CObservationGPS::Ptr navSatFixToObservation(
    const sensor_msgs::msg::NavSatFix& gps, const std::string& sensorLabel);

/** Convert an Odometry message to CObservationOdometry. */
mrpt::obs::CObservationOdometry::Ptr odometryToObservation(
    const nav_msgs::msg::Odometry& odo, const std::string& sensorLabel);

/** @} */

}  // namespace mrpt::ros2bridge
