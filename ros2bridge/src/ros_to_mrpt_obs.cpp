/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2026, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

/**
 * @file   ros_to_mrpt_obs.cpp
 * @brief  Convert already-deserialized ROS 2 messages to MRPT observations.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */

#include <mrpt/maps/CGenericPointsMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/ros2bridge/gps.h>
#include <mrpt/ros2bridge/imu.h>
#include <mrpt/ros2bridge/laser_scan.h>
#include <mrpt/ros2bridge/point_cloud2.h>
#include <mrpt/ros2bridge/pose.h>
#include <mrpt/ros2bridge/ros_to_mrpt_obs.h>
#include <mrpt/ros2bridge/time.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/version.h>

#if MRPT_VERSION < 0x030000
#include <mrpt/maps/CPointsMapXYZI.h>
#include <mrpt/maps/CPointsMapXYZIRT.h>
#endif

#include <algorithm>  // minmax_element
#include <stdexcept>
#include <tf2/buffer_core.hpp>
#include <tf2/convert.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace mrpt::ros2bridge
{

// -----------------------------------------------------------------------
//  lookupSensorPose
// -----------------------------------------------------------------------
bool lookupSensorPose(
    mrpt::poses::CPose3D& des,
    tf2::BufferCore& tfBuffer,
    const std::string& target_frame,
    const std::string& reference_frame,
    const std::optional<mrpt::poses::CPose3D>& fixedSensorPose,
    mrpt::system::COutputLogger* logger)
{
  if (fixedSensorPose)
  {
    des = fixedSensorPose.value();
    return true;
  }

  try
  {
    geometry_msgs::msg::TransformStamped ref_to_trgFrame =
        tfBuffer.lookupTransform(reference_frame, target_frame, {} /*latest*/);

    tf2::Transform tf;
    tf2::fromMsg(ref_to_trgFrame.transform, tf);
    des = mrpt::ros2bridge::fromROS(tf);

    if (logger && logger->isLoggingLevelVisible(mrpt::system::LVL_DEBUG))
    {
      logger->logFmt(
          mrpt::system::LVL_DEBUG, "[lookupSensorPose] Found pose %s -> %s: %s",
          reference_frame.c_str(), target_frame.c_str(), des.asString().c_str());
    }

    return true;
  }
  catch (const tf2::TransformException& e)
  {
    const auto errMsg = mrpt::format(
        "[lookupSensorPose] reference_frame: '%s', target_frame: '%s': %s", reference_frame.c_str(),
        target_frame.c_str(), e.what());

    if (logger)
    {
      logger->logStr(mrpt::system::LVL_ERROR, errMsg);
    }
    else
    {
      std::cerr << errMsg << "\n";
    }
    return false;
  }
}

// -----------------------------------------------------------------------
//  fixLivoxTimestampsIfNeeded
// -----------------------------------------------------------------------
void fixLivoxTimestampsIfNeeded(mrpt::maps::CPointsMap& pts)
{
#if MRPT_VERSION >= 0x020f00  // 2.15.0
  auto* pGeneric = dynamic_cast<mrpt::maps::CGenericPointsMap*>(&pts);
  if (!pGeneric)
  {
    return;
  }

#if MRPT_VERSION >= 0x020f03  // 2.15.3
  auto* ts =
      pGeneric->getPointsBufferRef_float_field(mrpt::maps::CPointsMap::POINT_FIELD_TIMESTAMP);
#else
  auto* ts =
      pGeneric->getPointsBufferRef_float_field(mrpt::maps::CPointsMapXYZIRT::POINT_FIELD_TIMESTAMP);
#endif

  if (ts != nullptr && !ts->empty())
  {
    const auto [minIt, maxIt] = std::minmax_element(ts->begin(), ts->end());
    const float time_span = *maxIt - *minIt;
    if (time_span > 1e5F)
    {
      // they must be nanoseconds, convert to seconds:
      for (auto& t : *ts)
      {
        t *= 1e-9F;
      }
    }
  }
#else
  (void)pts;  // unused in older MRPT
#endif
}

// -----------------------------------------------------------------------
//  pointCloud2ToObservation
// -----------------------------------------------------------------------
mrpt::obs::CObservationPointCloud::Ptr pointCloud2ToObservation(
    const sensor_msgs::msg::PointCloud2& pts, const std::string& sensorLabel)
{
  const std::set<std::string> fields = mrpt::ros2bridge::extractFields(pts);

  // We need at least X Y Z:
  if (!fields.count("x") || !fields.count("y") || !fields.count("z"))
  {
    return nullptr;
  }

  mrpt::maps::CPointsMap::Ptr mapPtr;

  // If we have anything beyond (x,y,z), use a generic multi-field cloud:
  if (fields.size() > 3)
  {
#if MRPT_VERSION >= 0x020f00  // 2.15.0
    auto p = mrpt::maps::CGenericPointsMap::Create();
    if (!mrpt::ros2bridge::fromROS(pts, *p))
    {
      throw std::runtime_error(
          "pointCloud2ToObservation: could not convert PointCloud2 to "
          "CGenericPointsMap");
    }

    fixLivoxTimestampsIfNeeded(*p);
    mapPtr = p;
#else
    // Fallback for older MRPT without CGenericPointsMap:
#if MRPT_VERSION >= 0x020b04
    if (fields.count("ring") || fields.count("time"))
    {
      auto p = mrpt::maps::CPointsMapXYZIRT::Create();
      if (mrpt::ros2bridge::fromROS(pts, *p))
      {
        mapPtr = p;
      }
    }
#endif
    if (!mapPtr && fields.count("intensity"))
    {
      auto p = mrpt::maps::CPointsMapXYZI::Create();
      if (mrpt::ros2bridge::fromROS(pts, *p))
      {
        mapPtr = p;
      }
    }
    // Final fallback to simple XYZ:
    if (!mapPtr)
    {
      auto p = mrpt::maps::CSimplePointsMap::Create();
      if (!mrpt::ros2bridge::fromROS(pts, *p))
      {
        throw std::runtime_error(
            "pointCloud2ToObservation: could not convert PointCloud2 to "
            "CSimplePointsMap");
      }
      mapPtr = p;
    }
#endif
  }
  else
  {
    // Pure XYZ:
    auto p = mrpt::maps::CSimplePointsMap::Create();
    if (!mrpt::ros2bridge::fromROS(pts, *p))
    {
      throw std::runtime_error(
          "pointCloud2ToObservation: could not convert PointCloud2 to "
          "CSimplePointsMap");
    }
    mapPtr = p;
  }

  auto obs = mrpt::obs::CObservationPointCloud::Create();
  obs->timestamp = mrpt::ros2bridge::fromROS(pts.header.stamp);
  obs->sensorLabel = sensorLabel;
  obs->pointcloud = mapPtr;

  return obs;
}

// -----------------------------------------------------------------------
//  laserScanToObservation
// -----------------------------------------------------------------------
mrpt::obs::CObservation2DRangeScan::Ptr laserScanToObservation(
    const sensor_msgs::msg::LaserScan& scan,
    const mrpt::poses::CPose3D& sensorPose,
    const std::string& sensorLabel)
{
  auto obs = mrpt::obs::CObservation2DRangeScan::Create();
  mrpt::ros2bridge::fromROS(scan, sensorPose, *obs);

  obs->sensorLabel = sensorLabel;
  // Note: fromROS already sets the timestamp from scan.header.stamp

  return obs;
}

// -----------------------------------------------------------------------
//  imuToObservation
// -----------------------------------------------------------------------
mrpt::obs::CObservationIMU::Ptr imuToObservation(
    const sensor_msgs::msg::Imu& imu, const std::string& sensorLabel)
{
  auto obs = mrpt::obs::CObservationIMU::Create();
  mrpt::ros2bridge::fromROS(imu, *obs);

  obs->sensorLabel = sensorLabel;
  obs->timestamp = mrpt::ros2bridge::fromROS(imu.header.stamp);

  return obs;
}

// -----------------------------------------------------------------------
//  navSatFixToObservation
// -----------------------------------------------------------------------
mrpt::obs::CObservationGPS::Ptr navSatFixToObservation(
    const sensor_msgs::msg::NavSatFix& gps, const std::string& sensorLabel)
{
  auto obs = mrpt::obs::CObservationGPS::Create();
  mrpt::ros2bridge::fromROS(gps, *obs);

  obs->sensorLabel = sensorLabel;
  obs->timestamp = mrpt::ros2bridge::fromROS(gps.header.stamp);

  return obs;
}

// -----------------------------------------------------------------------
//  gpsFixToObservation
// -----------------------------------------------------------------------
mrpt::obs::CObservationGPS::Ptr gpsFixToObservation(
    const gps_msgs::msg::GPSFix& gps, const std::string& sensorLabel)
{
  auto obs = mrpt::obs::CObservationGPS::Create();
  mrpt::ros2bridge::fromROS(gps, *obs);

  obs->sensorLabel = sensorLabel;
  obs->timestamp = mrpt::ros2bridge::fromROS(gps.header.stamp);

  return obs;
}

// -----------------------------------------------------------------------
//  odometryToObservation
// -----------------------------------------------------------------------
mrpt::obs::CObservationOdometry::Ptr odometryToObservation(
    const nav_msgs::msg::Odometry& odo, const std::string& sensorLabel)
{
  auto obs = mrpt::obs::CObservationOdometry::Create();

  obs->sensorLabel = sensorLabel;
  obs->timestamp = mrpt::ros2bridge::fromROS(odo.header.stamp);

  const auto pose = mrpt::ros2bridge::fromROS(odo.pose);
  obs->odometry = {pose.mean.x(), pose.mean.y(), pose.mean.yaw()};

  obs->hasVelocities = true;
  obs->velocityLocal.vx = odo.twist.twist.linear.x;
  obs->velocityLocal.vy = odo.twist.twist.linear.y;
  obs->velocityLocal.omega = odo.twist.twist.angular.z;

  return obs;
}

}  // namespace mrpt::ros2bridge
