/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2026, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

/**
 * @file   rosbag2_to_mrpt.cpp
 * @brief  Deserialize rosbag2 serialized messages and convert to MRPT
 *         observations.
 * @author Jose Luis Blanco Claraco
 * @date   2025
 */

#include <mrpt/core/exceptions.h>
#include <mrpt/img/CImage.h>
#include <mrpt/maps/CGenericPointsMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CObservationRotatingScan.h>
#include <mrpt/ros2bridge/gps.h>
#include <mrpt/ros2bridge/image.h>
#include <mrpt/ros2bridge/imu.h>
#include <mrpt/ros2bridge/laser_scan.h>
#include <mrpt/ros2bridge/point_cloud2.h>
#include <mrpt/ros2bridge/pose.h>
#include <mrpt/ros2bridge/ros_to_mrpt_obs.h>  // lookupSensorPose, fixLivox…
#include <mrpt/ros2bridge/rosbag2_to_mrpt.h>
#include <mrpt/ros2bridge/time.h>
#include <mrpt/version.h>
#include <rmw/error_handling.h>
#include <rmw/rmw.h>

#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rosidl_typesupport_cpp/message_type_support.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <stdexcept>
#include <string>
#include <tf2/buffer_core.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

namespace
{
// Deserializes straight from the bag buffer: wrapping it into an
// rclcpp::SerializedMessage first would copy the whole payload.
template <typename MSG>
void deserializeBagMessage(const rosbag2_storage::SerializedBagMessage& m, MSG& msg)
{
  // Shorter than a CDR header: some rmw implementations would read past it.
  if (!m.serialized_data || m.serialized_data->buffer_length < 4)
  {
    throw std::runtime_error("Empty or truncated message in topic '" + m.topic_name + "'");
  }
  const auto ret = rmw_deserialize(
      m.serialized_data.get(), rosidl_typesupport_cpp::get_message_type_support_handle<MSG>(),
      &msg);
  if (ret != RMW_RET_OK)
  {
    const std::string err = rmw_get_error_string().str;
    rmw_reset_error();
    throw std::runtime_error(
        "Failed to deserialize a message of topic '" + m.topic_name + "': " + err);
  }
}
}  // namespace

namespace mrpt::ros2bridge
{

// -----------------------------------------------------------------------
//  rosbag2ToPointCloud2
// -----------------------------------------------------------------------
ObsVector rosbag2ToPointCloud2(
    std::string_view sensorLabel,
    const rosbag2_storage::SerializedBagMessage& rosmsg,
    tf2::BufferCore& tfBuffer,
    const std::string& base_link_frame,
    const std::optional<mrpt::poses::CPose3D>& fixedSensorPose)
{
  sensor_msgs::msg::PointCloud2 pts;
  deserializeBagMessage(rosmsg, pts);

  // Use the shared helper that picks the richest map type and
  // fixes Livox timestamps:
  auto ptsObs = pointCloud2ToObservation(pts, std::string(sensorLabel));
  if (!ptsObs)
  {
    return {};  // missing x/y/z fields
  }

  // Resolve sensor pose:
  if (!lookupSensorPose(
          ptsObs->sensorPose, tfBuffer, pts.header.frame_id, base_link_frame, fixedSensorPose))
  {
    return {};
  }

  return {ptsObs};
}

// -----------------------------------------------------------------------
//  rosbag2ToLidar2D
// -----------------------------------------------------------------------
ObsVector rosbag2ToLidar2D(
    std::string_view sensorLabel,
    const rosbag2_storage::SerializedBagMessage& rosmsg,
    tf2::BufferCore& tfBuffer,
    const std::string& base_link_frame,
    const std::optional<mrpt::poses::CPose3D>& fixedSensorPose)
{
  sensor_msgs::msg::LaserScan scan;
  deserializeBagMessage(rosmsg, scan);

  // Resolve sensor pose first (needed by the fromROS conversion):
  mrpt::poses::CPose3D sensorPose;
  if (!lookupSensorPose(
          sensorPose, tfBuffer, scan.header.frame_id, base_link_frame, fixedSensorPose))
  {
    return {};
  }

  auto obs = laserScanToObservation(scan, sensorPose, std::string(sensorLabel));

  // Override the timestamp explicitly (fromROS already sets it, but be safe):
  obs->timestamp = mrpt::ros2bridge::fromROS(scan.header.stamp);

  return {obs};
}

// -----------------------------------------------------------------------
//  rosbag2ToRotatingScan
// -----------------------------------------------------------------------
ObsVector rosbag2ToRotatingScan(
    std::string_view sensorLabel,
    const rosbag2_storage::SerializedBagMessage& rosmsg,
    tf2::BufferCore& tfBuffer,
    const std::string& base_link_frame,
    const std::optional<mrpt::poses::CPose3D>& fixedSensorPose)
{
  sensor_msgs::msg::PointCloud2 pts;
  deserializeBagMessage(rosmsg, pts);

  // Check required fields:
  const std::set<std::string> fields = mrpt::ros2bridge::extractFields(pts);

  if (!fields.count("x") || !fields.count("y") || !fields.count("z") || !fields.count("ring"))
  {
    return {};
  }

  auto obsRotScan = mrpt::obs::CObservationRotatingScan::Create();
  const mrpt::poses::CPose3D sensorPoseForConversion;  // identity for the point conversion

  if (!mrpt::ros2bridge::fromROS(pts, *obsRotScan, sensorPoseForConversion))
  {
    return {};  // conversion failed
  }

  obsRotScan->sensorLabel = sensorLabel;
  obsRotScan->timestamp = mrpt::ros2bridge::fromROS(pts.header.stamp);

  // Resolve sensor pose:
  if (!lookupSensorPose(
          obsRotScan->sensorPose, tfBuffer, pts.header.frame_id, base_link_frame, fixedSensorPose))
  {
    return {};
  }

  return {obsRotScan};
}

// -----------------------------------------------------------------------
//  rosbag2ToIMU
// -----------------------------------------------------------------------
ObsVector rosbag2ToIMU(
    std::string_view sensorLabel,
    const rosbag2_storage::SerializedBagMessage& rosmsg,
    tf2::BufferCore& tfBuffer,
    const std::string& base_link_frame,
    const std::optional<mrpt::poses::CPose3D>& fixedSensorPose)
{
  sensor_msgs::msg::Imu imu;
  deserializeBagMessage(rosmsg, imu);

  auto obs = imuToObservation(imu, std::string(sensorLabel));

  // Resolve sensor pose:
  if (!lookupSensorPose(
          obs->sensorPose, tfBuffer, imu.header.frame_id, base_link_frame, fixedSensorPose))
  {
    return {};
  }

  return {obs};
}

// -----------------------------------------------------------------------
//  rosbag2ToGPS
// -----------------------------------------------------------------------
ObsVector rosbag2ToGPS(
    std::string_view sensorLabel,
    const rosbag2_storage::SerializedBagMessage& rosmsg,
    tf2::BufferCore& tfBuffer,
    const std::string& base_link_frame,
    const std::optional<mrpt::poses::CPose3D>& fixedSensorPose,
    bool isGpsFix)
{
  mrpt::obs::CObservationGPS::Ptr obs;
  std::string frame_id;

  if (isGpsFix)
  {
    gps_msgs::msg::GPSFix gps;
    deserializeBagMessage(rosmsg, gps);
    obs = gpsFixToObservation(gps, std::string(sensorLabel));
    frame_id = gps.header.frame_id;
  }
  else
  {
    sensor_msgs::msg::NavSatFix gps;
    deserializeBagMessage(rosmsg, gps);
    obs = navSatFixToObservation(gps, std::string(sensorLabel));
    frame_id = gps.header.frame_id;
  }

  // Resolve sensor pose:
  if (!lookupSensorPose(obs->sensorPose, tfBuffer, frame_id, base_link_frame, fixedSensorPose))
  {
    return {};
  }

  return {obs};
}

// -----------------------------------------------------------------------
//  rosbag2ToOdometry
// -----------------------------------------------------------------------
ObsVector rosbag2ToOdometry(
    std::string_view sensorLabel, const rosbag2_storage::SerializedBagMessage& rosmsg)
{
  nav_msgs::msg::Odometry odo;
  deserializeBagMessage(rosmsg, odo);

  auto obs = odometryToObservation(odo, std::string(sensorLabel));

  return {obs};
}

// -----------------------------------------------------------------------
//  rosbag2ToImage
// -----------------------------------------------------------------------
ObsVector rosbag2ToImage(
    std::string_view sensorLabel,
    const rosbag2_storage::SerializedBagMessage& rosmsg,
    tf2::BufferCore& tfBuffer,
    const std::string& base_link_frame,
    const std::optional<mrpt::poses::CPose3D>& fixedSensorPose)
{
  auto image = std::make_shared<sensor_msgs::msg::Image>();
  deserializeBagMessage(rosmsg, *image);

  auto imgObs = mrpt::obs::CObservationImage::Create();

  imgObs->sensorLabel = sensorLabel;
  imgObs->timestamp = mrpt::ros2bridge::fromROS(image->header.stamp);

  imgObs->image = mrpt::ros2bridge::fromROS(*image);

  // Note: CObservationImage uses cameraPose, not sensorPose:
  if (!lookupSensorPose(
          imgObs->cameraPose, tfBuffer, image->header.frame_id, base_link_frame, fixedSensorPose))
  {
    return {};
  }

  return {imgObs};
}

// -----------------------------------------------------------------------
//  rosbag2ToTf  (explicit template instantiations)
// -----------------------------------------------------------------------
template <bool isStatic>
void rosbag2ToTf(tf2::BufferCore& tfBuffer, const rosbag2_storage::SerializedBagMessage& rosmsg)
{
  tf2_msgs::msg::TFMessage tfs;
  deserializeBagMessage(rosmsg, tfs);

  for (auto& tf : tfs.transforms)
  {
    try
    {
      tfBuffer.setTransform(tf, "bagfile", isStatic);
    }
    catch (const tf2::TransformException& e)
    {
      std::cerr << "[rosbag2ToTf] Error: " << e.what() << "\n";
    }
  }
}

// Explicit instantiations so the linker finds them:
template void rosbag2ToTf<true>(tf2::BufferCore&, const rosbag2_storage::SerializedBagMessage&);
template void rosbag2ToTf<false>(tf2::BufferCore&, const rosbag2_storage::SerializedBagMessage&);

}  // namespace mrpt::ros2bridge
