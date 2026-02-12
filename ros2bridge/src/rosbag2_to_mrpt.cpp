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
#include <mrpt/ros2bridge/imu.h>
#include <mrpt/ros2bridge/laser_scan.h>
#include <mrpt/ros2bridge/point_cloud2.h>
#include <mrpt/ros2bridge/pose.h>
#include <mrpt/ros2bridge/ros_to_mrpt_obs.h>  // lookupSensorPose, fixLivox…
#include <mrpt/ros2bridge/rosbag2_to_mrpt.h>
#include <mrpt/ros2bridge/time.h>
#include <mrpt/version.h>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/buffer_core.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#if CV_BRIDGE_VERSION < 0x030400
#include <cv_bridge/cv_bridge.h>
#else
#include <cv_bridge/cv_bridge.hpp>
#endif

#include <memory>

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
  rclcpp::SerializedMessage serMsg(*rosmsg.serialized_data);
  static rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serializer;

  sensor_msgs::msg::PointCloud2 pts;
  serializer.deserialize_message(&serMsg, &pts);

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
  rclcpp::SerializedMessage serMsg(*rosmsg.serialized_data);
  static rclcpp::Serialization<sensor_msgs::msg::LaserScan> serializer;

  sensor_msgs::msg::LaserScan scan;
  serializer.deserialize_message(&serMsg, &scan);

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
  rclcpp::SerializedMessage serMsg(*rosmsg.serialized_data);
  static rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serializer;

  sensor_msgs::msg::PointCloud2 pts;
  serializer.deserialize_message(&serMsg, &pts);

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
  rclcpp::SerializedMessage serMsg(*rosmsg.serialized_data);
  static rclcpp::Serialization<sensor_msgs::msg::Imu> serializer;

  sensor_msgs::msg::Imu imu;
  serializer.deserialize_message(&serMsg, &imu);

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
    const std::optional<mrpt::poses::CPose3D>& fixedSensorPose)
{
  rclcpp::SerializedMessage serMsg(*rosmsg.serialized_data);
  static rclcpp::Serialization<sensor_msgs::msg::NavSatFix> serializer;

  sensor_msgs::msg::NavSatFix gps;
  serializer.deserialize_message(&serMsg, &gps);

  auto obs = navSatFixToObservation(gps, std::string(sensorLabel));

  // Resolve sensor pose:
  if (!lookupSensorPose(
          obs->sensorPose, tfBuffer, gps.header.frame_id, base_link_frame, fixedSensorPose))
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
  rclcpp::SerializedMessage serMsg(*rosmsg.serialized_data);
  static rclcpp::Serialization<nav_msgs::msg::Odometry> serializer;

  nav_msgs::msg::Odometry odo;
  serializer.deserialize_message(&serMsg, &odo);

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
  rclcpp::SerializedMessage serMsg(*rosmsg.serialized_data);
  static rclcpp::Serialization<sensor_msgs::msg::Image> serializer;

  auto image = std::make_shared<sensor_msgs::msg::Image>();
  serializer.deserialize_message(&serMsg, image.get());

  auto imgObs = mrpt::obs::CObservationImage::Create();

  imgObs->sensorLabel = sensorLabel;
  imgObs->timestamp = mrpt::ros2bridge::fromROS(image->header.stamp);

  auto cv_ptr = cv_bridge::toCvShare(image);
  imgObs->image = mrpt::img::CImage(cv_ptr->image, mrpt::img::DEEP_COPY);

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
  static rclcpp::Serialization<tf2_msgs::msg::TFMessage> tfSerializer;

  tf2_msgs::msg::TFMessage tfs;
  rclcpp::SerializedMessage msgData(*rosmsg.serialized_data);
  tfSerializer.deserialize_message(&msgData, &tfs);

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
