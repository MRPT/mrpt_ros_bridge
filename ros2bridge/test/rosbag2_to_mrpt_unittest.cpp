/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2026, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/ros2bridge/rosbag2_to_mrpt.h>
#include <mrpt/ros2bridge/time.h>

#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <string>
#include <tf2/buffer_core.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

namespace
{
// Serializes a message as it would be stored in a rosbag2 file.
template <typename MSG>
rosbag2_storage::SerializedBagMessage toBagMessage(const MSG& msg, const std::string& topic)
{
  rclcpp::Serialization<MSG> serializer;
  rclcpp::SerializedMessage serialized;
  serializer.serialize_message(&msg, &serialized);

  rosbag2_storage::SerializedBagMessage m;
  m.topic_name = topic;
  m.serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
      new rcutils_uint8_array_t(serialized.release_rcl_serialized_message()),
      [](rcutils_uint8_array_t* p)
      {
        (void)rcutils_uint8_array_fini(p);
        delete p;
      });
  return m;
}

std_msgs::msg::Header header(const std::string& frame)
{
  std_msgs::msg::Header h;
  h.frame_id = frame;
  h.stamp.sec = 1700000000;
  h.stamp.nanosec = 500000000;
  return h;
}

const std::optional<mrpt::poses::CPose3D> kFixedPose = mrpt::poses::CPose3D(1, 2, 3, 0, 0, 0);
}  // namespace

TEST(Rosbag2ToMRPT, IMU)
{
  sensor_msgs::msg::Imu imu;
  imu.header = header("imu");
  imu.linear_acceleration.x = 0.5;
  imu.linear_acceleration.z = 9.8;
  imu.angular_velocity.y = 0.25;

  tf2::BufferCore tf;
  const auto obs =
      mrpt::ros2bridge::rosbag2ToIMU("imu", toBagMessage(imu, "/imu"), tf, "base_link", kFixedPose);
  ASSERT_EQ(obs.size(), 1U);
  const auto o = std::dynamic_pointer_cast<mrpt::obs::CObservationIMU>(obs.at(0));
  ASSERT_TRUE(o);
  EXPECT_DOUBLE_EQ(o->get(mrpt::obs::IMU_X_ACC), 0.5);
  EXPECT_DOUBLE_EQ(o->get(mrpt::obs::IMU_Z_ACC), 9.8);
  EXPECT_DOUBLE_EQ(o->get(mrpt::obs::IMU_WY), 0.25);
  EXPECT_EQ(o->timestamp, mrpt::ros2bridge::fromROS(imu.header.stamp));
  EXPECT_DOUBLE_EQ(o->sensorPose.z(), 3.0);
}

TEST(Rosbag2ToMRPT, Image)
{
  sensor_msgs::msg::Image img;
  img.header = header("cam");
  img.width = 4;
  img.height = 3;
  img.encoding = "rgb8";
  img.step = 12;
  img.data.assign(36, 200);

  tf2::BufferCore tf;
  const auto obs = mrpt::ros2bridge::rosbag2ToImage(
      "cam", toBagMessage(img, "/cam"), tf, "base_link", kFixedPose);
  ASSERT_EQ(obs.size(), 1U);
  const auto o = std::dynamic_pointer_cast<mrpt::obs::CObservationImage>(obs.at(0));
  ASSERT_TRUE(o);
  EXPECT_EQ(o->image.getWidth(), 4);
  EXPECT_EQ(o->image.getHeight(), 3);
  EXPECT_EQ(o->image.at<uint8_t>(2, 1, 0), 200);
}

TEST(Rosbag2ToMRPT, PointCloud2)
{
  sensor_msgs::msg::PointCloud2 pc;
  pc.header = header("lidar");
  sensor_msgs::PointCloud2Modifier mod(pc);
  mod.setPointCloud2FieldsByString(1, "xyz");
  mod.resize(100);
  sensor_msgs::PointCloud2Iterator<float> x(pc, "x");
  for (int i = 0; i < 100; i++, ++x)
  {
    x[0] = static_cast<float>(i);
    x[1] = 1.0f;
    x[2] = 2.0f;
  }

  tf2::BufferCore tf;
  const auto obs = mrpt::ros2bridge::rosbag2ToPointCloud2(
      "lidar", toBagMessage(pc, "/points"), tf, "base_link", kFixedPose);
  ASSERT_EQ(obs.size(), 1U);
  const auto o = std::dynamic_pointer_cast<mrpt::obs::CObservationPointCloud>(obs.at(0));
  ASSERT_TRUE(o && o->pointcloud);
  ASSERT_EQ(o->pointcloud->size(), 100U);
  const auto& xs = o->pointcloud->getPointsBufferRef_x();
  const auto& zs = o->pointcloud->getPointsBufferRef_z();
  EXPECT_FLOAT_EQ(xs[42], 42.0f);
  EXPECT_FLOAT_EQ(zs[42], 2.0f);
}

TEST(Rosbag2ToMRPT, LaserScan)
{
  sensor_msgs::msg::LaserScan scan;
  scan.header = header("laser");
  scan.angle_min = -1.0f;
  scan.angle_max = 1.0f;
  scan.angle_increment = 0.5f;
  scan.range_min = 0.1f;
  scan.range_max = 10.0f;
  scan.ranges = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f};

  tf2::BufferCore tf;
  const auto obs = mrpt::ros2bridge::rosbag2ToLidar2D(
      "laser", toBagMessage(scan, "/scan"), tf, "base_link", kFixedPose);
  ASSERT_EQ(obs.size(), 1U);
  const auto o = std::dynamic_pointer_cast<mrpt::obs::CObservation2DRangeScan>(obs.at(0));
  ASSERT_TRUE(o);
  ASSERT_EQ(o->getScanSize(), 5U);
  EXPECT_FLOAT_EQ(o->getScanRange(2), 3.0f);
}

TEST(Rosbag2ToMRPT, NavSatFix)
{
  sensor_msgs::msg::NavSatFix fix;
  fix.header = header("gps");
  fix.latitude = 36.83;
  fix.longitude = -2.40;
  fix.altitude = 100.0;

  tf2::BufferCore tf;
  const auto obs =
      mrpt::ros2bridge::rosbag2ToGPS("gps", toBagMessage(fix, "/gps"), tf, "base_link", kFixedPose);
  ASSERT_EQ(obs.size(), 1U);
  EXPECT_TRUE(std::dynamic_pointer_cast<mrpt::obs::CObservationGPS>(obs.at(0)));
}

TEST(Rosbag2ToMRPT, EmptyPayloadThrows)
{
  sensor_msgs::msg::Imu imu;
  imu.header = header("imu");
  auto m = toBagMessage(imu, "/imu");
  tf2::BufferCore tf;

  m.serialized_data->buffer_length = 0;
  EXPECT_ANY_THROW(mrpt::ros2bridge::rosbag2ToIMU("imu", m, tf, "base_link", kFixedPose));
}

TEST(Rosbag2ToMRPT, Odometry)
{
  nav_msgs::msg::Odometry odo;
  odo.header = header("odom");
  odo.child_frame_id = "base_link";
  odo.pose.pose.position.x = 4.5;
  odo.pose.pose.orientation.w = 1.0;

  const auto obs = mrpt::ros2bridge::rosbag2ToOdometry("odom", toBagMessage(odo, "/odom"));
  ASSERT_EQ(obs.size(), 1U);
  const auto o = std::dynamic_pointer_cast<mrpt::obs::CObservationOdometry>(obs.at(0));
  ASSERT_TRUE(o);
  EXPECT_DOUBLE_EQ(o->odometry.x(), 4.5);
}

TEST(Rosbag2ToMRPT, StaticTf)
{
  tf2_msgs::msg::TFMessage tfs;
  geometry_msgs::msg::TransformStamped t;
  t.header = header("base_link");
  t.child_frame_id = "sensor";
  t.transform.translation.x = 0.7;
  t.transform.rotation.w = 1.0;
  tfs.transforms.push_back(t);

  tf2::BufferCore tf;
  mrpt::ros2bridge::rosbag2ToTf<true>(tf, toBagMessage(tfs, "/tf_static"));
  const auto r = tf.lookupTransform("base_link", "sensor", tf2::TimePointZero);
  EXPECT_DOUBLE_EQ(r.transform.translation.x, 0.7);
}
