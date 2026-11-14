/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2026, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
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
  EXPECT_EQ(o->pointcloud->size(), 100U);
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
