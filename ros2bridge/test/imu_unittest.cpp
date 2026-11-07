/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2026, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/ros2bridge/imu.h>
#include <mrpt/ros2bridge/time.h>

#include <sensor_msgs/msg/imu.hpp>

TEST(IMU, FromROS_CopiesTimestamp)
{
  sensor_msgs::msg::Imu msg;
  msg.header.stamp.sec = 1700000000;
  msg.header.stamp.nanosec = 250000000;

  mrpt::obs::CObservationIMU obs;
  ASSERT_TRUE(mrpt::ros2bridge::fromROS(msg, obs));

  EXPECT_NEAR(mrpt::Clock::toDouble(obs.timestamp), 1700000000.25, 1e-6);
}

TEST(IMU, FromROS_CopiesMeasurements)
{
  using namespace mrpt::obs;

  sensor_msgs::msg::Imu msg;
  msg.angular_velocity.x = 0.1;
  msg.angular_velocity.y = 0.2;
  msg.angular_velocity.z = 0.3;
  msg.linear_acceleration.x = 1.0;
  msg.linear_acceleration.y = 2.0;
  msg.linear_acceleration.z = 9.8;
  // Orientation not available:
  msg.orientation_covariance[0] = -1;

  CObservationIMU obs;
  ASSERT_TRUE(mrpt::ros2bridge::fromROS(msg, obs));

  EXPECT_FALSE(obs.has(IMU_ORI_QUAT_W));
  ASSERT_TRUE(obs.has(IMU_WZ));
  ASSERT_TRUE(obs.has(IMU_Z_ACC));
  EXPECT_DOUBLE_EQ(obs.get(IMU_WX), 0.1);
  EXPECT_DOUBLE_EQ(obs.get(IMU_WZ), 0.3);
  EXPECT_DOUBLE_EQ(obs.get(IMU_Y_ACC), 2.0);
  EXPECT_DOUBLE_EQ(obs.get(IMU_Z_ACC), 9.8);
}

TEST(IMU, ToROS_OrientationFromYawPitchRoll)
{
  using namespace mrpt::obs;

  CObservationIMU obs;
  obs.set(IMU_YAW, 0.5);
  obs.set(IMU_PITCH, -0.1);
  obs.set(IMU_ROLL, 0.2);
  obs.set(IMU_X_ACC, 0.0);
  obs.set(IMU_Y_ACC, 0.0);
  obs.set(IMU_Z_ACC, 9.8);

  sensor_msgs::msg::Imu msg;
  ASSERT_TRUE(mrpt::ros2bridge::toROS(obs, std_msgs::msg::Header(), msg));

  mrpt::math::CQuaternionDouble q;
  mrpt::poses::CPose3D::FromYawPitchRoll(0.5, -0.1, 0.2).getAsQuaternion(q);
  EXPECT_NEAR(msg.orientation.w, q.r(), 1e-9);
  EXPECT_NEAR(msg.orientation.x, q.x(), 1e-9);
  EXPECT_NEAR(msg.orientation.y, q.y(), 1e-9);
  EXPECT_NEAR(msg.orientation.z, q.z(), 1e-9);

  // Covariances are diagonal; unavailable fields are flagged with -1:
  EXPECT_GT(msg.orientation_covariance[0], 0);
  EXPECT_EQ(msg.orientation_covariance[1], 0);
  EXPECT_EQ(msg.linear_acceleration_covariance[3], 0);
  EXPECT_GT(msg.linear_acceleration_covariance[8], 0);
  EXPECT_EQ(msg.angular_velocity_covariance[0], -1);
}
