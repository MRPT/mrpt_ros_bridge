/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2026, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/ros2bridge/camera_info.h>
#include <mrpt/ros2bridge/odometry.h>

namespace
{
mrpt::img::TCamera roundTrip(const mrpt::img::TCamera& c)
{
  return mrpt::ros2bridge::fromROS(mrpt::ros2bridge::toROS(c, std_msgs::msg::Header()));
}
}  // namespace

TEST(CameraInfo, PlumbBobRoundTrip)
{
  mrpt::img::TCamera c;
  c.ncols = 1024;
  c.nrows = 768;
  c.setIntrinsicParamsFromValues(923.5, 922.2, 507.2, 383.6);
  c.setDistortionPlumbBob(-0.35, 0.16, 1.6e-3, 3.6e-4);

  const auto ci = mrpt::ros2bridge::toROS(c, std_msgs::msg::Header());
  EXPECT_EQ(ci.distortion_model, "plumb_bob");
  ASSERT_EQ(ci.d.size(), 5u);
  EXPECT_EQ(ci.width, 1024u);
  EXPECT_DOUBLE_EQ(ci.k[0], 923.5);
  EXPECT_DOUBLE_EQ(ci.p[6], 383.6);

  const auto c2 = roundTrip(c);
  EXPECT_EQ(c2.distortion, mrpt::img::DistortionModel::plumb_bob);
  EXPECT_EQ(c2.nrows, 768u);
  EXPECT_DOUBLE_EQ(c2.fy(), 922.2);
  EXPECT_DOUBLE_EQ(c2.cx(), 507.2);
  EXPECT_DOUBLE_EQ(c2.k1(), -0.35);
  EXPECT_DOUBLE_EQ(c2.p2(), 3.6e-4);
}

TEST(CameraInfo, RationalAndFisheye)
{
  mrpt::img::TCamera c;
  c.setDistortionPlumbBob(0.1, 0.2, 0.3, 0.4, 0.5);
  c.k6(0.6);
  EXPECT_EQ(mrpt::ros2bridge::toROS(c, {}).distortion_model, "rational_polynomial");
  EXPECT_DOUBLE_EQ(roundTrip(c).k6(), 0.6);

  c.setDistortionKannalaBrandt(0.1, 0.2, 0.3, 0.4);
  const auto ci = mrpt::ros2bridge::toROS(c, {});
  EXPECT_EQ(ci.distortion_model, "equidistant");
  ASSERT_EQ(ci.d.size(), 4u);
  const auto c2 = roundTrip(c);
  EXPECT_EQ(c2.distortion, mrpt::img::DistortionModel::kannala_brandt);
  EXPECT_DOUBLE_EQ(c2.k3(), 0.3);
  EXPECT_DOUBLE_EQ(c2.k4(), 0.4);
}

TEST(Odometry, ToROS)
{
  mrpt::obs::CObservationOdometry o;
  o.odometry = mrpt::poses::CPose2D(1.0, 2.0, 0.5);
  o.hasVelocities = true;
  o.velocityLocal = {0.3, 0.0, 0.1};

  std_msgs::msg::Header h;
  h.frame_id = "odom";
  const auto msg = mrpt::ros2bridge::toROS(o, h, "base_link");
  EXPECT_EQ(msg.header.frame_id, "odom");
  EXPECT_EQ(msg.child_frame_id, "base_link");
  EXPECT_DOUBLE_EQ(msg.pose.pose.position.y, 2.0);
  EXPECT_NEAR(msg.pose.pose.orientation.z, std::sin(0.25), 1e-12);
  EXPECT_DOUBLE_EQ(msg.twist.twist.linear.x, 0.3);
  EXPECT_DOUBLE_EQ(msg.twist.twist.angular.z, 0.1);
}
