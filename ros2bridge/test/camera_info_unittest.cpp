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
  EXPECT_EQ(
      mrpt::ros2bridge::toROS(c, std_msgs::msg::Header()).distortion_model, "rational_polynomial");
  EXPECT_DOUBLE_EQ(roundTrip(c).k6(), 0.6);

  c.setDistortionKannalaBrandt(0.1, 0.2, 0.3, 0.4);
  const auto ci = mrpt::ros2bridge::toROS(c, std_msgs::msg::Header());
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

TEST(CameraInfo, RectifiedRoiAndBinning)
{
  sensor_msgs::msg::CameraInfo ci;
  ci.width = 1280;
  ci.height = 720;
  ci.distortion_model = "plumb_bob";
  ci.d = {0, 0, 0, 0, 0};
  // Rectified images: P, not K, describes them.
  ci.k = {700, 0, 640, 0, 700, 360, 0, 0, 1};
  ci.p = {650, 0, 630, 0, 0, 650, 350, 0, 0, 0, 1, 0};
  ci.roi.x_offset = 100;
  ci.roi.y_offset = 50;
  ci.roi.width = 800;
  ci.roi.height = 600;
  ci.binning_x = 2;
  ci.binning_y = 2;

  const auto cam = mrpt::ros2bridge::fromROS(ci, true);
  EXPECT_EQ(cam.ncols, 400U);
  EXPECT_EQ(cam.nrows, 300U);
  EXPECT_DOUBLE_EQ(cam.fx(), 325.0);
  EXPECT_DOUBLE_EQ(cam.fy(), 325.0);
  EXPECT_DOUBLE_EQ(cam.cx(), (630.0 - 100.0) / 2);
  EXPECT_DOUBLE_EQ(cam.cy(), (350.0 - 50.0) / 2);

  // Raw images are described by K, even without distortion:
  ci.roi = sensor_msgs::msg::RegionOfInterest();
  ci.binning_x = 0;
  ci.binning_y = 0;
  const auto rawNoDistortion = mrpt::ros2bridge::fromROS(ci);
  EXPECT_EQ(rawNoDistortion.ncols, 1280U);
  EXPECT_DOUBLE_EQ(rawNoDistortion.fx(), 700.0);
  EXPECT_DOUBLE_EQ(rawNoDistortion.cx(), 640.0);

  ci.d[0] = 0.1;
  const auto raw = mrpt::ros2bridge::fromROS(ci);
  EXPECT_DOUBLE_EQ(raw.fx(), 700.0);
  EXPECT_DOUBLE_EQ(raw.k1(), 0.1);

  // Rectified images of a camera with distortion: P, and no distortion left.
  const auto rect = mrpt::ros2bridge::fromROS(ci, true);
  EXPECT_DOUBLE_EQ(rect.fx(), 650.0);
  EXPECT_DOUBLE_EQ(rect.cx(), 630.0);
  EXPECT_EQ(rect.distortion, mrpt::img::DistortionModel::none);
}
