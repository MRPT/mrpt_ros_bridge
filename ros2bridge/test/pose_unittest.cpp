/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2026, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

/*
 * test_pose.cpp
 *
 *  Created on: Mar 15, 2012
 *      Author: Pablo Iñigo Blasco
 */

#include <gtest/gtest.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/poses/Lie/SO.h>
#include <mrpt/ros2bridge/pose.h>

#include <Eigen/Dense>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std;

TEST(PoseConversions, copyMatrix3x3ToCMatrixDouble33)
{
  tf2::Matrix3x3 src(0, 1, 2, 3, 4, 5, 6, 7, 8);
  const mrpt::math::CMatrixDouble33 des = mrpt::ros2bridge::fromROS(src);

  for (int r = 0; r < 3; r++)
    for (int c = 0; c < 3; c++) EXPECT_FLOAT_EQ(des(r, c), src[r][c]);
}
TEST(PoseConversions, copyCMatrixDouble33ToMatrix3x3)
{
  mrpt::math::CMatrixDouble33 src;
  for (int r = 0, i = 0; r < 3; r++)
    for (int c = 0; c < 3; c++, i++) src(r, c) = i;

  const tf2::Matrix3x3 des = mrpt::ros2bridge::toROS(src);

  for (int r = 0; r < 3; r++)
    for (int c = 0; c < 3; c++) EXPECT_FLOAT_EQ(des[r][c], src(r, c));
}

// Declare a test
TEST(PoseConversions, reference_frame_change_with_rotations)
{
  geometry_msgs::msg::PoseWithCovariance ros_msg_original_pose;

  ros_msg_original_pose.pose.position.x = 1;
  ros_msg_original_pose.pose.position.y = 0;
  ros_msg_original_pose.pose.position.z = 0;
  ros_msg_original_pose.pose.orientation.x = 0;
  ros_msg_original_pose.pose.orientation.y = 0;
  ros_msg_original_pose.pose.orientation.z = 0;
  ros_msg_original_pose.pose.orientation.w = 1;

  // to mrpt
  mrpt::poses::CPose3DPDFGaussian mrpt_original_pose =
      mrpt::ros2bridge::fromROS(ros_msg_original_pose);

  EXPECT_EQ(ros_msg_original_pose.pose.position.x, mrpt_original_pose.mean[0]);

#if 0
	// to tf
	tf::Pose tf_original_pose;
	tf::poseMsgToTF(ros_msg_original_pose.pose, tf_original_pose);

	// rotate yaw pi in MRPT
	mrpt::poses::CPose3D rotation_mrpt;
	double yaw = M_PI / 2.0;
	rotation_mrpt.setFromValues(0, 0, 0, yaw, 0, 0);
	mrpt::poses::CPose3D mrpt_result = rotation_mrpt + mrpt_original_pose.mean;
	EXPECT_NEAR(mrpt_result[1], 1.0, 0.01);

	// rotate yaw pi in TF
	tf::Quaternion rotation_tf;
	rotation_tf.setRPY(0, 0, yaw);
	tf::Pose rotation_pose_tf;
	rotation_pose_tf.setIdentity();
	rotation_pose_tf.setRotation(rotation_tf);
	tf::Pose tf_result = rotation_pose_tf * tf_original_pose;
	EXPECT_NEAR(tf_result.getOrigin()[1], 1.0, 0.01);

	geometry_msgs::msg::Pose mrpt_ros_result;
	mrpt_bridge::convert(mrpt_result, mrpt_ros_result);

	EXPECT_NEAR(mrpt_ros_result.position.x, tf_result.getOrigin()[0], 0.01);
	EXPECT_NEAR(mrpt_ros_result.position.y, tf_result.getOrigin()[1], 0.01);
	EXPECT_NEAR(mrpt_ros_result.position.z, tf_result.getOrigin()[2], 0.01);
#endif
}

void check_CPose3D_tofrom_ROS(double x, double y, double z, double yaw, double pitch, double roll)
{
  const mrpt::poses::CPose3D p3D(x, y, z, yaw, pitch, roll);

  // Convert MRPT->ROS
  geometry_msgs::msg::Pose ros_p3D = mrpt::ros2bridge::toROS_Pose(p3D);

  // Compare ROS quat vs. MRPT quat:
  mrpt::math::CQuaternionDouble q;
  p3D.getAsQuaternion(q);

  EXPECT_NEAR(ros_p3D.position.x, p3D.x(), 1e-4) << "p: " << p3D << endl;
  EXPECT_NEAR(ros_p3D.position.y, p3D.y(), 1e-4) << "p: " << p3D << endl;
  EXPECT_NEAR(ros_p3D.position.z, p3D.z(), 1e-4) << "p: " << p3D << endl;

  EXPECT_NEAR(ros_p3D.orientation.x, q.x(), 1e-4) << "p: " << p3D << endl;
  EXPECT_NEAR(ros_p3D.orientation.y, q.y(), 1e-4) << "p: " << p3D << endl;
  EXPECT_NEAR(ros_p3D.orientation.z, q.z(), 1e-4) << "p: " << p3D << endl;
  EXPECT_NEAR(ros_p3D.orientation.w, q.r(), 1e-4) << "p: " << p3D << endl;

  // Test the other path: ROS->MRPT
  mrpt::poses::CPose3D p_bis = mrpt::ros2bridge::fromROS(ros_p3D);

  // p_bis==p3D?
  EXPECT_NEAR((p_bis.asVectorVal() - p3D.asVectorVal()).array().abs().maxCoeff(), 0, 1e-4)
      << "p_bis: " << p_bis << endl
      << "p3D: " << p3D << endl;
}

// Declare a test
TEST(PoseConversions, check_CPose3D_tofrom_ROS)
{
  using mrpt::DEG2RAD;
  using mrpt::RAD2DEG;
  using namespace mrpt;  // for 0.0_deg

  check_CPose3D_tofrom_ROS(0, 0, 0, 0.0_deg, 0.0_deg, 0.0_deg);
  check_CPose3D_tofrom_ROS(1, 2, 3, 0.0_deg, 0.0_deg, 0.0_deg);

  check_CPose3D_tofrom_ROS(1, 2, 3, 30.0_deg, 0.0_deg, 0.0_deg);
  check_CPose3D_tofrom_ROS(1, 2, 3, 0.0_deg, 30.0_deg, 0.0_deg);
  check_CPose3D_tofrom_ROS(1, 2, 3, 0.0_deg, 0.0_deg, 30.0_deg);

  check_CPose3D_tofrom_ROS(1, 2, 3, -5.0_deg, 15.0_deg, -30.0_deg);
}

// Declare a test
TEST(PoseConversions, check_CPose2D_to_ROS)
{
  const mrpt::poses::CPose2D p2D(1, 2, 0.56);

  // Convert MRPT->ROS
  const geometry_msgs::msg::Pose ros_p2D = mrpt::ros2bridge::toROS_Pose(p2D);

  // Compare vs. 3D pose:
  const mrpt::poses::CPose3D p3D = mrpt::poses::CPose3D(p2D);
  const mrpt::poses::CPose3D p3D_ros = mrpt::ros2bridge::fromROS(ros_p2D);

  // p3D_ros should equal p3D
  EXPECT_NEAR((p3D_ros.asVectorVal() - p3D.asVectorVal()).array().abs().maxCoeff(), 0, 1e-4)
      << "p3D_ros: " << p3D_ros << endl
      << "p3D: " << p3D << endl;
}

namespace
{
mrpt::poses::CPose3DPDFGaussian make_pose_pdf(
    double x, double y, double z, double yaw, double pitch, double roll)
{
  mrpt::poses::CPose3DPDFGaussian pdf;
  pdf.mean = mrpt::poses::CPose3D(x, y, z, yaw, pitch, roll);
  Eigen::Matrix<double, 6, 6> A;
  for (int i = 0; i < 6; i++)
  {
    for (int j = 0; j < 6; j++)
    {
      A(i, j) = std::sin(1.0 + i * 7.0 + j * 3.0);
    }
  }
  pdf.cov.asEigen() = 0.01 * (A * A.transpose() + Eigen::Matrix<double, 6, 6>::Identity());
  return pdf;
}

Eigen::Matrix<double, 6, 6> ros_cov(const geometry_msgs::msg::PoseWithCovariance& m)
{
  Eigen::Matrix<double, 6, 6> C;
  for (int i = 0; i < 6; i++)
  {
    for (int j = 0; j < 6; j++)
    {
      C(i, j) = m.covariance[static_cast<size_t>(i * 6 + j)];
    }
  }
  return C;
}

const std::vector<std::array<double, 3>> test_angles_deg = {
    {   0,  0,  0},
    {  90,  0,  0},
    {  30,  5, -3},
    { 170, 40, 60},
    {-120, 80, 10}
};
}  // namespace

// The rotation covariance of the message is that of a small rotation vector
// about the parent frame axes: R = Exp(dtheta) * R0. Checked against finite
// differences of the yaw/pitch/roll parameterization.
TEST(PoseConversions, PoseWithCovariance_rotation_is_parent_frame_vector)
{
  for (const auto& a : test_angles_deg)
  {
    const auto pdf =
        make_pose_pdf(1, 2, 3, mrpt::DEG2RAD(a[0]), mrpt::DEG2RAD(a[1]), mrpt::DEG2RAD(a[2]));
    const auto R0 = pdf.mean.getRotationMatrix();

    Eigen::Matrix<double, 6, 6> J = Eigen::Matrix<double, 6, 6>::Identity();
    const double h = 1e-6;
    for (int k = 3; k < 6; k++)
    {
      mrpt::math::CVectorFixedDouble<6> vp;
      mrpt::math::CVectorFixedDouble<6> vm;
      pdf.mean.asVector(vp);
      pdf.mean.asVector(vm);
      vp[k] += h;
      vm[k] -= h;
      const auto Rp =
          mrpt::poses::CPose3D::FromXYZYawPitchRoll(vp[0], vp[1], vp[2], vp[3], vp[4], vp[5])
              .getRotationMatrix();
      const auto Rm =
          mrpt::poses::CPose3D::FromXYZYawPitchRoll(vm[0], vm[1], vm[2], vm[3], vm[4], vm[5])
              .getRotationMatrix();
      const mrpt::math::CMatrixDouble33 dRp(Rp.asEigen() * R0.asEigen().transpose());
      const mrpt::math::CMatrixDouble33 dRm(Rm.asEigen() * R0.asEigen().transpose());
      const auto wp = mrpt::poses::Lie::SO<3>::log(dRp);
      const auto wm = mrpt::poses::Lie::SO<3>::log(dRm);
      for (int r = 0; r < 3; r++)
      {
        J(3 + r, k) = (wp[r] - wm[r]) / (2 * h);
      }
    }
    const Eigen::Matrix<double, 6, 6> expected = J * pdf.cov.asEigen() * J.transpose();

    const auto msg = mrpt::ros2bridge::toROS_Pose(pdf);
    EXPECT_NEAR((ros_cov(msg) - expected).cwiseAbs().maxCoeff(), 0.0, 1e-8)
        << "yaw,pitch,roll (deg): " << a[0] << " " << a[1] << " " << a[2];

    // Round trip:
    const auto back = mrpt::ros2bridge::fromROS(msg);
    EXPECT_NEAR((back.cov.asEigen() - pdf.cov.asEigen()).cwiseAbs().maxCoeff(), 0.0, 1e-9);
  }
}

namespace
{
// Covariance of T (+) p in yaw/pitch/roll, propagated with the numerical
// Jacobian of the pose composition with respect to p.
Eigen::Matrix<double, 6, 6> composed_cov_numerical(
    const mrpt::poses::CPose3D& T, const mrpt::poses::CPose3DPDFGaussian& pdf)
{
  Eigen::Matrix<double, 6, 6> J;
  const double h = 1e-6;
  for (int k = 0; k < 6; k++)
  {
    mrpt::math::CVectorFixedDouble<6> vp;
    mrpt::math::CVectorFixedDouble<6> vm;
    pdf.mean.asVector(vp);
    pdf.mean.asVector(vm);
    vp[k] += h;
    vm[k] -= h;
    mrpt::math::CVectorFixedDouble<6> rp;
    mrpt::math::CVectorFixedDouble<6> rm;
    (T + mrpt::poses::CPose3D::FromXYZYawPitchRoll(vp[0], vp[1], vp[2], vp[3], vp[4], vp[5]))
        .asVector(rp);
    (T + mrpt::poses::CPose3D::FromXYZYawPitchRoll(vm[0], vm[1], vm[2], vm[3], vm[4], vm[5]))
        .asVector(rm);
    for (int r = 0; r < 6; r++)
    {
      double d = rp[r] - rm[r];
      if (r >= 3)
      {
        d = mrpt::math::wrapToPi(d);
      }
      J(r, k) = d / (2 * h);
    }
  }
  return J * pdf.cov.asEigen() * J.transpose();
}
}  // namespace

// Changing the parent frame with tf2 must match the first-order propagation
// of the MRPT covariance through the pose composition.
TEST(PoseConversions, PoseWithCovariance_consistent_with_tf2_transform)
{
  const mrpt::poses::CPose3D T(
      -4.0, 1.0, 0.5, mrpt::DEG2RAD(50.0), mrpt::DEG2RAD(-20.0), mrpt::DEG2RAD(35.0));
  const tf2::Transform tfT = mrpt::ros2bridge::toROS_tfTransform(T);

  for (const auto& a : test_angles_deg)
  {
    const auto pdf =
        make_pose_pdf(1, 2, 3, mrpt::DEG2RAD(a[0]), mrpt::DEG2RAD(a[1]), mrpt::DEG2RAD(a[2]));

    const Eigen::Matrix<double, 6, 6> expected = composed_cov_numerical(T, pdf);

    const auto msg = mrpt::ros2bridge::toROS_Pose(pdf);
    geometry_msgs::msg::PoseWithCovariance msgT;
    msgT.covariance = tf2::transformCovariance(msg.covariance, tfT);
    msgT.pose = mrpt::ros2bridge::toROS_Pose(T + pdf.mean);
    const auto rosPath = mrpt::ros2bridge::fromROS(msgT);

    EXPECT_NEAR(
        (rosPath.cov.asEigen() - expected).cwiseAbs().maxCoeff(), 0.0,
        1e-6 * expected.cwiseAbs().maxCoeff())
        << "yaw,pitch,roll (deg): " << a[0] << " " << a[1] << " " << a[2];
  }
}

// For a planar pose, the yaw variance is the "rotation about Z" one.
TEST(PoseConversions, PoseWithCovariance_planar_yaw_variance)
{
  mrpt::poses::CPose3DPDFGaussian pdf;
  pdf.mean = mrpt::poses::CPose3D(1, 2, 0, mrpt::DEG2RAD(120.0), 0, 0);
  pdf.cov.setZero();
  pdf.cov(0, 0) = 0.25;
  pdf.cov(1, 1) = 0.04;
  pdf.cov(2, 2) = 0.01;
  pdf.cov(3, 3) = 0.09;  // yaw
  pdf.cov(4, 4) = 1e-4;  // pitch
  pdf.cov(5, 5) = 4e-4;  // roll

  const auto msg = mrpt::ros2bridge::toROS_Pose(pdf);
  EXPECT_NEAR(msg.covariance[0], 0.25, 1e-12);
  EXPECT_NEAR(msg.covariance[7], 0.04, 1e-12);
  EXPECT_NEAR(msg.covariance[35], 0.09, 1e-12);
  // Roll/pitch variances are rotated by the yaw into the parent frame axes:
  const double c = std::cos(mrpt::DEG2RAD(120.0));
  const double s = std::sin(mrpt::DEG2RAD(120.0));
  EXPECT_NEAR(msg.covariance[21], c * c * 4e-4 + s * s * 1e-4, 1e-12);
  EXPECT_NEAR(msg.covariance[28], s * s * 4e-4 + c * c * 1e-4, 1e-12);
}
