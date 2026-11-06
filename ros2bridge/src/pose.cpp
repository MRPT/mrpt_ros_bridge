/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2026, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

/*
 * pose.cpp
 *
 *  Created on: Mar 14, 2012
 *      Author: Pablo Iñigo Blasco
 *
 *      To understand better how this is implemented see the references:
 *      - http://www.mrpt.org/2D_3D_Geometry
 *
 * Aug 17, 2019: Refactored into mrpt::ros2bridge library for MRPT 2.0 (JLBC)
 *
 */

#include <mrpt/core/exceptions.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/poses/CPose3DPDFGaussianInf.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/poses/CPosePDFGaussianInf.h>
#include <mrpt/ros2bridge/pose.h>

#include <cmath>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>

namespace
{
// Jacobian of the PoseWithCovariance parameters, (x,y,z) plus a small rotation
// vector about the fixed axes of the parent frame (R = Exp(dtheta) * R0, the
// convention of tf2::doTransform()), with respect to MRPT (x,y,z,yaw,pitch,roll),
// R0 = Rz(yaw) * Ry(pitch) * Rx(roll). Each column of the rotation block is
// the parent-frame axis of one of the three elementary rotations.
mrpt::math::CMatrixDouble66 jacobian_ros_wrt_ypr(const mrpt::poses::CPose3D& p)
{
  const double cy = std::cos(p.yaw());
  const double sy = std::sin(p.yaw());
  const double cp = std::cos(p.pitch());
  const double sp = std::sin(p.pitch());

  mrpt::math::CMatrixDouble66 J;
  J.setIdentity();
  // clang-format off
  J(3, 3) = 0;  J(3, 4) = -sy;  J(3, 5) = cy * cp;
  J(4, 3) = 0;  J(4, 4) = cy;   J(4, 5) = sy * cp;
  J(5, 3) = 1;  J(5, 4) = 0;    J(5, 5) = -sp;
  // clang-format on
  return J;
}

// Inverse of jacobian_ros_wrt_ypr(), singular at pitch = +-90 deg (kept finite).
mrpt::math::CMatrixDouble66 jacobian_ypr_wrt_ros(const mrpt::poses::CPose3D& p)
{
  const double cy = std::cos(p.yaw());
  const double sy = std::sin(p.yaw());
  constexpr double MIN_COS_PITCH = 1e-9;
  double cp = std::cos(p.pitch());
  if (std::abs(cp) < MIN_COS_PITCH)
  {
    cp = std::copysign(MIN_COS_PITCH, cp);
  }
  const double tp = std::sin(p.pitch()) / cp;

  mrpt::math::CMatrixDouble66 J;
  J.setIdentity();
  // clang-format off
  J(3, 3) = tp * cy;  J(3, 4) = tp * sy;  J(3, 5) = 1;
  J(4, 3) = -sy;      J(4, 4) = cy;       J(4, 5) = 0;
  J(5, 3) = cy / cp;  J(5, 4) = sy / cp;  J(5, 5) = 0;
  // clang-format on
  return J;
}

// J * C * J^T
mrpt::math::CMatrixDouble66 sandwich(
    const mrpt::math::CMatrixDouble66& J, const mrpt::math::CMatrixDouble66& C)
{
  mrpt::math::CMatrixDouble66 R;
  for (int i = 0; i < 6; i++)
  {
    for (int j = 0; j < 6; j++)
    {
      double acc = 0;
      for (int k = 0; k < 6; k++)
      {
        for (int l = 0; l < 6; l++)
        {
          acc += J(i, k) * C(k, l) * J(j, l);
        }
      }
      R(i, j) = acc;
    }
  }
  return R;
}
}  // namespace

// MRPT -> ROS functions:
tf2::Matrix3x3 mrpt::ros2bridge::toROS(const mrpt::math::CMatrixDouble33& src)
{
  tf2::Matrix3x3 des;
  for (int r = 0; r < 3; r++)
    for (int c = 0; c < 3; c++) des[r][c] = src(r, c);
  return des;
}

tf2::Transform mrpt::ros2bridge::toROS_tfTransform(const mrpt::poses::CPose2D& src)
{
  return toROS_tfTransform(mrpt::poses::CPose3D(src));
}

geometry_msgs::msg::Pose mrpt::ros2bridge::toROS_Pose(const mrpt::poses::CPose2D& src)
{
  return toROS_Pose(mrpt::poses::CPose3D(src));
}

tf2::Transform mrpt::ros2bridge::toROS_tfTransform(const mrpt::math::TPose2D& src)
{
  return toROS_tfTransform(mrpt::poses::CPose3D(mrpt::math::TPose3D(src)));
}

geometry_msgs::msg::Pose mrpt::ros2bridge::toROS_Pose(const mrpt::math::TPose2D& src)
{
  geometry_msgs::msg::Pose des;

  des.position.x = src.x;
  des.position.y = src.y;
  des.position.z = 0;

  const double yaw = src.phi;
  if (std::abs(yaw) < 1e-10)
  {
    des.orientation.x = 0.;
    des.orientation.y = 0.;
    des.orientation.z = .5 * yaw;
    des.orientation.w = 1.;
  }
  else
  {
    const double s = ::sin(yaw * .5);
    const double c = ::cos(yaw * .5);
    des.orientation.x = 0.;
    des.orientation.y = 0.;
    des.orientation.z = s;
    des.orientation.w = c;
  }

  return des;
}

tf2::Transform mrpt::ros2bridge::toROS_tfTransform(const mrpt::poses::CPose3D& src)
{
  tf2::Transform des;
  des.setBasis(toROS(src.getRotationMatrix()));
  des.setOrigin(tf2::Vector3(src.x(), src.y(), src.z()));
  return des;
}

geometry_msgs::msg::Pose mrpt::ros2bridge::toROS_Pose(const mrpt::poses::CPose3D& src)
{
  geometry_msgs::msg::Pose des;
  des.position.x = src.x();
  des.position.y = src.y();
  des.position.z = src.z();

  mrpt::math::CQuaternionDouble q;
  src.getAsQuaternion(q);

  des.orientation.x = q.x();
  des.orientation.y = q.y();
  des.orientation.z = q.z();
  des.orientation.w = q.r();

  return des;
}

tf2::Transform mrpt::ros2bridge::toROS_tfTransform(const mrpt::math::TPose3D& src)
{
  return toROS_tfTransform(mrpt::poses::CPose3D(src));
}

geometry_msgs::msg::Pose mrpt::ros2bridge::toROS_Pose(const mrpt::math::TPose3D& src)
{
  return toROS_Pose(mrpt::poses::CPose3D(src));
}

geometry_msgs::msg::PoseWithCovariance mrpt::ros2bridge::toROS_Pose(
    const mrpt::poses::CPose3DPDFGaussian& src)
{
  geometry_msgs::msg::PoseWithCovariance des;
  des.pose = toROS_Pose(src.mean);

  // geometry_msgs/PoseWithCovariance: (x, y, z, rotation about X axis,
  // rotation about Y axis, rotation about Z axis), the rotations being a small
  // rotation vector about the fixed axes of the parent frame. MRPT uses
  // (x,y,z,yaw,pitch,roll): both only coincide for yaw = pitch = 0.
  const auto J = jacobian_ros_wrt_ypr(src.mean);
  const mrpt::math::CMatrixDouble66 cov = sandwich(J, src.cov);
  for (int i = 0; i < 6; i++)
  {
    for (int j = 0; j < 6; j++)
    {
      des.covariance[static_cast<size_t>(i * 6 + j)] = cov(i, j);
    }
  }
  return des;
}

geometry_msgs::msg::PoseWithCovariance mrpt::ros2bridge::toROS(
    const mrpt::poses::CPose3DPDFGaussianInf& src)
{
  mrpt::poses::CPose3DPDFGaussian src2;
  src2.copyFrom(src);
  return toROS_Pose(src2);
}

geometry_msgs::msg::PoseWithCovariance mrpt::ros2bridge::toROS(
    const mrpt::poses::CPosePDFGaussian& src)
{
  geometry_msgs::msg::PoseWithCovariance des;

  des.pose = toROS_Pose(src.mean);

  // A planar pose only rotates about the parent Z axis, so its yaw is exactly
  // the "rotation about Z axis" parameter of the message.

  // geometry_msgs/PoseWithCovariance msg stores the covariance matrix in
  // row-major representation
  // Indexes are :
  // [ 0   1   2   3   4   5  ]
  // [ 6   7   8   9   10  11 ]
  // [ 12  13  14  15  16  17 ]
  // [ 18  19  20  21  22  23 ]
  // [ 24  25  26  27  28  29 ]
  // [ 30  31  32  33  34  35 ]

  des.covariance[0] = src.cov(0, 0);
  des.covariance[1] = src.cov(0, 1);
  des.covariance[5] = src.cov(0, 2);
  des.covariance[6] = src.cov(1, 0);
  des.covariance[7] = src.cov(1, 1);
  des.covariance[11] = src.cov(1, 2);
  des.covariance[30] = src.cov(2, 0);
  des.covariance[31] = src.cov(2, 1);
  des.covariance[35] = src.cov(2, 2);

  return des;
}

geometry_msgs::msg::PoseWithCovariance mrpt::ros2bridge::toROS(
    const mrpt::poses::CPosePDFGaussianInf& src)
{
  mrpt::poses::CPosePDFGaussian src2;
  src2.copyFrom(src);

  return toROS(src2);
}

geometry_msgs::msg::Quaternion mrpt::ros2bridge::toROS(const mrpt::math::CQuaternionDouble& src)
{
  geometry_msgs::msg::Quaternion des;
  des.x = src.x();
  des.y = src.y();
  des.z = src.z();
  des.w = src.r();
  return des;
}

// ROS -> MRPT functions:
mrpt::poses::CPose3D mrpt::ros2bridge::fromROS(const tf2::Transform& src)
{
  mrpt::poses::CPose3D des;
  const tf2::Vector3& t = src.getOrigin();
  des.x(t[0]);
  des.y(t[1]);
  des.z(t[2]);
  des.setRotationMatrix(fromROS(src.getBasis()));
  return des;
}
mrpt::math::CMatrixDouble33 mrpt::ros2bridge::fromROS(const tf2::Matrix3x3& src)
{
  mrpt::math::CMatrixDouble33 des;
  for (int r = 0; r < 3; r++)
    for (int c = 0; c < 3; c++) des(r, c) = src[r][c];
  return des;
}

mrpt::poses::CPose3D mrpt::ros2bridge::fromROS(const geometry_msgs::msg::Pose& src)
{
  const mrpt::math::CQuaternionDouble q(
      src.orientation.w, src.orientation.x, src.orientation.y, src.orientation.z);
  return mrpt::poses::CPose3D(q, src.position.x, src.position.y, src.position.z);
}

mrpt::poses::CPose3DPDFGaussian mrpt::ros2bridge::fromROS(
    const geometry_msgs::msg::PoseWithCovariance& src)
{
  mrpt::poses::CPose3DPDFGaussian des;

  des.mean = fromROS(src.pose);

  // See toROS_Pose(): from parent-frame rotation vectors to yaw/pitch/roll.
  mrpt::math::CMatrixDouble66 cov;
  for (int i = 0; i < 6; i++)
  {
    for (int j = 0; j < 6; j++)
    {
      cov(i, j) = src.covariance[static_cast<size_t>(i * 6 + j)];
    }
  }
  des.cov = sandwich(jacobian_ypr_wrt_ros(des.mean), cov);

  return des;
}

mrpt::math::CQuaternionDouble mrpt::ros2bridge::fromROS(const geometry_msgs::msg::Quaternion& src)
{
  mrpt::math::CQuaternionDouble des;
  des.x(src.x);
  des.y(src.y);
  des.z(src.z);
  des.r(src.w);
  return des;
}
