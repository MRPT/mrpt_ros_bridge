/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2026, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

/*---------------------------------------------------------------
  APPLICATION: mrpt_ros bridge
  FILE: imu.cpp
  AUTHOR: Raghavender Sahdev <raghavendersahdev@gmail.com>
  ---------------------------------------------------------------*/

#include <mrpt/poses/CPose3D.h>
#include <mrpt/ros2bridge/imu.h>
#include <mrpt/ros2bridge/time.h>

#include <array>

bool mrpt::ros2bridge::fromROS(const sensor_msgs::msg::Imu& msg, mrpt::obs::CObservationIMU& obj)
{
  using namespace mrpt::obs;

  obj.timestamp = mrpt::ros2bridge::fromROS(msg.header.stamp);

  if (msg.orientation_covariance.at(0) >= 0)
  {
    obj.set(IMU_ORI_QUAT_X, msg.orientation.x);
    obj.set(IMU_ORI_QUAT_Y, msg.orientation.y);
    obj.set(IMU_ORI_QUAT_Z, msg.orientation.z);
    obj.set(IMU_ORI_QUAT_W, msg.orientation.w);
  }

  if (msg.linear_acceleration_covariance.at(0) >= 0)
  {
    obj.set(IMU_X_ACC, msg.linear_acceleration.x);
    obj.set(IMU_Y_ACC, msg.linear_acceleration.y);
    obj.set(IMU_Z_ACC, msg.linear_acceleration.z);
  }

  if (msg.angular_velocity_covariance.at(0) >= 0)
  {
    obj.set(IMU_WX, msg.angular_velocity.x);
    obj.set(IMU_WY, msg.angular_velocity.y);
    obj.set(IMU_WZ, msg.angular_velocity.z);
  }

  return true;
}

namespace
{
// Diagonal covariance with the given variance. ROS uses -1 in the first
// element to flag an unavailable field.
void setDiagonalCovariance(std::array<double, 9>& cov, double variance)
{
  cov.fill(0);
  cov[0] = variance;
  cov[4] = variance;
  cov[8] = variance;
}

void setUnavailableCovariance(std::array<double, 9>& cov)
{
  cov.fill(0);
  cov[0] = -1;
}
}  // namespace

bool mrpt::ros2bridge::toROS(
    const mrpt::obs::CObservationIMU& obj,
    const std_msgs::msg::Header& msg_header,
    sensor_msgs::msg::Imu& msg)
{
  using namespace mrpt::obs;

  // Nominal variance, since CObservationIMU carries no covariance:
  constexpr double nominalVariance = 0.01;

  msg.header = msg_header;

  const auto& m = obj.rawMeasurements;
  if (obj.has(IMU_ORI_QUAT_X))
  {
    msg.orientation.x = m.at(IMU_ORI_QUAT_X);
    msg.orientation.y = m.at(IMU_ORI_QUAT_Y);
    msg.orientation.z = m.at(IMU_ORI_QUAT_Z);
    msg.orientation.w = m.at(IMU_ORI_QUAT_W);
    setDiagonalCovariance(msg.orientation_covariance, nominalVariance);
  }
  else if (obj.has(IMU_YAW) && obj.has(IMU_PITCH) && obj.has(IMU_ROLL))
  {
    mrpt::math::CQuaternionDouble q;
    mrpt::poses::CPose3D::FromYawPitchRoll(m.at(IMU_YAW), m.at(IMU_PITCH), m.at(IMU_ROLL))
        .getAsQuaternion(q);
    msg.orientation.x = q.x();
    msg.orientation.y = q.y();
    msg.orientation.z = q.z();
    msg.orientation.w = q.r();
    setDiagonalCovariance(msg.orientation_covariance, nominalVariance);
  }
  else
  {
    setUnavailableCovariance(msg.orientation_covariance);
  }

  if (obj.has(IMU_X_ACC))
  {
    msg.linear_acceleration.x = m.at(IMU_X_ACC);
    msg.linear_acceleration.y = m.at(IMU_Y_ACC);
    msg.linear_acceleration.z = m.at(IMU_Z_ACC);
    setDiagonalCovariance(msg.linear_acceleration_covariance, nominalVariance);
  }
  else
  {
    setUnavailableCovariance(msg.linear_acceleration_covariance);
  }

  if (obj.has(IMU_WX))
  {
    msg.angular_velocity.x = m.at(IMU_WX);
    msg.angular_velocity.y = m.at(IMU_WY);
    msg.angular_velocity.z = m.at(IMU_WZ);
    setDiagonalCovariance(msg.angular_velocity_covariance, nominalVariance);
  }
  else
  {
    setUnavailableCovariance(msg.angular_velocity_covariance);
  }

  return true;
}

/*
std_msgs/Header header
geometry_msgs/Quaternion orientation
float64[9] orientation_covariance
geometry_msgs/Vector3 angular_velocity
float64[9] angular_velocity_covariance
geometry_msgs/Vector3 linear_acceleration
float64[9] linear_acceleration_covariance
 */
