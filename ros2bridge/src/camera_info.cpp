/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2026, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/ros2bridge/camera_info.h>

#include <sensor_msgs/distortion_models.hpp>

sensor_msgs::msg::CameraInfo mrpt::ros2bridge::toROS(
    const mrpt::img::TCamera& cam, const std_msgs::msg::Header& msg_header)
{
  using mrpt::img::DistortionModel;
  namespace dm = sensor_msgs::distortion_models;

  sensor_msgs::msg::CameraInfo ci;
  ci.header = msg_header;
  ci.width = cam.ncols;
  ci.height = cam.nrows;

  switch (cam.distortion)
  {
    case DistortionModel::kannala_brandt:
      ci.distortion_model = dm::EQUIDISTANT;
      ci.d = {cam.k1(), cam.k2(), cam.k3(), cam.k4()};
      break;
    case DistortionModel::plumb_bob:
      if (cam.k4() != 0 || cam.k5() != 0 || cam.k6() != 0)
      {
        ci.distortion_model = dm::RATIONAL_POLYNOMIAL;
        ci.d.assign(cam.dist.begin(), cam.dist.begin() + 8);
      }
      else
      {
        ci.distortion_model = dm::PLUMB_BOB;
        ci.d.assign(cam.dist.begin(), cam.dist.begin() + 5);
      }
      break;
    case DistortionModel::none:
    default:
      ci.distortion_model = dm::PLUMB_BOB;
      ci.d.assign(5, 0.0);
      break;
  }

  ci.k.fill(0);
  ci.k[0] = cam.fx();
  ci.k[2] = cam.cx();
  ci.k[4] = cam.fy();
  ci.k[5] = cam.cy();
  ci.k[8] = 1.0;

  ci.r.fill(0);
  ci.r[0] = 1.0;
  ci.r[4] = 1.0;
  ci.r[8] = 1.0;

  ci.p.fill(0);
  ci.p[0] = cam.fx();
  ci.p[2] = cam.cx();
  ci.p[5] = cam.fy();
  ci.p[6] = cam.cy();
  ci.p[10] = 1.0;

  return ci;
}

mrpt::img::TCamera mrpt::ros2bridge::fromROS(const sensor_msgs::msg::CameraInfo& info)
{
  namespace dm = sensor_msgs::distortion_models;

  mrpt::img::TCamera cam;
  cam.ncols = info.width;
  cam.nrows = info.height;
  cam.setIntrinsicParamsFromValues(info.k[0], info.k[4], info.k[2], info.k[5]);

  const auto d = [&info](size_t i) { return i < info.d.size() ? info.d[i] : 0.0; };

  if (info.distortion_model == dm::PLUMB_BOB || info.distortion_model == dm::RATIONAL_POLYNOMIAL)
  {
    cam.setDistortionPlumbBob(d(0), d(1), d(2), d(3), d(4));
    cam.k4(d(5));
    cam.k5(d(6));
    cam.k6(d(7));
  }
  else if (info.distortion_model == dm::EQUIDISTANT || info.distortion_model == "kannala_brandt")
  {
    cam.setDistortionKannalaBrandt(d(0), d(1), d(2), d(3));
  }
  else
  {
    cam.distortion = mrpt::img::DistortionModel::none;
  }
  return cam;
}
