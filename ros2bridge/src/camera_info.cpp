/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2026, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/ros2bridge/camera_info.h>

#include <algorithm>
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

mrpt::img::TCamera mrpt::ros2bridge::fromROS(
    const sensor_msgs::msg::CameraInfo& info, bool rectified)
{
  namespace dm = sensor_msgs::distortion_models;

  const auto d = [&info](size_t i) { return i < info.d.size() ? info.d[i] : 0.0; };

  // A raw image is described by K and D, a rectified one by P (and no distortion).
  double fx = rectified ? info.p[0] : info.k[0];
  double fy = rectified ? info.p[5] : info.k[4];
  double cx = rectified ? info.p[2] : info.k[2];
  double cy = rectified ? info.p[6] : info.k[5];

  // The calibration is for the full sensor resolution: apply the ROI, then the binning.
  uint32_t width = info.width;
  uint32_t height = info.height;
  if (info.roi.width > 0 && info.roi.height > 0)
  {
    cx -= info.roi.x_offset;
    cy -= info.roi.y_offset;
    width = info.roi.width;
    height = info.roi.height;
  }
  const uint32_t bx = std::max<uint32_t>(info.binning_x, 1);
  const uint32_t by = std::max<uint32_t>(info.binning_y, 1);
  fx /= bx;
  cx /= bx;
  fy /= by;
  cy /= by;

  mrpt::img::TCamera cam;
  cam.ncols = width / bx;
  cam.nrows = height / by;
  cam.setIntrinsicParamsFromValues(fx, fy, cx, cy);

  if (rectified)
  {
    cam.distortion = mrpt::img::DistortionModel::none;
  }
  else if (
      info.distortion_model == dm::PLUMB_BOB || info.distortion_model == dm::RATIONAL_POLYNOMIAL)
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
