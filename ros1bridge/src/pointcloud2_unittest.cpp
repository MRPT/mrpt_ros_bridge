/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

/*
 * test_pose_conversions.cpp
 *
 *  Created on: Mar 15, 2012
 *      Author: Pablo Iñigo Blasco
 */

#include <gtest/gtest.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/ros1bridge/point_cloud2.h>
#include <mrpt/version.h>

#include <cstring>

// The per-point-field tests below need CGenericPointsMap, introduced in
// MRPT 2.15.0; ROS 1 Noetic still builds against older MRPT.
#if MRPT_VERSION >= 0x20f00
#include <mrpt/maps/CGenericPointsMap.h>
#endif

#if HAVE_PCL
#include <pcl/common/common_headers.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>

TEST(PointCloud2, basicTest)
{
  pcl::PointCloud<pcl::PointXYZ> point_cloud;

  point_cloud.height = 10;
  point_cloud.width = 10;
  point_cloud.is_dense = true;

  int num_points = point_cloud.height * point_cloud.width;
  point_cloud.points.resize(num_points);

  float i_f = 0;
  for (int i = 0; i < num_points; i++)
  {
    pcl::PointXYZ& point = point_cloud.points[i];
    point.x = i_f;
    point.y = -i_f;
    point.z = -2 * i_f;
    i_f += 1.0;
  }

  sensor_msgs::PointCloud2 point_cloud2_msg;
  pcl::toROSMsg(point_cloud, point_cloud2_msg);

  mrpt::maps::CSimplePointsMap mrpt_pc;

  mrpt::ros1bridge::fromROS(point_cloud2_msg, mrpt_pc);

  i_f = 0;
  for (int i = 0; i < num_points; i++)
  {
    float mrpt_x, mrpt_y, mrpt_z;
    mrpt_pc.getPoint(i, mrpt_x, mrpt_y, mrpt_z);
    EXPECT_FLOAT_EQ(mrpt_x, i_f);
    EXPECT_FLOAT_EQ(mrpt_y, -i_f);
    EXPECT_FLOAT_EQ(mrpt_z, -2 * i_f);

    i_f += 1.0;
  }
}
#endif  // HAVE_PCL

TEST(PointCloud2, toROS)
{
  mrpt::maps::CSimplePointsMap pc1;

  const size_t num_points = 1000;
  pc1.resize(num_points);

  float i_f = 0;
  for (size_t i = 0; i < num_points; i++)
  {
    pc1.setPoint(i, i_f, -i_f, -2 * i_f);
    i_f += 1.0;
  }

  sensor_msgs::PointCloud2 pc_msg;
  std_msgs::Header hdr;
  hdr.frame_id = "map";
  bool ok = mrpt::ros1bridge::toROS(pc1, hdr, pc_msg);
  ASSERT_(ok);

  EXPECT_EQ(pc_msg.header.frame_id, hdr.frame_id);

  //
  mrpt::maps::CSimplePointsMap pc2;
  bool ok2 = mrpt::ros1bridge::fromROS(pc_msg, pc2);
  ASSERT_(ok2);

  EXPECT_EQ(pc1.size(), pc2.size());
  for (size_t i = 0; i < pc1.size(); i++)
  {
    mrpt::math::TPoint3D pt1, pt2;
    pc1.getPoint(i, pt1);
    pc2.getPoint(i, pt2);
    EXPECT_TRUE(pt1 == pt2);
  }
}

#if MRPT_VERSION >= 0x20f00  // CGenericPointsMap

namespace
{
/** Builds a minimal XYZ + per-point-time cloud by hand, so the time field's
 * datatype and its values are exactly what the test wants to exercise.
 */
sensor_msgs::PointCloud2 makeCloudWithTimeField(
    uint8_t timeDatatype, const std::vector<double>& times)
{
  const size_t n = times.size();
  const bool isF64 = (timeDatatype == sensor_msgs::PointField::FLOAT64);
  const uint32_t tSize = isF64 ? 8 : 4;

  sensor_msgs::PointCloud2 msg;
  msg.height = 1;
  msg.width = static_cast<uint32_t>(n);
  msg.is_dense = true;
  msg.is_bigendian = false;

  const char* names[3] = {"x", "y", "z"};
  for (int i = 0; i < 3; i++)
  {
    sensor_msgs::PointField f;
    f.name = names[i];
    f.offset = static_cast<uint32_t>(4 * i);
    f.datatype = sensor_msgs::PointField::FLOAT32;
    f.count = 1;
    msg.fields.push_back(f);
  }
  {
    sensor_msgs::PointField f;
    f.name = "timestamp";
    f.offset = 12;
    f.datatype = timeDatatype;
    f.count = 1;
    msg.fields.push_back(f);
  }

  msg.point_step = 12 + tSize;
  msg.row_step = msg.point_step * msg.width;
  msg.data.resize(static_cast<size_t>(msg.row_step));

  for (size_t i = 0; i < n; i++)
  {
    unsigned char* p = &msg.data[i * msg.point_step];
    const float xyz[3] = {static_cast<float>(i), static_cast<float>(-static_cast<double>(i)), 1.0f};
    std::memcpy(p, xyz, sizeof(xyz));

    if (isF64)
    {
      const double t = times[i];
      std::memcpy(p + 12, &t, sizeof(t));
    }
    else
    {
      const float t = static_cast<float>(times[i]);
      std::memcpy(p + 12, &t, sizeof(t));
    }
  }
  return msg;
}

/// Span of the per-point time field after conversion.
double timeSpanOf(const mrpt::maps::CGenericPointsMap& pc)
{
  const auto* ts = pc.getPointsBufferRef_float_field(mrpt::maps::CPointsMap::POINT_FIELD_TIMESTAMP);
  if (!ts || ts->empty()) return 0;
  return static_cast<double>(*std::max_element(ts->cbegin(), ts->cend())) -
         static_cast<double>(*std::min_element(ts->cbegin(), ts->cend()));
}
}  // namespace

// A FLOAT64 time field used to fall through to the uint32 branch, which does
// not handle that datatype: every point came out with a timestamp of exactly
// zero, silently. Sensors publishing absolute per-point times (Hesai, among
// others) use this datatype.
TEST(PointCloud2, perPointTimesFloat64Absolute)
{
  // An absolute Unix-epoch time, as such sensors publish it, over a 100 ms sweep.
  const double t0 = 1733733287.298618;
  std::vector<double> times;
  for (int i = 0; i < 10; i++) times.push_back(t0 + 0.01 * i);

  const auto msg = makeCloudWithTimeField(sensor_msgs::PointField::FLOAT64, times);

  mrpt::maps::CGenericPointsMap pc;
  ASSERT_TRUE(mrpt::ros1bridge::fromROS(msg, pc));
  ASSERT_EQ(pc.size(), times.size());

  const auto* ts = pc.getPointsBufferRef_float_field(mrpt::maps::CPointsMap::POINT_FIELD_TIMESTAMP);
  ASSERT_TRUE(ts != nullptr);
  ASSERT_EQ(ts->size(), times.size());

  // The whole point: the sweep's duration must survive. Storing an absolute
  // epoch value in the float destination buffer would quantize every point
  // onto one value (float spacing near 1.7e9 is ~128 s), so the conversion has
  // to re-base onto the scan before narrowing.
  EXPECT_NEAR(timeSpanOf(pc), 0.09, 1e-3);

  // ...and the offsets must be ordered and evenly spaced, not just non-zero.
  for (size_t i = 1; i < ts->size(); i++)
  {
    EXPECT_NEAR(static_cast<double>((*ts)[i] - (*ts)[i - 1]), 0.01, 1e-3);
  }
}

// The same cloud with relative offsets in a FLOAT32 field: the path that always
// worked, kept here so a future change to the absolute-time handling cannot
// silently alter it.
TEST(PointCloud2, perPointTimesFloat32Relative)
{
  std::vector<double> times;
  for (int i = 0; i < 10; i++) times.push_back(0.01 * i);

  const auto msg = makeCloudWithTimeField(sensor_msgs::PointField::FLOAT32, times);

  mrpt::maps::CGenericPointsMap pc;
  ASSERT_TRUE(mrpt::ros1bridge::fromROS(msg, pc));

  EXPECT_NEAR(timeSpanOf(pc), 0.09, 1e-4);
}

#endif  // MRPT_VERSION >= 0x20f00
