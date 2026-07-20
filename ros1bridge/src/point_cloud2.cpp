/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/maps/CColouredPointsMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/ros1bridge/point_cloud2.h>
#include <mrpt/ros1bridge/time.h>
#include <mrpt/version.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>

using namespace mrpt::maps;

namespace
{
bool check_field(
    const sensor_msgs::PointField& input_field,
    std::string check_name,
    const sensor_msgs::PointField** output)
{
  bool coherence_error = false;
  if (input_field.name == check_name)
  {
    if (input_field.datatype != sensor_msgs::PointField::FLOAT32 &&
        input_field.datatype != sensor_msgs::PointField::FLOAT64 &&
        input_field.datatype != sensor_msgs::PointField::UINT16 &&
        input_field.datatype != sensor_msgs::PointField::UINT32 &&
        input_field.datatype != sensor_msgs::PointField::UINT8)
    {
      *output = nullptr;
      coherence_error = true;
    }
    else
    {
      *output = &input_field;
    }
  }
  return coherence_error;
}

void get_float_from_field(
    const sensor_msgs::PointField* field, const unsigned char* data, float& output)
{
  if (field != nullptr)
  {
    if (field->datatype == sensor_msgs::PointField::FLOAT32)
      output = *(reinterpret_cast<const float*>(&data[field->offset]));
    else
      output = (float)(*(reinterpret_cast<const double*>(&data[field->offset])));
  }
  else
    output = 0.0;
}
void get_uint16_from_field(
    const sensor_msgs::PointField* field, const unsigned char* data, uint16_t& output)
{
  if (field != nullptr)
  {
    if (field->datatype == sensor_msgs::PointField::UINT16)
      output = *(reinterpret_cast<const uint16_t*>(&data[field->offset]));
    else if (field->datatype == sensor_msgs::PointField::UINT8)
      output = *(reinterpret_cast<const uint8_t*>(&data[field->offset]));
  }
  else
    output = 0;
}
void get_uint32_from_field(
    const sensor_msgs::PointField* field, const unsigned char* data, uint32_t& output)
{
  if (field != nullptr)
  {
    if (field->datatype == sensor_msgs::PointField::UINT32)
      output = *(reinterpret_cast<const uint32_t*>(&data[field->offset]));
  }
  else
    output = 0;
}
}  // namespace

std::set<std::string> mrpt::ros1bridge::extractFields(const sensor_msgs::PointCloud2& msg)
{
  std::set<std::string> lst;
  for (const auto& f : msg.fields) lst.insert(f.name);
  return lst;
}

/** Convert sensor_msgs/PointCloud2 -> mrpt::slam::CSimplePointsMap
 *
 * \return true on successful conversion, false on any error.
 */
bool mrpt::ros1bridge::fromROS(const sensor_msgs::PointCloud2& msg, CSimplePointsMap& obj)
{
  // Copy point data
  unsigned int num_points = msg.width * msg.height;
  obj.clear();
  obj.reserve(num_points);

  bool incompatible = false;
  const sensor_msgs::PointField *x_field = nullptr, *y_field = nullptr, *z_field = nullptr;

  for (unsigned int i = 0; i < msg.fields.size() && !incompatible; i++)
  {
    incompatible |= check_field(msg.fields[i], "x", &x_field);
    incompatible |= check_field(msg.fields[i], "y", &y_field);
    incompatible |= check_field(msg.fields[i], "z", &z_field);
  }

  if (incompatible || (!x_field || !y_field || !z_field)) return false;

  // If not, memcpy each group of contiguous fields separately
  for (unsigned int row = 0; row < msg.height; ++row)
  {
    const unsigned char* row_data = &msg.data[row * msg.row_step];
    for (uint32_t col = 0; col < msg.width; ++col)
    {
      const unsigned char* msg_data = row_data + col * msg.point_step;

      float x, y, z;
      get_float_from_field(x_field, msg_data, x);
      get_float_from_field(y_field, msg_data, y);
      get_float_from_field(z_field, msg_data, z);
      obj.insertPoint(x, y, z);
    }
  }

  return true;
}

bool mrpt::ros1bridge::fromROS(const sensor_msgs::PointCloud2& msg, CGenericPointsMap& obj)
{
  // Copy point data
  unsigned int num_points = msg.width * msg.height;

  bool incompatible = false;
  const sensor_msgs::PointField *x_field = nullptr, *y_field = nullptr, *z_field = nullptr,
                                *i_field = nullptr, *r_field = nullptr, *t_field = nullptr;

  for (unsigned int i = 0; i < msg.fields.size() && !incompatible; i++)
  {
    incompatible |= check_field(msg.fields[i], "x", &x_field);
    incompatible |= check_field(msg.fields[i], "y", &y_field);
    incompatible |= check_field(msg.fields[i], "z", &z_field);
    incompatible |= check_field(msg.fields[i], "intensity", &i_field);
    incompatible |= check_field(msg.fields[i], "ring", &r_field);

    incompatible |= check_field(msg.fields[i], "timestamp", &t_field);
    incompatible |= check_field(msg.fields[i], "time", &t_field);
    incompatible |= check_field(msg.fields[i], "t", &t_field);
  }

  if (incompatible || (!x_field || !y_field || !z_field)) return false;

  if (i_field) obj.registerField_float(CPointsMap::POINT_FIELD_INTENSITY);
  if (r_field) obj.registerField_uint16(CPointsMap::POINT_FIELD_RING_ID);
  if (t_field) obj.registerField_float(CPointsMap::POINT_FIELD_TIMESTAMP);

  obj.resize(num_points);

  std::optional<float> minTime, maxTime;

  unsigned int idx = 0;
  for (unsigned int row = 0; row < msg.height; ++row)
  {
    const unsigned char* row_data = &msg.data[row * msg.row_step];
    for (uint32_t col = 0; col < msg.width; ++col, ++idx)
    {
      const unsigned char* msg_data = row_data + col * msg.point_step;

      float x, y, z;
      get_float_from_field(x_field, msg_data, x);
      get_float_from_field(y_field, msg_data, y);
      get_float_from_field(z_field, msg_data, z);
      obj.setPointFast(idx, x, y, z);

      if (i_field)
      {
        float i;
        get_float_from_field(i_field, msg_data, i);
        obj.setPointField_float(idx, CPointsMap::POINT_FIELD_INTENSITY, i);
      }
      if (r_field)
      {
        uint16_t ring_id = 0;
        get_uint16_from_field(r_field, msg_data, ring_id);
        obj.setPointField_uint16(idx, CPointsMap::POINT_FIELD_RING_ID, ring_id);
      }
      if (t_field)
      {
        float t = 0;
        if (t_field->datatype == sensor_msgs::PointField::FLOAT32)
        {
          get_float_from_field(t_field, msg_data, t);
        }
        else
        {
          uint32_t t_raw = 0;
          get_uint32_from_field(t_field, msg_data, t_raw);

          // Convention:
          // I only found one case (NTU Viral dataset) using uint32_t for time,
          // and times ranged from 0 to ~99822766 = 100,000,000 = 1e8
          // so they seem to be nanoseconds:
          t = t_raw * 1e-9f;
        }
        obj.setPointField_float(idx, CPointsMap::POINT_FIELD_TIMESTAMP, t);

        if (!minTime)
        {
          minTime = t;
          maxTime = t;
        }
        else
        {
          mrpt::keep_min(*minTime, t);
          mrpt::keep_max(*maxTime, t);
        }
      }
    }
  }

  // Force timestamps to be in the range [-T/2,T/2]:
  if (t_field && minTime && *maxTime > *minTime)
  {
    const float At = (*maxTime - *minTime) * 0.5f;
    for (size_t i = 0; i < obj.size(); i++)
    {
      const float t = obj.getPointField_float(i, CPointsMap::POINT_FIELD_TIMESTAMP);
      obj.setPointField_float(i, CPointsMap::POINT_FIELD_TIMESTAMP, t - At);
    }
  }

  return true;
}

bool mrpt::ros1bridge::toROS(
    const CSimplePointsMap& obj, const std_msgs::Header& msg_header, sensor_msgs::PointCloud2& msg)
{
  msg.header = msg_header;

  // 2D structure of the point cloud. If the cloud is unordered, height is
  //  1 and width is the length of the point cloud.
  msg.height = 1;
  msg.width = obj.size();

  std::array<std::string, 3> names = {"x", "y", "z"};
  std::array<size_t, 3> offsets = {0, sizeof(float) * 1, sizeof(float) * 2};

  msg.fields.resize(3);
  for (size_t i = 0; i < 3; i++)
  {
    auto& f = msg.fields.at(i);

    f.count = 1;
    f.offset = offsets[i];
    f.datatype = sensor_msgs::PointField::FLOAT32;
    f.name = names[i];
  }

#if MRPT_IS_BIG_ENDIAN
  msg.is_bigendian = true;
#else
  msg.is_bigendian = false;
#endif

  msg.point_step = sizeof(float) * 3;
  msg.row_step = msg.width * msg.point_step;

  // data:
  msg.data.resize(msg.row_step * msg.height);

  const auto& xs = obj.getPointsBufferRef_x();
  const auto& ys = obj.getPointsBufferRef_y();
  const auto& zs = obj.getPointsBufferRef_z();

  float* pointDest = reinterpret_cast<float*>(msg.data.data());
  for (size_t i = 0; i < xs.size(); i++)
  {
    *pointDest++ = xs[i];
    *pointDest++ = ys[i];
    *pointDest++ = zs[i];
  }

  return true;
}

bool mrpt::ros1bridge::toROS(
    const CGenericPointsMap& obj, const std_msgs::Header& msg_header, sensor_msgs::PointCloud2& msg)
{
  msg.header = msg_header;

  // 2D structure of the point cloud. If the cloud is unordered, height is
  //  1 and width is the length of the point cloud.
  msg.height = 1;
  msg.width = obj.size();

  const bool hasIntensity = obj.hasPointField(CPointsMap::POINT_FIELD_INTENSITY);
  const bool hasTime = obj.hasPointField(CPointsMap::POINT_FIELD_TIMESTAMP);
  const bool hasRing = obj.hasPointField(CPointsMap::POINT_FIELD_RING_ID);

  std::vector<std::string> names = {"x", "y", "z"};
  std::vector<size_t> offsets = {0, sizeof(float) * 1, sizeof(float) * 2};

  msg.point_step = sizeof(float) * 3;

  if (hasIntensity)
  {
    ASSERT_EQUAL_(
        obj.getPointsBufferRef_float_field(CPointsMap::POINT_FIELD_INTENSITY)->size(), obj.size());
    names.push_back("intensity");
    offsets.push_back(msg.point_step);
    msg.point_step += sizeof(float);
  }
  if (hasTime)
  {
    ASSERT_EQUAL_(
        obj.getPointsBufferRef_float_field(CPointsMap::POINT_FIELD_TIMESTAMP)->size(), obj.size());
    names.push_back("time");
    offsets.push_back(msg.point_step);
    msg.point_step += sizeof(float);
  }
  if (hasRing)
  {
    ASSERT_EQUAL_(
        obj.getPointsBufferRef_uint16_field(CPointsMap::POINT_FIELD_RING_ID)->size(), obj.size());
    names.push_back("ring");
    offsets.push_back(msg.point_step);
    msg.point_step += sizeof(uint16_t);
  }

  msg.fields.resize(names.size());
  for (size_t i = 0; i < names.size(); i++)
  {
    auto& f = msg.fields.at(i);

    f.count = 1;
    f.offset = offsets[i];
    f.datatype =
        (names[i] == "ring") ? sensor_msgs::PointField::UINT16 : sensor_msgs::PointField::FLOAT32;
    f.name = names[i];
  }

#if MRPT_IS_BIG_ENDIAN
  msg.is_bigendian = true;
#else
  msg.is_bigendian = false;
#endif

  msg.row_step = msg.width * msg.point_step;

  // data:
  msg.data.resize(msg.row_step * msg.height);

  const auto& xs = obj.getPointsBufferRef_x();
  const auto& ys = obj.getPointsBufferRef_y();
  const auto& zs = obj.getPointsBufferRef_z();
  const auto* Is = hasIntensity
                       ? obj.getPointsBufferRef_float_field(CPointsMap::POINT_FIELD_INTENSITY)
                       : nullptr;
  const auto* Ts =
      hasTime ? obj.getPointsBufferRef_float_field(CPointsMap::POINT_FIELD_TIMESTAMP) : nullptr;
  const auto* Rs =
      hasRing ? obj.getPointsBufferRef_uint16_field(CPointsMap::POINT_FIELD_RING_ID) : nullptr;

  uint8_t* pointDest = msg.data.data();
  for (size_t i = 0; i < xs.size(); i++)
  {
    int f = 0;
    memcpy(pointDest + offsets[f++], &xs[i], sizeof(float));
    memcpy(pointDest + offsets[f++], &ys[i], sizeof(float));
    memcpy(pointDest + offsets[f++], &zs[i], sizeof(float));

    if (hasIntensity) memcpy(pointDest + offsets[f++], &(*Is)[i], sizeof(float));

    if (hasTime) memcpy(pointDest + offsets[f++], &(*Ts)[i], sizeof(float));

    if (hasRing) memcpy(pointDest + offsets[f++], &(*Rs)[i], sizeof(uint16_t));

    pointDest += msg.point_step;
  }

  return true;
}

/** Convert sensor_msgs/PointCloud2 -> mrpt::obs::CObservationRotatingScan */
bool mrpt::ros1bridge::fromROS(
    const sensor_msgs::PointCloud2& msg,
    mrpt::obs::CObservationRotatingScan& obj,
    const mrpt::poses::CPose3D& sensorPoseOnRobot,
    unsigned int num_azimuth_divisions,
    float max_intensity)
{
  // Copy point data
  obj.timestamp = mrpt::ros1bridge::fromROS(msg.header.stamp);

  bool incompatible = false;
  const sensor_msgs::PointField *x_field = nullptr, *y_field = nullptr, *z_field = nullptr,
                                *i_field = nullptr, *ring_field = nullptr;

  for (unsigned int i = 0; i < msg.fields.size() && !incompatible; i++)
  {
    incompatible |= check_field(msg.fields[i], "x", &x_field);
    incompatible |= check_field(msg.fields[i], "y", &y_field);
    incompatible |= check_field(msg.fields[i], "z", &z_field);
    incompatible |= check_field(msg.fields[i], "ring", &ring_field);
    check_field(msg.fields[i], "intensity", &i_field);
  }

  if (incompatible || (!x_field || !y_field || !z_field || !ring_field)) return false;

  // 1st: go through the scan and find ring count:
  uint16_t ring_min = 0, ring_max = 0;

  for (unsigned int row = 0; row < msg.height; ++row)
  {
    const unsigned char* row_data = &msg.data[row * msg.row_step];
    for (uint32_t col = 0; col < msg.width; ++col)
    {
      const unsigned char* msg_data = row_data + col * msg.point_step;
      uint16_t ring_id = 0;
      get_uint16_from_field(ring_field, msg_data, ring_id);

      mrpt::keep_min(ring_min, ring_id);
      mrpt::keep_max(ring_max, ring_id);
    }
  }
  ASSERT_NOT_EQUAL_(ring_min, ring_max);

  obj.rowCount = ring_max - ring_min + 1;

  const bool inputCloudIsOrganized = msg.height != 1;

  if (!num_azimuth_divisions)
  {
    if (inputCloudIsOrganized)
    {
      ASSERT_GT_(msg.width, 0U);
      num_azimuth_divisions = msg.width;
    }
    else
    {
      THROW_EXCEPTION(
          "An explicit value for num_azimuth_divisions must be given if "
          "the input cloud is not 'organized'");
    }
  }

  obj.columnCount = num_azimuth_divisions;

  obj.rangeImage.resize(obj.rowCount, obj.columnCount);
  obj.rangeImage.fill(0);

  obj.sensorPose = sensorPoseOnRobot;

  // Default unit: 1cm
  if (obj.rangeResolution == 0) obj.rangeResolution = 1e-2;

  if (i_field)
  {
    obj.intensityImage.resize(obj.rowCount, obj.columnCount);
    obj.intensityImage.fill(0);
  }
  else
    obj.intensityImage.resize(0, 0);

  if (inputCloudIsOrganized)
  {
    obj.organizedPoints.resize(obj.rowCount, obj.columnCount);
  }

  // If not, memcpy each group of contiguous fields separately
  for (unsigned int row = 0; row < msg.height; ++row)
  {
    const unsigned char* row_data = &msg.data[row * msg.row_step];
    for (uint32_t col = 0; col < msg.width; ++col)
    {
      const unsigned char* msg_data = row_data + col * msg.point_step;

      float x = 0, y = 0, z = 0;
      uint16_t ring_id = 0;
      get_float_from_field(x_field, msg_data, x);
      get_float_from_field(y_field, msg_data, y);
      get_float_from_field(z_field, msg_data, z);
      get_uint16_from_field(ring_field, msg_data, ring_id);

      const mrpt::math::TPoint3D localPt = {x, y, z};

      unsigned int az_idx;
      if (inputCloudIsOrganized)
      {
        // "azimuth index" is just the "column":
        az_idx = col;
      }
      else
      {
        // Recover "azimuth index" from trigonometry:
        const double azimuth = std::atan2(localPt.y, localPt.x);

        az_idx = lround((num_azimuth_divisions - 1) * (azimuth + M_PI) / (2 * M_PI));
        ASSERT_LE_(az_idx, num_azimuth_divisions - 1);
      }

      // Store in matrix form:
      obj.rangeImage(ring_id, az_idx) = lround(localPt.norm() / obj.rangeResolution);

      if (i_field)
      {
        float intensity = 0;
        get_float_from_field(i_field, msg_data, intensity);
        obj.intensityImage(ring_id, az_idx) = lround(255 * intensity / max_intensity);
      }

      if (inputCloudIsOrganized) obj.organizedPoints(ring_id, az_idx) = localPt;
    }
  }

  return true;
}
