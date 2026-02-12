/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2026, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/maps/CColouredPointsMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/ros2bridge/point_cloud2.h>
#include <mrpt/ros2bridge/time.h>
#include <mrpt/version.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
//
#include <mrpt/config.h>  // MRPT_IS_BIG_ENDIAN

#include <cstdlib>

using namespace mrpt::maps;

namespace
{
bool check_field(
    const sensor_msgs::msg::PointField& input_field,
    std::string check_name,
    const sensor_msgs::msg::PointField** output)
{
  bool coherence_error = false;
  if (input_field.name == check_name)
  {
    if (input_field.datatype != sensor_msgs::msg::PointField::FLOAT32 &&
        input_field.datatype != sensor_msgs::msg::PointField::FLOAT64 &&
        input_field.datatype != sensor_msgs::msg::PointField::UINT16 &&
        input_field.datatype != sensor_msgs::msg::PointField::UINT32 &&
        input_field.datatype != sensor_msgs::msg::PointField::UINT8)
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

// This will be used as a wildcard for all other field types not directly supported by
// CGenericPointsMap
void get_float_from_field(
    const sensor_msgs::msg::PointField* field, const unsigned char* data, float& output)
{
  if (field != nullptr)
  {
    if (field->datatype == sensor_msgs::msg::PointField::FLOAT32)
    {
      output = *(reinterpret_cast<const float*>(&data[field->offset]));
    }
    else if (field->datatype == sensor_msgs::msg::PointField::FLOAT64)
    {
      output = static_cast<float>(*(reinterpret_cast<const double*>(&data[field->offset])));
    }
    else if (field->datatype == sensor_msgs::msg::PointField::UINT32)
    {
      output = static_cast<float>(*(reinterpret_cast<const uint32_t*>(&data[field->offset])));
    }
    else if (field->datatype == sensor_msgs::msg::PointField::INT8)
    {
      output = static_cast<float>(*(reinterpret_cast<const int8_t*>(&data[field->offset])));
    }
    else if (field->datatype == sensor_msgs::msg::PointField::INT16)
    {
      output = static_cast<float>(*(reinterpret_cast<const int16_t*>(&data[field->offset])));
    }
    else if (field->datatype == sensor_msgs::msg::PointField::INT32)
    {
      output = static_cast<float>(*(reinterpret_cast<const int32_t*>(&data[field->offset])));
    }
  }
  else
  {
    output = 0.0;
  }
}

void get_double_from_field(
    const sensor_msgs::msg::PointField* field, const unsigned char* data, double& output)
{
  if (field != nullptr)
  {
    if (field->datatype == sensor_msgs::msg::PointField::FLOAT32)
    {
      output = static_cast<double>(*(reinterpret_cast<const float*>(&data[field->offset])));
    }
    else if (field->datatype == sensor_msgs::msg::PointField::FLOAT64)
    {
      output = *(reinterpret_cast<const double*>(&data[field->offset]));
    }
  }
  else
  {
    output = 0.0;
  }
}

void get_uint8_from_field(
    const sensor_msgs::msg::PointField* field, const unsigned char* data, uint8_t& output)
{
  if (field != nullptr)
  {
    if (field->datatype == sensor_msgs::msg::PointField::UINT8)
    {
      output = *(reinterpret_cast<const uint8_t*>(&data[field->offset]));
    }
  }
  else
  {
    output = 0;
  }
}

void get_uint16_from_field(
    const sensor_msgs::msg::PointField* field, const unsigned char* data, uint16_t& output)
{
  if (field != nullptr)
  {
    if (field->datatype == sensor_msgs::msg::PointField::UINT16)
    {
      output = *(reinterpret_cast<const uint16_t*>(&data[field->offset]));
    }
  }
  else
  {
    output = 0;
  }
}
void get_uint32_from_field(
    const sensor_msgs::msg::PointField* field, const unsigned char* data, uint32_t& output)
{
  if (field != nullptr)
  {
    if (field->datatype == sensor_msgs::msg::PointField::UINT32)
    {
      output = *(reinterpret_cast<const uint32_t*>(&data[field->offset]));
    }
  }
  else
  {
    output = 0;
  }
}
}  // namespace

std::set<std::string> mrpt::ros2bridge::extractFields(const sensor_msgs::msg::PointCloud2& msg)
{
  std::set<std::string> lst;
  for (const auto& f : msg.fields)
  {
    lst.insert(f.name);
  }
  return lst;
}

/** Convert sensor_msgs/PointCloud2 -> mrpt::slam::CSimplePointsMap
 *
 * \return true on successful conversion, false on any error.
 */
bool mrpt::ros2bridge::fromROS(const sensor_msgs::msg::PointCloud2& msg, CSimplePointsMap& obj)
{
  // Copy point data
  unsigned int num_points = msg.width * msg.height;
  obj.clear();
  obj.reserve(num_points);

  bool incompatible = false;
  const sensor_msgs::msg::PointField *x_field = nullptr, *y_field = nullptr, *z_field = nullptr;

  for (unsigned int i = 0; i < msg.fields.size() && !incompatible; i++)
  {
    incompatible |= check_field(msg.fields[i], "x", &x_field);
    incompatible |= check_field(msg.fields[i], "y", &y_field);
    incompatible |= check_field(msg.fields[i], "z", &z_field);
  }

  if (incompatible || (!x_field || !y_field || !z_field))
  {
    return false;
  }

  // If not, memcpy each group of contiguous fields separately
  for (std::size_t row = 0; row < msg.height; ++row)
  {
    const unsigned char* row_data = &msg.data[row * msg.row_step];
    for (std::size_t col = 0; col < msg.width; ++col)
    {
      const unsigned char* msg_data = row_data + col * msg.point_step;

      float x = 0, y = 0, z = 0;
      get_float_from_field(x_field, msg_data, x);
      get_float_from_field(y_field, msg_data, y);
      get_float_from_field(z_field, msg_data, z);
      obj.insertPoint(x, y, z);
    }
  }

  return true;
}

bool mrpt::ros2bridge::fromROS(const sensor_msgs::msg::PointCloud2& msg, CPointsMapXYZI& obj)
{
  // Copy point data
  unsigned int num_points = msg.width * msg.height;
  obj.clear();
  obj.reserve(num_points);

  bool incompatible = false;
  const sensor_msgs::msg::PointField *x_field = nullptr, *y_field = nullptr, *z_field = nullptr,
                                     *i_field = nullptr;

  for (unsigned int i = 0; i < msg.fields.size() && !incompatible; i++)
  {
    incompatible |= check_field(msg.fields[i], "x", &x_field);
    incompatible |= check_field(msg.fields[i], "y", &y_field);
    incompatible |= check_field(msg.fields[i], "z", &z_field);
    incompatible |= check_field(msg.fields[i], "intensity", &i_field);
  }

  if (incompatible || (!x_field || !y_field || !z_field || !i_field))
  {
    return false;
  }

#if MRPT_VERSION >= 0x20f00  // 2.15.0
  auto* Is = obj.getPointsBufferRef_float_field(CPointsMapXYZI::POINT_FIELD_INTENSITY);
  ASSERT_(Is);
#endif

  for (std::size_t row = 0; row < msg.height; ++row)
  {
    const unsigned char* row_data = &msg.data[row * msg.row_step];
    for (std::size_t col = 0; col < msg.width; ++col)
    {
      const unsigned char* msg_data = row_data + col * msg.point_step;

      float x = 0, y = 0, z = 0, i = 0;
      get_float_from_field(x_field, msg_data, x);
      get_float_from_field(y_field, msg_data, y);
      get_float_from_field(z_field, msg_data, z);
      get_float_from_field(i_field, msg_data, i);
      obj.insertPoint(x, y, z);

#if MRPT_VERSION >= 0x20f00  // 2.15.0
      Is->push_back(i);
#else
      obj.insertPointField_Intensity(i);
#endif
    }
  }
  return true;
}

bool mrpt::ros2bridge::fromROS(const sensor_msgs::msg::PointCloud2& msg, CPointsMapXYZIRT& obj)
{
  // Copy point data
  unsigned int num_points = msg.width * msg.height;

  bool incompatible = false;
  const sensor_msgs::msg::PointField *x_field = nullptr, *y_field = nullptr, *z_field = nullptr,
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

  if (incompatible || (!x_field || !y_field || !z_field))
  {
    return false;
  }

  obj.resize_XYZIRT(num_points, !!i_field, !!r_field, !!t_field);

  unsigned int idx = 0;
  std::optional<double> baseTimeStamp;
  for (std::size_t row = 0; row < msg.height; ++row)
  {
    const unsigned char* row_data = &msg.data[row * msg.row_step];
    for (std::size_t col = 0; col < msg.width; ++col, ++idx)
    {
      const unsigned char* msg_data = row_data + col * msg.point_step;

      float x = 0, y = 0, z = 0;
      get_float_from_field(x_field, msg_data, x);
      get_float_from_field(y_field, msg_data, y);
      get_float_from_field(z_field, msg_data, z);
      obj.setPointFast(idx, x, y, z);

      if (i_field)
      {
        float i = 0;
        get_float_from_field(i_field, msg_data, i);
        obj.setPointIntensity(idx, i);
      }
      if (r_field)
      {
        uint16_t ring_id = 0;
        get_uint16_from_field(r_field, msg_data, ring_id);
        obj.setPointRing(idx, ring_id);
      }
      if (t_field)
      {
        double t = 0;

        if (t_field->datatype == sensor_msgs::msg::PointField::FLOAT32 ||
            t_field->datatype == sensor_msgs::msg::PointField::FLOAT64)
        {
          get_double_from_field(t_field, msg_data, t);
        }
        else
        {
          uint32_t tim = 0;

          get_uint32_from_field(t_field, msg_data, tim);

          // Convention: they seem to be nanoseconds:
          t = tim * 1e-9;
        }

        // If the sensor uses absolute timestamp, convert them to relative
        // since otherwise precision is lost in the double->float conversion:
        if (std::abs(t) > 5.0)
        {
          // It looks like absolute timestamps, convert to relative:
          if (!baseTimeStamp)
          {
            baseTimeStamp = t;
          }
          obj.setPointTime(idx, static_cast<float>(t - *baseTimeStamp));
        }
        else
        {
          // It looks relative timestamps:
          obj.setPointTime(idx, static_cast<float>(t));
        }
      }
    }
  }
  return true;
}

#if MRPT_VERSION >= 0x20f00  // 2.15.0
bool mrpt::ros2bridge::fromROS(
    const sensor_msgs::msg::PointCloud2& msg, mrpt::maps::CGenericPointsMap& obj)
{
  // Copy point data
  unsigned int num_points = msg.width * msg.height;

  bool incompatible = false;
  const sensor_msgs::msg::PointField* x_field = nullptr;
  const sensor_msgs::msg::PointField* y_field = nullptr;
  const sensor_msgs::msg::PointField* z_field = nullptr;
  const sensor_msgs::msg::PointField* t_field = nullptr;
  std::map<std::string, const sensor_msgs::msg::PointField*> other_fields_float;
  std::map<std::string, const sensor_msgs::msg::PointField*> other_fields_double;
  std::map<std::string, const sensor_msgs::msg::PointField*> other_fields_uint8;
  std::map<std::string, const sensor_msgs::msg::PointField*> other_fields_uint16;

  for (const auto& field : msg.fields)
  {
    if (field.name == "x" || field.name == "y" || field.name == "z")
    {
      incompatible |= check_field(field, "x", &x_field);
      incompatible |= check_field(field, "y", &y_field);
      incompatible |= check_field(field, "z", &z_field);
      continue;
    }

    // Timestamp per point must be handled specially to handle different conventions:
    if (field.name == "timestamp" || field.name == "time" || field.name == "t")
    {
      incompatible |= check_field(field, "timestamp", &t_field);
      incompatible |= check_field(field, "time", &t_field);
      incompatible |= check_field(field, "t", &t_field);
      continue;
    }

    // In MRPT CGenericPointsMap we support: double/float/uint16/uint8:
    if (field.datatype == sensor_msgs::msg::PointField::UINT16)
    {
      other_fields_uint16[field.name] = &field;
    }
#if MRPT_VERSION >= 0x020f03  // 2.15.3
    else if (field.datatype == sensor_msgs::msg::PointField::UINT8)
    {
      other_fields_uint8[field.name] = &field;
    }
    else if (field.datatype == sensor_msgs::msg::PointField::FLOAT64)
    {
      other_fields_double[field.name] = &field;
    }
#endif
    // Then, for float32, and for
    // anything else (very rarely used fields?), we convert into "float"
    else
    {
      other_fields_float[field.name] = &field;
    }
  }

  if (incompatible || (!x_field || !y_field || !z_field))
  {
    return false;
  }

  for (const auto& [name, _] : other_fields_float)
  {
    obj.registerField_float(name);
  }
  for (const auto& [name, _] : other_fields_uint16)
  {
    obj.registerField_uint16(name);
  }
  if (t_field)
  {
    obj.registerField_float("t");
  }
#if MRPT_VERSION >= 0x020f03  // 2.15.3
  for (const auto& [name, _] : other_fields_double)
  {
    obj.registerField_double(name);
  }
  for (const auto& [name, _] : other_fields_uint8)
  {
    obj.registerField_uint8(name);
  }
#endif

  obj.resize(num_points);

  unsigned int idx = 0;
  std::optional<double> baseTimeStamp;
  for (std::size_t row = 0; row < msg.height; ++row)
  {
    const unsigned char* row_data = &msg.data[static_cast<std::size_t>(row) * msg.row_step];
    for (std::size_t col = 0; col < msg.width; ++col, ++idx)
    {
      const unsigned char* msg_data = row_data + static_cast<std::size_t>(col) * msg.point_step;

      float x = 0, y = 0, z = 0;
      get_float_from_field(x_field, msg_data, x);
      get_float_from_field(y_field, msg_data, y);
      get_float_from_field(z_field, msg_data, z);
      obj.setPointFast(idx, x, y, z);

      for (const auto& [name, field_ptr] : other_fields_float)
      {
        float val = 0;
        get_float_from_field(field_ptr, msg_data, val);
        obj.setPointField_float(idx, name, val);
      }
      for (const auto& [name, field_ptr] : other_fields_uint16)
      {
        uint16_t val = 0;
        get_uint16_from_field(field_ptr, msg_data, val);
        obj.setPointField_uint16(idx, name, val);
      }
#if MRPT_VERSION >= 0x020f03  // 2.15.3
      for (const auto& [name, field_ptr] : other_fields_double)
      {
        double val = 0;
        get_double_from_field(field_ptr, msg_data, val);
        obj.setPointField_double(idx, name, val);
      }
      for (const auto& [name, field_ptr] : other_fields_uint8)
      {
        uint8_t val = 0;
        get_uint8_from_field(field_ptr, msg_data, val);
        obj.setPointField_uint8(idx, name, val);
      }
#endif

      if (t_field)
      {
        double t = 0;

        if (t_field->datatype == sensor_msgs::msg::PointField::FLOAT32 ||
            t_field->datatype == sensor_msgs::msg::PointField::FLOAT64)
        {
          get_double_from_field(t_field, msg_data, t);
        }
        else
        {
          uint32_t tim = 0;

          get_uint32_from_field(t_field, msg_data, tim);

          // Convention: they seem to be nanoseconds:
          t = tim * 1e-9;
        }

        // If the sensor uses absolute timestamp, convert them to relative
        // since otherwise precision is lost in the double->float conversion:
        if (std::abs(t) > 5.0)
        {
          // It looks like absolute timestamps, convert to relative:
          if (!baseTimeStamp)
          {
            baseTimeStamp = t;
          }
          obj.setPointField_float(idx, "t", static_cast<float>(t - *baseTimeStamp));
        }
        else
        {
          // It looks relative timestamps:
          obj.setPointField_float(idx, "t", static_cast<float>(t));
        }
      }
    }
  }
  return true;
}
bool mrpt::ros2bridge::toROS(
    const mrpt::maps::CGenericPointsMap& obj,
    const std_msgs::msg::Header& msg_header,
    sensor_msgs::msg::PointCloud2& msg)
{
  msg.header = msg_header;

  // Unordered cloud:
  msg.height = 1;
  msg.width = obj.size();

  // ---------------------------------------------------------------
  // Detect RGB color channels
  // ---------------------------------------------------------------
  // Two conventions in MRPT CGenericPointsMap / CPointsMap:
  //   uint8:  color_r, color_g, color_b  (range [0, 255])
  //   float:  color_rf, color_gf, color_bf (range [0, 1])
  // We merge whichever is present into a single "rgb" field using
  // the PCL/RViz packed-float convention.
  const bool hasColorU8 = obj.hasColor_u8();
  const bool hasColorF = obj.hasColor_f();
  const bool hasAnyColor = hasColorU8 || hasColorF;

  // Names of individual color channels to exclude from generic export:
  static const std::array<std::string_view, 6> colorFieldNames = {
      CPointsMap::POINT_FIELD_COLOR_Ru8, CPointsMap::POINT_FIELD_COLOR_Gu8,
      CPointsMap::POINT_FIELD_COLOR_Bu8, CPointsMap::POINT_FIELD_COLOR_Rf,
      CPointsMap::POINT_FIELD_COLOR_Gf,  CPointsMap::POINT_FIELD_COLOR_Bf};

  auto isColorField = [&](const std::string_view& name) -> bool
  {
    for (const auto& cn : colorFieldNames)
    {
      if (name == cn)
      {
        return true;
      }
    }
    return false;
  };

  // ---------------------------------------------------------------
  // Collect field metadata
  // ---------------------------------------------------------------
  // Basic XYZ:
  std::vector<std::string> names = {"x", "y", "z"};
  std::vector<size_t> offsets = {0, sizeof(float) * 1, sizeof(float) * 2};
  size_t point_step = sizeof(float) * 3;
  // Track the PointField datatype for each entry in names[]:
  std::vector<uint8_t> datatypes = {
      sensor_msgs::msg::PointField::FLOAT32, sensor_msgs::msg::PointField::FLOAT32,
      sensor_msgs::msg::PointField::FLOAT32};

  // Helper to skip xyz and color channels:
  auto remove_xyz = [](std::vector<std::string_view>& vec)
  {
    auto it = std::remove_if(
        vec.begin(), vec.end(),
        [](const std::string_view& n) { return n == "x" || n == "y" || n == "z"; });
    vec.erase(it, vec.end());
  };
  auto remove_color = [&](std::vector<std::string_view>& vec)
  {
    auto it = std::remove_if(
        vec.begin(), vec.end(), [&](const std::string_view& n) { return isColorField(n); });
    vec.erase(it, vec.end());
  };

  // Gather all field name lists, filtering out xyz and color:
  auto float_fields = obj.getPointFieldNames_float();
  remove_xyz(float_fields);
  remove_color(float_fields);

  auto uint16_fields = obj.getPointFieldNames_uint16();
  remove_xyz(uint16_fields);
  remove_color(uint16_fields);

#if MRPT_VERSION >= 0x020f03  // 2.15.3
  auto uint8_fields = obj.getPointFieldNames_uint8();
  remove_color(uint8_fields);  // strip color_r/g/b; we handle them via "rgb"

  auto double_fields = obj.getPointFieldNames_double();
  remove_color(double_fields);
#endif

  // Append packed "rgb" field if color is available:
  // Declared as FLOAT32 (4 bytes = packed 0x00RRGGBB) per PCL/RViz convention.
  size_t rgb_offset = 0;
  if (hasAnyColor)
  {
    names.push_back("rgb");
    rgb_offset = point_step;
    offsets.push_back(point_step);
    datatypes.push_back(sensor_msgs::msg::PointField::FLOAT32);
    point_step += sizeof(float);
  }

  // Append float fields:
  for (const auto& fn : float_fields)
  {
    names.push_back(std::string(fn));
    offsets.push_back(point_step);
    datatypes.push_back(sensor_msgs::msg::PointField::FLOAT32);
    point_step += sizeof(float);
  }

  // Append uint16 fields:
  for (const auto& un : uint16_fields)
  {
    names.push_back(std::string(un));
    offsets.push_back(point_step);
    datatypes.push_back(sensor_msgs::msg::PointField::UINT16);
    point_step += sizeof(uint16_t);
  }

#if MRPT_VERSION >= 0x020f03  // 2.15.3
  // Append uint8 fields (non-color ones):
  for (const auto& u8n : uint8_fields)
  {
    names.push_back(std::string(u8n));
    offsets.push_back(point_step);
    datatypes.push_back(sensor_msgs::msg::PointField::UINT8);
    point_step += sizeof(uint8_t);
  }

  // Append double fields:
  for (const auto& dn : double_fields)
  {
    names.push_back(std::string(dn));
    offsets.push_back(point_step);
    datatypes.push_back(sensor_msgs::msg::PointField::FLOAT64);
    point_step += sizeof(double);
  }
#endif

  // ---------------------------------------------------------------
  // Build msg.fields
  // ---------------------------------------------------------------
  msg.fields.resize(names.size());
  for (size_t i = 0; i < names.size(); ++i)
  {
    auto& f = msg.fields[i];
    f.name = names[i];
    f.count = 1;
    f.offset = static_cast<uint32_t>(offsets[i]);
    f.datatype = datatypes[i];
  }

#if MRPT_IS_BIG_ENDIAN
  msg.is_bigendian = true;
#else
  msg.is_bigendian = false;
#endif

  msg.point_step = static_cast<uint32_t>(point_step);
  msg.row_step = msg.width * msg.point_step;
  msg.data.resize(static_cast<std::size_t>(msg.row_step) * msg.height);

  // ---------------------------------------------------------------
  // Prepare buffer pointers for all field types
  // ---------------------------------------------------------------
  const auto& xs = obj.getPointsBufferRef_x();
  const auto& ys = obj.getPointsBufferRef_y();
  const auto& zs = obj.getPointsBufferRef_z();
  const size_t N = xs.size();
  ASSERT_EQUAL_(ys.size(), N);
  ASSERT_EQUAL_(zs.size(), N);
  ASSERT_EQUAL_(msg.width, N);

  // Color source buffers (whichever variant is available):
  const mrpt::aligned_std_vector<float>* color_rf_buf = nullptr;
  const mrpt::aligned_std_vector<float>* color_gf_buf = nullptr;
  const mrpt::aligned_std_vector<float>* color_bf_buf = nullptr;
#if MRPT_VERSION >= 0x020f03
  const mrpt::aligned_std_vector<uint8_t>* color_ru8_buf = nullptr;
  const mrpt::aligned_std_vector<uint8_t>* color_gu8_buf = nullptr;
  const mrpt::aligned_std_vector<uint8_t>* color_bu8_buf = nullptr;
#endif

  if (hasColorF)
  {
    color_rf_buf = obj.getPointsBufferRef_float_field(CPointsMap::POINT_FIELD_COLOR_Rf);
    color_gf_buf = obj.getPointsBufferRef_float_field(CPointsMap::POINT_FIELD_COLOR_Gf);
    color_bf_buf = obj.getPointsBufferRef_float_field(CPointsMap::POINT_FIELD_COLOR_Bf);
    ASSERT_(color_rf_buf && color_gf_buf && color_bf_buf);
    ASSERT_EQUAL_(color_rf_buf->size(), N);
    ASSERT_EQUAL_(color_gf_buf->size(), N);
    ASSERT_EQUAL_(color_bf_buf->size(), N);
  }
#if MRPT_VERSION >= 0x020f03
  else if (hasColorU8)
  {
    color_ru8_buf = obj.getPointsBufferRef_uint8_field(CPointsMap::POINT_FIELD_COLOR_Ru8);
    color_gu8_buf = obj.getPointsBufferRef_uint8_field(CPointsMap::POINT_FIELD_COLOR_Gu8);
    color_bu8_buf = obj.getPointsBufferRef_uint8_field(CPointsMap::POINT_FIELD_COLOR_Bu8);
    ASSERT_(color_ru8_buf && color_gu8_buf && color_bu8_buf);
    ASSERT_EQUAL_(color_ru8_buf->size(), N);
    ASSERT_EQUAL_(color_gu8_buf->size(), N);
    ASSERT_EQUAL_(color_bu8_buf->size(), N);
  }
#endif

  // Generic float field buffers:
  std::vector<const mrpt::aligned_std_vector<float>*> float_bufs;
  float_bufs.reserve(float_fields.size());
  for (const auto& fn : float_fields)
  {
    const auto* v = obj.getPointsBufferRef_float_field(fn);
    ASSERT_(v);
    ASSERT_EQUAL_(v->size(), N);
    float_bufs.push_back(v);
  }

  // Generic uint16 field buffers:
  std::vector<const mrpt::aligned_std_vector<uint16_t>*> uint16_bufs;
  uint16_bufs.reserve(uint16_fields.size());
  for (const auto& un : uint16_fields)
  {
#if MRPT_VERSION >= 0x020f04  // 2.15.4
    const auto* v = obj.getPointsBufferRef_uint16_field(un);
#else
    const auto* v = obj.getPointsBufferRef_uint_field(un);
#endif
    ASSERT_(v);
    ASSERT_EQUAL_(v->size(), N);
    uint16_bufs.push_back(v);
  }

#if MRPT_VERSION >= 0x020f03
  // Generic uint8 field buffers (non-color):
  std::vector<const mrpt::aligned_std_vector<uint8_t>*> uint8_bufs;
  uint8_bufs.reserve(uint8_fields.size());
  for (const auto& u8n : uint8_fields)
  {
    const auto* v = obj.getPointsBufferRef_uint8_field(u8n);
    ASSERT_(v);
    ASSERT_EQUAL_(v->size(), N);
    uint8_bufs.push_back(v);
  }

  // Generic double field buffers:
  std::vector<const mrpt::aligned_std_vector<double>*> double_bufs;
  double_bufs.reserve(double_fields.size());
  for (const auto& dn : double_fields)
  {
    const auto* v = obj.getPointsBufferRef_double_field(dn);
    ASSERT_(v);
    ASSERT_EQUAL_(v->size(), N);
    double_bufs.push_back(v);
  }
#endif

  // ---------------------------------------------------------------
  // Fill per-point data
  // ---------------------------------------------------------------
  // Track offsets by field group for the inner loop.
  // Layout in names[]: x(0), y(1), z(2), [rgb(3)], floats..., uint16s..., [uint8s..., doubles...]
  // We use direct offset values already stored in offsets[].
  const size_t firstFloatIdx = 3 + (hasAnyColor ? 1 : 0);
  const size_t firstU16Idx = firstFloatIdx + float_fields.size();
#if MRPT_VERSION >= 0x020f03
  const size_t firstU8Idx = firstU16Idx + uint16_fields.size();
  const size_t firstDblIdx = firstU8Idx + uint8_fields.size();
#endif

  uint8_t* dst = msg.data.data();
  for (size_t i = 0; i < N; ++i)
  {
    // x, y, z
    std::memcpy(dst + offsets[0], &xs[i], sizeof(float));
    std::memcpy(dst + offsets[1], &ys[i], sizeof(float));
    std::memcpy(dst + offsets[2], &zs[i], sizeof(float));

    // Packed rgb (PCL/RViz convention: uint32 0x00RRGGBB reinterpreted as float)
    if (hasAnyColor)
    {
      uint8_t r = 0, g = 0, b = 0;
      if (hasColorF)
      {
        // float [0,1] → uint8 [0,255]
        r = static_cast<uint8_t>(std::min(255.0f, std::max(0.0f, (*color_rf_buf)[i] * 255.0f)));
        g = static_cast<uint8_t>(std::min(255.0f, std::max(0.0f, (*color_gf_buf)[i] * 255.0f)));
        b = static_cast<uint8_t>(std::min(255.0f, std::max(0.0f, (*color_bf_buf)[i] * 255.0f)));
      }
#if MRPT_VERSION >= 0x020f03
      else if (hasColorU8)
      {
        r = (*color_ru8_buf)[i];
        g = (*color_gu8_buf)[i];
        b = (*color_bu8_buf)[i];
      }
#endif

      const uint32_t rgb_packed = (static_cast<uint32_t>(r) << 16) |
                                  (static_cast<uint32_t>(g) << 8) | static_cast<uint32_t>(b);
      std::memcpy(dst + rgb_offset, &rgb_packed, sizeof(uint32_t));
    }

    // Generic float fields
    for (size_t fi = 0; fi < float_bufs.size(); ++fi)
    {
      std::memcpy(dst + offsets[firstFloatIdx + fi], &(*float_bufs[fi])[i], sizeof(float));
    }

    // Generic uint16 fields
    for (size_t ui = 0; ui < uint16_bufs.size(); ++ui)
    {
      std::memcpy(dst + offsets[firstU16Idx + ui], &(*uint16_bufs[ui])[i], sizeof(uint16_t));
    }

#if MRPT_VERSION >= 0x020f03
    // Generic uint8 fields (non-color)
    for (size_t u8i = 0; u8i < uint8_bufs.size(); ++u8i)
    {
      std::memcpy(dst + offsets[firstU8Idx + u8i], &(*uint8_bufs[u8i])[i], sizeof(uint8_t));
    }

    // Generic double fields
    for (size_t di = 0; di < double_bufs.size(); ++di)
    {
      std::memcpy(dst + offsets[firstDblIdx + di], &(*double_bufs[di])[i], sizeof(double));
    }
#endif

    dst += msg.point_step;
  }

  return true;
}

#endif

bool mrpt::ros2bridge::toROS(
    const CSimplePointsMap& obj,
    const std_msgs::msg::Header& msg_header,
    sensor_msgs::msg::PointCloud2& msg)
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
    f.datatype = sensor_msgs::msg::PointField::FLOAT32;
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
  msg.data.resize(static_cast<size_t>(msg.row_step) * msg.height);

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

bool mrpt::ros2bridge::toROS(
    const CPointsMapXYZI& obj,
    const std_msgs::msg::Header& msg_header,
    sensor_msgs::msg::PointCloud2& msg)
{
  msg.header = msg_header;

  // 2D structure of the point cloud. If the cloud is unordered, height is
  //  1 and width is the length of the point cloud.
  msg.height = 1;
  msg.width = obj.size();

  std::array<std::string, 4> names = {"x", "y", "z", "intensity"};
  std::array<size_t, 4> offsets = {0, sizeof(float) * 1, sizeof(float) * 2, sizeof(float) * 3};

  msg.fields.resize(4);
  for (size_t i = 0; i < 4; i++)
  {
    auto& f = msg.fields.at(i);

    f.count = 1;
    f.offset = offsets[i];
    f.datatype = sensor_msgs::msg::PointField::FLOAT32;
    f.name = names[i];
  }

#if MRPT_IS_BIG_ENDIAN
  msg.is_bigendian = true;
#else
  msg.is_bigendian = false;
#endif

  msg.point_step = sizeof(float) * 4;
  msg.row_step = msg.width * msg.point_step;

  // data:
  msg.data.resize(static_cast<std::size_t>(msg.row_step) * msg.height);

  const auto& xs = obj.getPointsBufferRef_x();
  const auto& ys = obj.getPointsBufferRef_y();
  const auto& zs = obj.getPointsBufferRef_z();

#if MRPT_VERSION >= 0x20f00  // 2.15.0
  const auto* Is = obj.getPointsBufferRef_float_field(CPointsMapXYZI::POINT_FIELD_INTENSITY);
#else
  const auto* Is = obj.getPointsBufferRef_intensity();
#endif
  ASSERT_(Is);

  float* pointDest = reinterpret_cast<float*>(msg.data.data());
  for (size_t i = 0; i < xs.size(); i++)
  {
    *pointDest++ = xs[i];
    *pointDest++ = ys[i];
    *pointDest++ = zs[i];
    *pointDest++ = (*Is)[i];
  }

  return true;
}

bool mrpt::ros2bridge::toROS(
    const CPointsMapXYZIRT& obj,
    const std_msgs::msg::Header& msg_header,
    sensor_msgs::msg::PointCloud2& msg)
{
  msg.header = msg_header;

  // 2D structure of the point cloud. If the cloud is unordered, height is
  //  1 and width is the length of the point cloud.
  msg.height = 1;
  msg.width = obj.size();

  std::vector<std::string> names = {"x", "y", "z"};
  std::vector<size_t> offsets = {0, sizeof(float) * 1, sizeof(float) * 2};

  msg.point_step = sizeof(float) * 3;

#if MRPT_VERSION >= 0x20f00  // 2.15.0
  const auto* Is = obj.getPointsBufferRef_float_field(CPointsMapXYZIRT::POINT_FIELD_INTENSITY);

#if MRPT_VERSION >= 0x020f04  // 2.15.4
  const auto* Rs = obj.getPointsBufferRef_uint16_field(CPointsMapXYZIRT::POINT_FIELD_RING_ID);
#else
  const auto* Rs = obj.getPointsBufferRef_uint_field(CPointsMapXYZIRT::POINT_FIELD_RING_ID);
#endif

  const auto* Ts = obj.getPointsBufferRef_float_field(CPointsMapXYZIRT::POINT_FIELD_TIMESTAMP);
#else
  const auto* Is = obj.getPointsBufferRef_intensity();
  const auto* Rs = obj.getPointsBufferRef_ring();
  const auto* Ts = obj.getPointsBufferRef_timestamp();
#endif

  if (obj.hasIntensityField())
  {
    ASSERT_(Is);
    ASSERT_EQUAL_(Is->size(), obj.size());
    names.push_back("intensity");
    offsets.push_back(msg.point_step);
    msg.point_step += sizeof(float);
  }
  if (obj.hasTimeField())
  {
    ASSERT_(Ts);
    ASSERT_EQUAL_(Ts->size(), obj.size());
    names.push_back("time");
    offsets.push_back(msg.point_step);
    msg.point_step += sizeof(float);
  }
  if (obj.hasRingField())
  {
    ASSERT_(Rs);
    ASSERT_EQUAL_(Rs->size(), obj.size());
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
    f.datatype = (names[i] == "ring") ? sensor_msgs::msg::PointField::UINT16
                                      : sensor_msgs::msg::PointField::FLOAT32;
    f.name = names[i];
  }

#if MRPT_IS_BIG_ENDIAN
  msg.is_bigendian = true;
#else
  msg.is_bigendian = false;
#endif

  msg.row_step = msg.width * msg.point_step;

  // data:
  msg.data.resize(static_cast<std::size_t>(msg.row_step) * msg.height);

  const auto& xs = obj.getPointsBufferRef_x();
  const auto& ys = obj.getPointsBufferRef_y();
  const auto& zs = obj.getPointsBufferRef_z();

  uint8_t* pointDest = msg.data.data();
  for (size_t i = 0; i < xs.size(); i++)
  {
    int f = 0;
    memcpy(pointDest + offsets[f++], &xs[i], sizeof(float));
    memcpy(pointDest + offsets[f++], &ys[i], sizeof(float));
    memcpy(pointDest + offsets[f++], &zs[i], sizeof(float));

    if (obj.hasIntensityField())
    {
      memcpy(pointDest + offsets[f++], &(*Is)[i], sizeof(float));
    }

    if (obj.hasTimeField())
    {
      memcpy(pointDest + offsets[f++], &(*Ts)[i], sizeof(float));
    }

    if (obj.hasRingField())
    {
      memcpy(pointDest + offsets[f++], &(*Rs)[i], sizeof(uint16_t));
    }

    pointDest += msg.point_step;
  }

  return true;
}

/** Convert sensor_msgs/PointCloud2 -> mrpt::obs::CObservationRotatingScan */
bool mrpt::ros2bridge::fromROS(
    const sensor_msgs::msg::PointCloud2& msg,
    mrpt::obs::CObservationRotatingScan& obj,
    const mrpt::poses::CPose3D& sensorPoseOnRobot,
    unsigned int num_azimuth_divisions,
    float max_intensity)
{
  // Copy point data
  obj.timestamp = mrpt::ros2bridge::fromROS(msg.header.stamp);
  obj.originalReceivedTimestamp = obj.timestamp;

  bool incompatible = false;
  const sensor_msgs::msg::PointField *x_field = nullptr, *y_field = nullptr, *z_field = nullptr,
                                     *i_field = nullptr, *ring_field = nullptr;

  for (unsigned int i = 0; i < msg.fields.size() && !incompatible; i++)
  {
    incompatible |= check_field(msg.fields[i], "x", &x_field);
    incompatible |= check_field(msg.fields[i], "y", &y_field);
    incompatible |= check_field(msg.fields[i], "z", &z_field);
    incompatible |= check_field(msg.fields[i], "ring", &ring_field);
    check_field(msg.fields[i], "intensity", &i_field);
  }

  if (incompatible || (!x_field || !y_field || !z_field || !ring_field))
  {
    return false;
  }

  // 1st: go through the scan and find ring count:
  uint16_t ring_min = 0, ring_max = 0;

  for (std::size_t row = 0; row < msg.height; ++row)
  {
    const unsigned char* row_data = &msg.data[row * msg.row_step];
    for (std::size_t col = 0; col < msg.width; ++col)
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
  if (obj.rangeResolution == 0)
  {
    obj.rangeResolution = 1e-2;
  }

  if (i_field)
  {
    obj.intensityImage.resize(obj.rowCount, obj.columnCount);
    obj.intensityImage.fill(0);
  }
  else
  {
    obj.intensityImage.resize(0, 0);
  }

  if (inputCloudIsOrganized)
  {
    obj.organizedPoints.resize(obj.rowCount, obj.columnCount);
  }

  // If not, memcpy each group of contiguous fields separately
  for (std::size_t row = 0; row < msg.height; ++row)
  {
    const unsigned char* row_data = &msg.data[row * msg.row_step];
    for (std::size_t col = 0; col < msg.width; ++col)
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

      if (inputCloudIsOrganized)
      {
        obj.organizedPoints(ring_id, az_idx) = localPt;
      }
    }
  }

  return true;
}
