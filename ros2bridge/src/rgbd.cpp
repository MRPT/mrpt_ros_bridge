/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2026, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/core/bits_math.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/ros2bridge/camera_info.h>
#include <mrpt/ros2bridge/image.h>
#include <mrpt/ros2bridge/pose.h>
#include <mrpt/ros2bridge/rgbd.h>
#include <mrpt/ros2bridge/time.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <sensor_msgs/image_encodings.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/buffer_core.hpp>
#include <tf2/exceptions.hpp>
#include <tf2/time.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using mrpt::math::CMatrix_u16;

namespace
{
double stampOf(const std_msgs::msg::Header& h)
{
  return static_cast<double>(h.stamp.sec) + 1e-9 * static_cast<double>(h.stamp.nanosec);
}

// Depth in meters to millimeters, 0 for "no measurement":
uint16_t metersToMillimeters(float d)
{
  constexpr float kMaxMeters = std::numeric_limits<uint16_t>::max() * 1e-3f;
  return (std::isfinite(d) && d > 0 && d < kMaxMeters)
             ? static_cast<uint16_t>(std::lround(d * 1e3f))
             : 0;
}

// Reads the 32-bit words of an RVL stream, with bounds checks.
class RvlReader
{
 public:
  RvlReader(const uint8_t* data, size_t length) : data_(data), length_(length) {}

  int decodeVLE()
  {
    unsigned int nibble = 0;
    int value = 0;
    int bits = 29;
    do
    {
      if (nibblesLeft_ == 0)
      {
        ASSERTMSG_(pos_ + 4 <= length_, "Truncated RVL depth image");
        std::memcpy(&word_, data_ + pos_, 4);
        pos_ += 4;
        nibblesLeft_ = 8;
      }
      nibble = word_ & 0xf0000000;
      value |= static_cast<int>((nibble << 1) >> bits);
      word_ <<= 4;
      nibblesLeft_--;
      bits -= 3;
    } while ((nibble & 0x80000000) != 0);
    return value;
  }

 private:
  const uint8_t* data_;
  size_t length_;
  size_t pos_ = 0;
  uint32_t word_ = 0;
  int nibblesLeft_ = 0;
};

// RVL codec (Wilson, "Fast lossless depth image compression", 2017), as written by
// compressed_depth_image_transport: width and height (uint32) and the RVL words.
CMatrix_u16 decodeRVL(const uint8_t* data, size_t length)
{
  ASSERTMSG_(length >= 8, "Truncated RVL depth image");
  uint32_t cols = 0;
  uint32_t rows = 0;
  std::memcpy(&cols, data, 4);
  std::memcpy(&rows, data + 4, 4);
  const uint64_t numPixels = static_cast<uint64_t>(rows) * cols;
  // The best RVL compression ratio is ~4x: larger sizes mean corrupted data.
  ASSERTMSG_(
      rows > 0 && cols > 0 && numPixels <= 10 * static_cast<uint64_t>(length),
      "Malformed RVL depth image");

  CMatrix_u16 out(static_cast<int>(rows), static_cast<int>(cols));
  uint16_t* dst = &out(0, 0);
  const uint16_t* const end = dst + numPixels;
  RvlReader rvl(data + 8, length - 8);
  uint16_t previous = 0;
  while (dst != end)
  {
    const int zeros = rvl.decodeVLE();
    ASSERTMSG_(zeros >= 0 && zeros <= end - dst, "Malformed RVL depth image");
    std::fill(dst, dst + zeros, 0);
    dst += zeros;
    const int nonzeros = rvl.decodeVLE();
    ASSERTMSG_(nonzeros >= 0 && nonzeros <= end - dst, "Malformed RVL depth image");
    for (int i = 0; i < nonzeros; i++)
    {
      const int positive = rvl.decodeVLE();
      const int delta = (positive >> 1) ^ -(positive & 1);
      previous = static_cast<uint16_t>(previous + delta);
      *dst++ = previous;
    }
  }
  return out;
}

CMatrix_u16 decodePNG16(const uint8_t* data, size_t length)
{
  mrpt::img::CImage img;
  ASSERTMSG_(
      img.loadFromEncodedBuffer(data, length, mrpt::img::CH_GRAY, mrpt::img::PixelDepth::D16U),
      "Cannot decode the PNG of a compressedDepth image");
  const int w = img.getWidth();
  const int h = img.getHeight();
  CMatrix_u16 out(h, w);
  for (int r = 0; r < h; r++)
  {
    std::memcpy(&out(r, 0), img.ptrLine<uint16_t>(r), sizeof(uint16_t) * static_cast<size_t>(w));
  }
  return out;
}
}  // namespace

namespace mrpt::ros2bridge
{
CMatrix_u16 depthImageFromROS(const sensor_msgs::msg::Image& img)
{
  namespace enc = sensor_msgs::image_encodings;

  const auto h = static_cast<int>(img.height);
  const auto w = static_cast<int>(img.width);
  ASSERTMSG_(!img.is_bigendian, "Big-endian depth images are not supported");

  CMatrix_u16 out(h, w);
  if (img.encoding == enc::TYPE_16UC1 || img.encoding == enc::MONO16)
  {
    ASSERT_GE_(img.step, img.width * sizeof(uint16_t));
    ASSERT_GE_(img.data.size(), static_cast<size_t>(img.step) * img.height);
    for (int r = 0; r < h; r++)
    {
      std::memcpy(
          &out(r, 0), img.data.data() + static_cast<size_t>(r) * img.step,
          sizeof(uint16_t) * img.width);
    }
  }
  else if (img.encoding == enc::TYPE_32FC1)
  {
    ASSERT_GE_(img.step, img.width * sizeof(float));
    ASSERT_GE_(img.data.size(), static_cast<size_t>(img.step) * img.height);
    for (int r = 0; r < h; r++)
    {
      const uint8_t* row = img.data.data() + static_cast<size_t>(r) * img.step;
      for (int c = 0; c < w; c++)
      {
        float d = 0;
        std::memcpy(&d, row + sizeof(float) * static_cast<size_t>(c), sizeof(float));
        out(r, c) = metersToMillimeters(d);
      }
    }
  }
  else
  {
    THROW_EXCEPTION_FMT(
        "Unsupported depth image encoding '%s' (expected 16UC1, mono16 or 32FC1)",
        img.encoding.c_str());
  }
  return out;
}

CMatrix_u16 depthImageFromROS(const sensor_msgs::msg::CompressedImage& img)
{
  namespace enc = sensor_msgs::image_encodings;

  // Format: "<encoding>; compressedDepth [png|rvl]" (png if not given).
  const auto sep = img.format.find(';');
  std::string encoding = img.format.substr(0, sep);
  encoding.erase(std::remove(encoding.begin(), encoding.end(), ' '), encoding.end());
  const std::string rest = sep == std::string::npos ? std::string() : img.format.substr(sep);
  ASSERTMSG_(
      rest.find("compressedDepth") != std::string::npos,
      "Not a compressedDepth image (format: '" + img.format + "')");
  const bool isRVL = rest.find("compressedDepth rvl") != std::string::npos;

  // compressed_depth_image_transport::ConfigHeader: int32 format, float depthParam[2].
  constexpr size_t kHeaderSize = 12;
  ASSERTMSG_(img.data.size() > kHeaderSize, "Truncated compressedDepth image");
  float quantA = 0;
  float quantB = 0;
  std::memcpy(&quantA, img.data.data() + 4, sizeof(float));
  std::memcpy(&quantB, img.data.data() + 8, sizeof(float));
  const uint8_t* payload = img.data.data() + kHeaderSize;
  const size_t payloadSize = img.data.size() - kHeaderSize;

  CMatrix_u16 raw = isRVL ? decodeRVL(payload, payloadSize) : decodePNG16(payload, payloadSize);

  if (encoding == enc::TYPE_16UC1 || encoding == enc::MONO16)
  {
    return raw;  // millimeters
  }
  ASSERTMSG_(
      encoding == enc::TYPE_32FC1, "Unsupported compressedDepth encoding '" + encoding + "'");
  // 32FC1 images are stored as quantized inverse depth:
  for (int r = 0; r < raw.rows(); r++)
  {
    for (int c = 0; c < raw.cols(); c++)
    {
      const uint16_t invDepth = raw(r, c);
      raw(r, c) =
          invDepth == 0 ? 0 : metersToMillimeters(quantA / (static_cast<float>(invDepth) - quantB));
    }
  }
  return raw;
}

mrpt::img::CImage intensityImageFromROS(const sensor_msgs::msg::Image& img, float scale16bit)
{
  namespace enc = sensor_msgs::image_encodings;
  if (img.encoding != enc::MONO16 && img.encoding != enc::TYPE_16UC1)
  {
    return fromROS(img);
  }

  ASSERT_GE_(img.step, img.width * sizeof(uint16_t));
  ASSERT_GE_(img.data.size(), static_cast<size_t>(img.step) * img.height);
  const auto w = static_cast<int32_t>(img.width);
  const auto h = static_cast<int32_t>(img.height);
  mrpt::img::CImage out(w, h, mrpt::img::CH_GRAY);
  for (int32_t r = 0; r < h; r++)
  {
    const uint8_t* row = img.data.data() + static_cast<size_t>(r) * img.step;
    auto* dst = out.ptrLine<uint8_t>(r);
    for (int32_t c = 0; c < w; c++)
    {
      const uint8_t* px = row + 2 * static_cast<size_t>(c);
      const uint16_t v = img.is_bigendian ? static_cast<uint16_t>((px[0] << 8) | px[1])
                                          : static_cast<uint16_t>((px[1] << 8) | px[0]);
      dst[c] = static_cast<uint8_t>(std::min(255.0f, static_cast<float>(v) * scale16bit));
    }
  }
  return out;
}

mrpt::img::CImage intensityImageFromROS(const sensor_msgs::msg::CompressedImage& img)
{
  auto out = fromROS(img);
  ASSERTMSG_(
      out.getWidth() > 0,
      "Cannot decode the compressed intensity image (format: '" + img.format + "')");
  return out;
}

std::string cameraInfoTopicFor(const std::string& imageTopic)
{
  std::string t = imageTopic;
  for (const char* transport : {"/compressedDepth", "/compressed", "/theora", "/zstd"})
  {
    const std::string suffix = transport;
    if (t.size() > suffix.size() && t.compare(t.size() - suffix.size(), suffix.size(), suffix) == 0)
    {
      t.resize(t.size() - suffix.size());
      break;
    }
  }
  const auto pos = t.find_last_of('/');
  return (pos == std::string::npos ? std::string() : t.substr(0, pos + 1)) + "camera_info";
}

bool isCompressedImageTopic(const std::string& imageTopic)
{
  for (const std::string suffix : {"/compressed", "/compressedDepth"})
  {
    if (imageTopic.size() > suffix.size() &&
        imageTopic.compare(imageTopic.size() - suffix.size(), suffix.size(), suffix) == 0)
    {
      return true;
    }
  }
  return false;
}

std::pair<RgbdTopics, RgbdAssembler::Parameters> rgbdSensorFromYaml(
    const mrpt::containers::yaml& sensor, const std::string& sensorLabel)
{
  RgbdTopics t;
  t.depth = sensor["topic"].as<std::string>();
  t.depthCameraInfo =
      sensor.getOrDefault<std::string>("camera_info_topic", cameraInfoTopicFor(t.depth));
  t.intensity = sensor.getOrDefault<std::string>("intensity_topic", "");
  if (!t.intensity.empty())
  {
    t.intensityCameraInfo = sensor.getOrDefault<std::string>(
        "intensity_camera_info_topic", cameraInfoTopicFor(t.intensity));
  }

  RgbdAssembler::Parameters p;
  p.sensorLabel = sensorLabel;
  p.withIntensity = !t.intensity.empty();
  p.maxTimeOffset = sensor.getOrDefault<double>("max_time_offset", p.maxTimeOffset);
  p.maxRange = sensor.getOrDefault<float>("max_range", p.maxRange);
  p.maxQueueLength = sensor.getOrDefault<size_t>("max_queue_length", p.maxQueueLength);
  p.intensity16bitScale =
      sensor.getOrDefault<float>("intensity_16bit_scale", p.intensity16bitScale);
  const auto channel = sensor.getOrDefault<std::string>("intensity_channel", "visible");
  ASSERTMSG_(
      channel == "visible" || channel == "ir",
      "RGB-D sensor '" + sensorLabel + "': intensity_channel must be 'visible' or 'ir'");
  p.intensityChannel = channel == "ir" ? mrpt::obs::CObservation3DRangeScan::CH_IR
                                       : mrpt::obs::CObservation3DRangeScan::CH_VISIBLE;
  return {t, p};
}

// -----------------------------------------------------------------------
//  RgbdAssembler
// -----------------------------------------------------------------------
const mrpt::poses::CPose3D& RgbdAssembler::opticalFrameInSensorFrame()
{
  using namespace mrpt;  // _deg
  static const mrpt::poses::CPose3D p(0, 0, 0, -90.0_deg, 0.0_deg, -90.0_deg);
  return p;
}

RgbdAssembler::RgbdAssembler(
    Parameters p, std::shared_ptr<const tf2::BufferCore> tf, std::string baseLinkFrame) :
    params_(std::move(p)), tf_(std::move(tf)), baseLinkFrame_(std::move(baseLinkFrame))
{
  ASSERT_(tf_ || params_.fixedSensorPose);
  ASSERT_GT_(params_.maxQueueLength, 0U);
}

void RgbdAssembler::onDepthCameraInfo(const sensor_msgs::msg::CameraInfo& info)
{
  depthCamera_ = fromROS(info);
}

void RgbdAssembler::onIntensityCameraInfo(const sensor_msgs::msg::CameraInfo& info)
{
  intensityCamera_ = fromROS(info);
}

RgbdAssembler::Obs RgbdAssembler::onDepthImage(const sensor_msgs::msg::Image& img)
{
  return onDepth({stampOf(img.header), img.header, depthImageFromROS(img), {}});
}

RgbdAssembler::Obs RgbdAssembler::onDepthImage(const sensor_msgs::msg::CompressedImage& img)
{
  return onDepth({stampOf(img.header), img.header, depthImageFromROS(img), {}});
}

RgbdAssembler::Obs RgbdAssembler::onIntensityImage(const sensor_msgs::msg::Image& img)
{
  return pairWith(
      {stampOf(img.header),
       img.header,
       {},
       intensityImageFromROS(img, params_.intensity16bitScale)},
      intensityQueue_, depthQueue_, false);
}

RgbdAssembler::Obs RgbdAssembler::onIntensityImage(const sensor_msgs::msg::CompressedImage& img)
{
  return pairWith(
      {stampOf(img.header), img.header, {}, intensityImageFromROS(img)}, intensityQueue_,
      depthQueue_, false);
}

RgbdAssembler::Obs RgbdAssembler::onDepth(Frame&& f)
{
  if (!params_.withIntensity)
  {
    auto obs = build(std::move(f), nullptr);
    return obs ? Obs{obs} : Obs{};
  }
  return pairWith(std::move(f), depthQueue_, intensityQueue_, true);
}

RgbdAssembler::Obs RgbdAssembler::pairWith(
    Frame&& f, std::deque<Frame>& own, std::deque<Frame>& other, bool isDepth)
{
  const double t = f.stamp;

  auto best = other.end();
  double bestDt = params_.maxTimeOffset;
  for (auto it = other.begin(); it != other.end(); ++it)
  {
    const double dt = std::abs(it->stamp - t);
    if (dt <= bestDt)
    {
      best = it;
      bestDt = dt;
    }
  }

  if (best == other.end())
  {
    own.push_back(std::move(f));
    while (own.size() > params_.maxQueueLength)
    {
      // Report how close its best counterpart was, to tell a too strict
      // maxTimeOffset from a stream that is not arriving at all:
      double closest = std::numeric_limits<double>::infinity();
      for (const auto& o : other)
      {
        closest = std::min(closest, std::abs(o.stamp - own.front().stamp));
      }
      lastEvictedClosestDt_ = closest;
      droppedUnpaired_++;
      own.pop_front();
    }
    return {};
  }

  Frame match = std::move(*best);
  // Streams arrive in time order: older entries can no longer be paired.
  other.erase(other.begin(), best + 1);
  while (!own.empty() && own.front().stamp <= t)
  {
    own.pop_front();
  }

  auto obs = isDepth ? build(std::move(f), &match) : build(std::move(match), &f);
  return obs ? Obs{obs} : Obs{};
}

std::optional<mrpt::poses::CPose3D> RgbdAssembler::lookupPose(
    const std::string& target, const std::string& source)
{
  if (!tf_)
  {
    lastTfError_ = "no /tf source and no fixed sensor pose";
    return std::nullopt;
  }
  try
  {
    const auto tfMsg = tf_->lookupTransform(target, source, tf2::TimePointZero);
    tf2::Transform t;
    tf2::fromMsg(tfMsg.transform, t);
    return mrpt::ros2bridge::fromROS(t);
  }
  catch (const tf2::TransformException& ex)
  {
    lastTfError_ = "'" + target + "' -> '" + source + "': " + ex.what();
    return std::nullopt;
  }
}

mrpt::obs::CObservation3DRangeScan::Ptr RgbdAssembler::build(Frame&& depth, Frame* intensity)
{
  // Drivers usually publish camera_info after the first images:
  if (!depthCamera_ || (intensity != nullptr && !intensityCamera_))
  {
    droppedNoInfo_++;
    return {};
  }

  // ROS camera images are referenced to an optical frame (+Z forward):
  const auto& depthFrame = depth.header.frame_id;
  std::optional<mrpt::poses::CPose3D> sensorPose = params_.fixedSensorPose;
  if (!sensorPose)
  {
    if (const auto p = lookupPose(baseLinkFrame_, depthFrame); p)
    {
      sensorPose = *p + (-opticalFrameInSensorFrame());
    }
  }

  // Camera extrinsics are static: look them up only once.
  std::optional<mrpt::poses::CPose3D> intensityWrtDepth;
  if (intensity != nullptr)
  {
    const auto key = std::make_pair(depthFrame, intensity->header.frame_id);
    if (auto it = intensityWrtDepth_.find(key); it != intensityWrtDepth_.end())
    {
      intensityWrtDepth = it->second;
    }
    else if (const auto depthToIntensity = key.first == key.second
                                               ? std::optional(mrpt::poses::CPose3D::Identity())
                                               : lookupPose(key.first, key.second);
             depthToIntensity)
    {
      intensityWrtDepth = opticalFrameInSensorFrame() + *depthToIntensity;
      intensityWrtDepth_.emplace(key, *intensityWrtDepth);
    }
  }

  if (!sensorPose || (intensity != nullptr && !intensityWrtDepth))
  {
    droppedNoTf_++;
    return {};
  }

  auto obs = mrpt::obs::CObservation3DRangeScan::Create();
  obs->sensorLabel = params_.sensorLabel;
  obs->timestamp = mrpt::ros2bridge::fromROS(depth.header.stamp);
  obs->range_is_depth = true;
  obs->maxRange = params_.maxRange;
  obs->sensorPose = *sensorPose;

  obs->hasRangeImage = true;
  obs->rangeUnits = 1e-3f;
  obs->rangeImage = std::move(depth.depth);
  obs->cameraParams = *depthCamera_;
  ASSERTMSG_(
      obs->cameraParams.ncols == static_cast<uint32_t>(obs->rangeImage.cols()) &&
          obs->cameraParams.nrows == static_cast<uint32_t>(obs->rangeImage.rows()),
      "Depth image of sensor '" + params_.sensorLabel + "' and its camera_info differ in size");

  if (intensity == nullptr)
  {
    obs->cameraParamsIntensity = obs->cameraParams;
    return obs;
  }

  obs->intensityImage = std::move(intensity->intensity);
  obs->hasIntensityImage = true;
  obs->intensityImageChannel = params_.intensityChannel;
  obs->cameraParamsIntensity = *intensityCamera_;
  obs->relativePoseIntensityWRTDepth = *intensityWrtDepth;
  // Coinciding cameras with images of the same size are read pixel by pixel, so they
  // must share their intrinsics too (otherwise, likely a wrong topic or frame_id):
  if (obs->doDepthAndIntensityCamerasCoincide() &&
      obs->intensityImage.getWidth() == static_cast<int32_t>(obs->rangeImage.cols()) &&
      obs->intensityImage.getHeight() == static_cast<int32_t>(obs->rangeImage.rows()))
  {
    const auto& dc = obs->cameraParams;
    const auto& ic = obs->cameraParamsIntensity;
    const auto same = [](double a, double b) { return std::abs(a - b) <= 1e-3 * std::abs(a); };
    ASSERTMSG_(
        same(dc.fx(), ic.fx()) && same(dc.fy(), ic.fy()) && same(dc.cx(), ic.cx()) &&
            same(dc.cy(), ic.cy()),
        "Depth and intensity cameras of sensor '" + params_.sensorLabel +
            "' coincide and have the same image size, but different intrinsics");
  }
#if !defined(MRPT_HAS_3DSCAN_INTENSITY_PROJECTION)
  // Older MRPT versions read the intensity image at depth pixel coordinates whenever both
  // cameras coincide, even if the image sizes differ:
  ASSERTMSG_(
      !obs->doDepthAndIntensityCamerasCoincide() ||
          (obs->intensityImage.getWidth() == static_cast<int32_t>(obs->rangeImage.cols()) &&
           obs->intensityImage.getHeight() == static_cast<int32_t>(obs->rangeImage.rows())),
      "Sensor '" + params_.sensorLabel +
          "': depth registered to an intensity image of another size needs a newer MRPT "
          "(MRPT_HAS_3DSCAN_INTENSITY_PROJECTION)");
#endif
  ASSERTMSG_(
      obs->cameraParamsIntensity.ncols == static_cast<uint32_t>(obs->intensityImage.getWidth()) &&
          obs->cameraParamsIntensity.nrows ==
              static_cast<uint32_t>(obs->intensityImage.getHeight()),
      "Intensity image of sensor '" + params_.sensorLabel + "' and its camera_info differ in size");

  return obs;
}

}  // namespace mrpt::ros2bridge
