/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2026, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/ros2bridge/image.h>
#include <mrpt/ros2bridge/rgbd.h>

#include <cmath>
#include <cstring>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <limits>
#include <sensor_msgs/distortion_models.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <tf2/buffer_core.hpp>

namespace enc = sensor_msgs::image_encodings;
using mrpt::ros2bridge::RgbdAssembler;

namespace
{
std_msgs::msg::Header header(double t, const std::string& frame)
{
  std_msgs::msg::Header h;
  h.stamp.sec = static_cast<int32_t>(std::floor(t));
  h.stamp.nanosec = static_cast<uint32_t>(std::lround((t - std::floor(t)) * 1e9));
  h.frame_id = frame;
  return h;
}

sensor_msgs::msg::Image depth16(
    const std::vector<uint16_t>& mm,
    uint32_t w,
    uint32_t h,
    double t,
    const std::string& frame = "cam_depth")
{
  sensor_msgs::msg::Image img;
  img.header = header(t, frame);
  img.width = w;
  img.height = h;
  img.encoding = enc::TYPE_16UC1;
  img.step = 2 * w;
  img.data.resize(mm.size() * 2);
  std::memcpy(img.data.data(), mm.data(), img.data.size());
  return img;
}

sensor_msgs::msg::Image rgb8(uint32_t w, uint32_t h, double t, const std::string& frame)
{
  sensor_msgs::msg::Image img;
  img.header = header(t, frame);
  img.width = w;
  img.height = h;
  img.encoding = enc::RGB8;
  img.step = 3 * w;
  img.data.assign(static_cast<size_t>(img.step) * h, 100);
  return img;
}

sensor_msgs::msg::CameraInfo cameraInfo(uint32_t w, uint32_t h, const std::string& frame)
{
  sensor_msgs::msg::CameraInfo ci;
  ci.header.frame_id = frame;
  ci.width = w;
  ci.height = h;
  ci.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
  ci.d = {0.1, 0.0, 0.0, 0.0, 0.0};
  ci.k = {10.0 * w, 0, 0.5 * w, 0, 10.0 * w, 0.5 * h, 0, 0, 1};
  ci.p = {10.0 * w, 0, 0.5 * w, 0, 0, 10.0 * w, 0.5 * h, 0, 0, 0, 1, 0};
  return ci;
}

void addStaticTf(
    tf2::BufferCore& tf,
    const std::string& parent,
    const std::string& child,
    double x,
    double y,
    double z)
{
  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id = parent;
  t.child_frame_id = child;
  t.transform.translation.x = x;
  t.transform.translation.y = y;
  t.transform.translation.z = z;
  t.transform.rotation.w = 1.0;
  tf.setTransform(t, "test", true);
}

// --- Minimal encoders of the image_transport compressedDepth formats ---

uint32_t crc32(const uint8_t* data, size_t n, uint32_t crc = 0)
{
  crc = ~crc;
  for (size_t i = 0; i < n; i++)
  {
    crc ^= data[i];
    for (int k = 0; k < 8; k++)
    {
      crc = (crc >> 1) ^ (0xEDB88320U & (0U - (crc & 1U)));
    }
  }
  return ~crc;
}

void putBE32(std::vector<uint8_t>& v, uint32_t x)
{
  for (int s = 24; s >= 0; s -= 8)
  {
    v.push_back(static_cast<uint8_t>(x >> s));
  }
}

void pngChunk(std::vector<uint8_t>& out, const char* tag, const std::vector<uint8_t>& payload)
{
  putBE32(out, static_cast<uint32_t>(payload.size()));
  std::vector<uint8_t> body(tag, tag + 4);
  body.insert(body.end(), payload.begin(), payload.end());
  out.insert(out.end(), body.begin(), body.end());
  putBE32(out, crc32(body.data(), body.size()));
}

// 16-bit gray PNG, with uncompressed (stored) deflate blocks.
std::vector<uint8_t> png16(const std::vector<uint16_t>& px, uint32_t w, uint32_t h)
{
  std::vector<uint8_t> raw;
  for (uint32_t r = 0; r < h; r++)
  {
    raw.push_back(0);  // filter: none
    for (uint32_t c = 0; c < w; c++)
    {
      raw.push_back(static_cast<uint8_t>(px[r * w + c] >> 8));
      raw.push_back(static_cast<uint8_t>(px[r * w + c] & 0xff));
    }
  }
  std::vector<uint8_t> z = {0x78, 0x01};
  z.push_back(1);  // final stored block
  const auto len = static_cast<uint16_t>(raw.size());
  for (const uint16_t x : {len, static_cast<uint16_t>(~len)})
  {
    z.push_back(static_cast<uint8_t>(x & 0xff));
    z.push_back(static_cast<uint8_t>(x >> 8));
  }
  z.insert(z.end(), raw.begin(), raw.end());
  uint32_t a = 1;
  uint32_t b = 0;
  for (const uint8_t x : raw)
  {
    a = (a + x) % 65521U;
    b = (b + a) % 65521U;
  }
  putBE32(z, (b << 16) | a);

  std::vector<uint8_t> out = {0x89, 'P', 'N', 'G', '\r', '\n', 0x1a, '\n'};
  std::vector<uint8_t> ihdr;
  putBE32(ihdr, w);
  putBE32(ihdr, h);
  ihdr.insert(ihdr.end(), {16, 0, 0, 0, 0});  // 16-bit, gray
  pngChunk(out, "IHDR", ihdr);
  pngChunk(out, "IDAT", z);
  pngChunk(out, "IEND", {});
  return out;
}

// Port of compressed_depth_image_transport RvlCodec::CompressRVL (Wilson, 2017).
class RvlWriter
{
 public:
  void encodeVLE(int value)
  {
    do
    {
      int nibble = value & 0x7;
      if ((value >>= 3) != 0)
      {
        nibble |= 0x8;
      }
      word_ <<= 4;
      word_ |= static_cast<uint32_t>(nibble);
      if (++nibbles_ == 8)
      {
        flushWord();
      }
    } while (value != 0);
  }
  std::vector<uint8_t> finish()
  {
    if (nibbles_ != 0)
    {
      word_ <<= 4 * (8 - nibbles_);
      flushWord();
    }
    return out_;
  }

 private:
  void flushWord()
  {
    const auto* p = reinterpret_cast<const uint8_t*>(&word_);
    out_.insert(out_.end(), p, p + 4);
    nibbles_ = 0;
    word_ = 0;
  }
  std::vector<uint8_t> out_;
  uint32_t word_ = 0;
  int nibbles_ = 0;
};

std::vector<uint8_t> rvl(const std::vector<uint16_t>& px, uint32_t w, uint32_t h)
{
  RvlWriter wr;
  uint16_t previous = 0;
  size_t i = 0;
  while (i < px.size())
  {
    int zeros = 0;
    while (i < px.size() && px[i] == 0)
    {
      i++;
      zeros++;
    }
    wr.encodeVLE(zeros);
    int nonzeros = 0;
    for (size_t j = i; j < px.size() && px[j] != 0; j++)
    {
      nonzeros++;
    }
    wr.encodeVLE(nonzeros);
    for (int k = 0; k < nonzeros; k++)
    {
      const int delta = px[i] - previous;
      wr.encodeVLE((delta << 1) ^ (delta >> 31));
      previous = px[i++];
    }
  }
  std::vector<uint8_t> out(8);
  std::memcpy(out.data(), &w, 4);
  std::memcpy(out.data() + 4, &h, 4);
  const auto words = wr.finish();
  out.insert(out.end(), words.begin(), words.end());
  return out;
}

void expectPoseNear(const mrpt::poses::CPose3D& a, const mrpt::poses::CPose3D& b)
{
  EXPECT_NEAR(a.x(), b.x(), 1e-9);
  EXPECT_NEAR(a.y(), b.y(), 1e-9);
  EXPECT_NEAR(a.z(), b.z(), 1e-9);
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      EXPECT_NEAR(a.getRotationMatrix()(i, j), b.getRotationMatrix()(i, j), 1e-9);
    }
  }
}

sensor_msgs::msg::CompressedImage compressedDepth(
    const std::string& format,
    const std::vector<uint8_t>& payload,
    float quantA = 0,
    float quantB = 0)
{
  sensor_msgs::msg::CompressedImage msg;
  msg.format = format;
  msg.data.resize(12);
  const int32_t fmt = 0;
  std::memcpy(msg.data.data(), &fmt, 4);
  std::memcpy(msg.data.data() + 4, &quantA, 4);
  std::memcpy(msg.data.data() + 8, &quantB, 4);
  msg.data.insert(msg.data.end(), payload.begin(), payload.end());
  return msg;
}
}  // namespace

TEST(RGBD, DepthImage16And32Bits)
{
  const auto d16 = mrpt::ros2bridge::depthImageFromROS(depth16({0, 1000, 2500, 65535}, 2, 2, 0));
  ASSERT_EQ(d16.rows(), 2);
  ASSERT_EQ(d16.cols(), 2);
  EXPECT_EQ(d16(0, 0), 0);
  EXPECT_EQ(d16(0, 1), 1000);
  EXPECT_EQ(d16(1, 0), 2500);
  EXPECT_EQ(d16(1, 1), 65535);

  // 32FC1 meters (e.g. ZED): NaN, inf and non-positive values mean "no measurement"
  const std::vector<float> m = {
      1.5f, std::numeric_limits<float>::quiet_NaN(), -1.0f, std::numeric_limits<float>::infinity()};
  sensor_msgs::msg::Image img;
  img.width = 2;
  img.height = 2;
  img.encoding = enc::TYPE_32FC1;
  img.step = 8;
  img.data.resize(16);
  std::memcpy(img.data.data(), m.data(), 16);
  const auto d32 = mrpt::ros2bridge::depthImageFromROS(img);
  EXPECT_EQ(d32(0, 0), 1500);
  EXPECT_EQ(d32(0, 1), 0);
  EXPECT_EQ(d32(1, 0), 0);
  EXPECT_EQ(d32(1, 1), 0);

  img.encoding = enc::RGB8;
  EXPECT_ANY_THROW(mrpt::ros2bridge::depthImageFromROS(img));
}

TEST(RGBD, CompressedDepthPngAndRvl)
{
  constexpr uint32_t W = 5;
  constexpr uint32_t H = 3;
  std::vector<uint16_t> mm(W * H);
  for (size_t i = 0; i < mm.size(); i++)
  {
    mm[i] = (i % 4 == 0) ? 0 : static_cast<uint16_t>(500 + 37 * i);
  }

  for (const bool useRvl : {false, true})
  {
    const auto payload = useRvl ? rvl(mm, W, H) : png16(mm, W, H);
    const auto d = mrpt::ros2bridge::depthImageFromROS(compressedDepth(
        useRvl ? "16UC1; compressedDepth rvl" : "16UC1; compressedDepth png", payload));
    ASSERT_EQ(d.rows(), static_cast<int>(H)) << (useRvl ? "rvl" : "png");
    ASSERT_EQ(d.cols(), static_cast<int>(W));
    for (uint32_t r = 0; r < H; r++)
    {
      for (uint32_t c = 0; c < W; c++)
      {
        EXPECT_EQ(d(r, c), mm[r * W + c]) << (useRvl ? "rvl" : "png");
      }
    }
  }

  // 32FC1 is stored as quantized inverse depth (encoder: depthZ0 = 100, depthMax = 10):
  const float quantA = 100.0f * 101.0f;
  const float quantB = 1.0f - quantA / 10.0f;
  const std::vector<uint16_t> inv = {
      0, static_cast<uint16_t>(std::lround(quantA / 2.0f + quantB)),
      static_cast<uint16_t>(std::lround(quantA / 5.0f + quantB))};
  const auto d = mrpt::ros2bridge::depthImageFromROS(
      compressedDepth("32FC1; compressedDepth", png16(inv, 3, 1), quantA, quantB));
  EXPECT_EQ(d(0, 0), 0);
  EXPECT_NEAR(d(0, 1), 2000, 1);
  EXPECT_NEAR(d(0, 2), 5000, 3);

  // Corrupted or wrong data:
  EXPECT_ANY_THROW(mrpt::ros2bridge::depthImageFromROS(
      compressedDepth("16UC1; compressedDepth rvl", {1, 0, 0, 0, 1, 0, 0, 0})));
  sensor_msgs::msg::CompressedImage jpeg;
  jpeg.format = "jpeg";
  jpeg.data.resize(100);
  EXPECT_ANY_THROW(mrpt::ros2bridge::depthImageFromROS(jpeg));
}

TEST(RGBD, IntensityMono16Scaled)
{
  sensor_msgs::msg::Image img;
  img.width = 3;
  img.height = 1;
  img.encoding = enc::MONO16;
  img.step = 6;
  const std::vector<uint16_t> v = {10, 200, 3000};
  img.data.resize(6);
  std::memcpy(img.data.data(), v.data(), 6);
  const auto out = mrpt::ros2bridge::intensityImageFromROS(img, 0.5f);
  ASSERT_FALSE(out.isColor());
  EXPECT_EQ(out.at<uint8_t>(0, 0), 5);
  EXPECT_EQ(out.at<uint8_t>(1, 0), 100);
  EXPECT_EQ(out.at<uint8_t>(2, 0), 255);  // saturated
}

TEST(RGBD, CameraInfoTopicNames)
{
  using mrpt::ros2bridge::cameraInfoTopicFor;
  using mrpt::ros2bridge::isCompressedImageTopic;
  EXPECT_EQ(cameraInfoTopicFor("/k/depth/image_raw"), "/k/depth/camera_info");
  EXPECT_EQ(cameraInfoTopicFor("/k/rgb/image_raw/compressed"), "/k/rgb/camera_info");
  EXPECT_EQ(
      cameraInfoTopicFor("/zed/depth/depth_registered/compressedDepth"), "/zed/depth/camera_info");
  EXPECT_EQ(cameraInfoTopicFor("image"), "camera_info");
  EXPECT_TRUE(isCompressedImageTopic("/k/rgb/image_raw/compressed"));
  EXPECT_TRUE(isCompressedImageTopic("/k/depth/image_raw/compressedDepth"));
  EXPECT_FALSE(isCompressedImageTopic("/k/depth/image_raw"));
}

TEST(RGBD, AssemblerPairsDepthAndColor)
{
  auto tf = std::make_shared<tf2::BufferCore>();
  addStaticTf(*tf, "base_link", "cam_depth", 1.0, 2.0, 3.0);
  addStaticTf(*tf, "cam_depth", "cam_color", 0.05, 0.0, 0.0);

  RgbdAssembler::Parameters p;
  p.sensorLabel = "rgbd";
  p.maxTimeOffset = 0.01;
  RgbdAssembler a(p, tf, "base_link");

  const std::vector<uint16_t> mm = {0, 1000, 1200, 1300, 1400, 1500};
  // Before any camera_info: each image is dropped and counted.
  EXPECT_TRUE(a.onDepthImage(depth16(mm, 3, 2, 0.9)).empty());
  EXPECT_TRUE(a.onIntensityImage(rgb8(6, 4, 0.901, "cam_color")).empty());
  EXPECT_EQ(a.droppedWithoutCameraInfo(), 2U);

  a.onDepthCameraInfo(cameraInfo(3, 2, "cam_depth"));
  a.onIntensityCameraInfo(cameraInfo(6, 4, "cam_color"));

  // Depth first, then its color image 4 ms later:
  EXPECT_TRUE(a.onDepthImage(depth16(mm, 3, 2, 1.0)).empty());
  auto obs = a.onIntensityImage(rgb8(6, 4, 1.004, "cam_color"));
  ASSERT_EQ(obs.size(), 1U);
  auto o = std::dynamic_pointer_cast<mrpt::obs::CObservation3DRangeScan>(obs.front());
  ASSERT_TRUE(o);
  EXPECT_EQ(o->sensorLabel, "rgbd");
  EXPECT_NEAR(mrpt::Clock::toDouble(o->timestamp), 1.0, 1e-6);
  EXPECT_TRUE(o->hasRangeImage);
  EXPECT_EQ(o->rangeImage(0, 1), 1000);
  EXPECT_EQ(o->rangeImage(1, 2), 1500);
  EXPECT_FLOAT_EQ(o->rangeUnits, 1e-3f);
  EXPECT_TRUE(o->hasIntensityImage);
  EXPECT_EQ(o->intensityImage.getWidth(), 6);
  EXPECT_EQ(o->cameraParamsIntensity.ncols, 6U);
  EXPECT_NEAR(o->cameraParams.k1(), 0.1, 1e-12);

  const auto& optical = RgbdAssembler::opticalFrameInSensorFrame();
  // sensorPose: the depth optical frame on base_link, in the +X forward convention.
  const auto depthOptical = o->sensorPose + optical;
  expectPoseNear(depthOptical, mrpt::poses::CPose3D(1, 2, 3, 0, 0, 0));
  // The color camera, 5 cm to the right of the depth one (optical x axis):
  const auto colorInDepthOptical = (-optical) + o->relativePoseIntensityWRTDepth;
  expectPoseNear(colorInDepthOptical, mrpt::poses::CPose3D(0.05, 0, 0, 0, 0, 0));
  EXPECT_FALSE(o->doDepthAndIntensityCamerasCoincide());

  // Color first, then depth; and a color image too far in time is never paired.
  EXPECT_TRUE(a.onIntensityImage(rgb8(6, 4, 2.0, "cam_color")).empty());
  EXPECT_EQ(a.onDepthImage(depth16(mm, 3, 2, 2.003)).size(), 1U);
  EXPECT_TRUE(a.onIntensityImage(rgb8(6, 4, 3.0, "cam_color")).empty());
  EXPECT_TRUE(a.onDepthImage(depth16(mm, 3, 2, 3.05)).empty());

  // An unknown frame: dropped, with the reason.
  a.onDepthCameraInfo(cameraInfo(3, 2, "other"));
  EXPECT_TRUE(a.onDepthImage(depth16(mm, 3, 2, 4.0, "other")).empty());
  EXPECT_TRUE(a.onIntensityImage(rgb8(6, 4, 4.0, "cam_color")).empty());
  EXPECT_EQ(a.droppedWithoutTf(), 1U);
  EXPECT_FALSE(a.lastTfError().empty());
}

TEST(RGBD, AssemblerRegisteredDepthAndCompressedColor)
{
  // Depth registered to the color camera (same frame, e.g. ZED), JPEG color.
  auto tf = std::make_shared<tf2::BufferCore>();
  addStaticTf(*tf, "base_link", "cam", 0.0, 0.0, 0.5);
  RgbdAssembler::Parameters p;
  p.sensorLabel = "zed";
  RgbdAssembler a(p, tf, "base_link");
  a.onDepthCameraInfo(cameraInfo(3, 2, "cam"));
  a.onIntensityCameraInfo(cameraInfo(3, 2, "cam"));

  mrpt::img::CImage color(3, 2, mrpt::img::CH_RGB);
  color.filledRectangle({0, 0}, {2, 1}, mrpt::img::TColor(200, 50, 10));
  const auto jpeg = mrpt::ros2bridge::toROS_Compressed(color, header(5.0, "cam"));

  EXPECT_TRUE(a.onIntensityImage(jpeg).empty());
  const auto obs = a.onDepthImage(depth16({1000, 1000, 1000, 1000, 1000, 1000}, 3, 2, 5.0, "cam"));
  ASSERT_EQ(obs.size(), 1U);
  const auto o = std::dynamic_pointer_cast<mrpt::obs::CObservation3DRangeScan>(obs.front());
  ASSERT_TRUE(o);
  EXPECT_TRUE(o->doDepthAndIntensityCamerasCoincide());
  EXPECT_TRUE(o->intensityImage.isColor());
  EXPECT_EQ(o->intensityImage.getWidth(), 3);
}

TEST(RGBD, AssemblerEvictsUnpairedAndChecksCoincidingCameras)
{
  auto tf = std::make_shared<tf2::BufferCore>();
  addStaticTf(*tf, "base_link", "cam", 0.0, 0.0, 0.5);
  RgbdAssembler::Parameters p;
  RgbdAssembler a(p, tf, "base_link");
  a.onDepthCameraInfo(cameraInfo(3, 2, "cam"));
  a.onIntensityCameraInfo(cameraInfo(3, 2, "cam"));

  // A stream that never finds its counterpart is evicted and counted:
  for (int i = 0; i < 12; i++)
  {
    EXPECT_TRUE(
        a.onDepthImage(depth16(std::vector<uint16_t>(6, 1000), 3, 2, 10.0 + i, "cam")).empty());
  }
  EXPECT_EQ(a.droppedUnpaired(), 2U);
  EXPECT_TRUE(std::isinf(a.lastEvictedClosestDt()));

  // Same frame and image size, other intrinsics: a misconfiguration.
  auto other = cameraInfo(3, 2, "cam");
  other.k[0] *= 2;
  a.onIntensityCameraInfo(other);
  EXPECT_ANY_THROW(a.onIntensityImage(rgb8(3, 2, 21.0, "cam")));
}

TEST(RGBD, AssemblerDepthOnlyWithFixedPose)
{
  RgbdAssembler::Parameters p;
  p.sensorLabel = "depth";
  p.withIntensity = false;
  p.fixedSensorPose = mrpt::poses::CPose3D(0.1, 0.2, 0.3, 0, 0, 0);
  RgbdAssembler a(p, nullptr, "base_link");
  a.onDepthCameraInfo(cameraInfo(2, 1, "cam"));
  const auto obs = a.onDepthImage(depth16({700, 800}, 2, 1, 1.0, "cam"));
  ASSERT_EQ(obs.size(), 1U);
  const auto o = std::dynamic_pointer_cast<mrpt::obs::CObservation3DRangeScan>(obs.front());
  ASSERT_TRUE(o);
  EXPECT_FALSE(o->hasIntensityImage);
  EXPECT_NEAR(o->sensorPose.x(), 0.1, 1e-12);
}

TEST(RGBD, SensorFromYaml)
{
  const auto y = mrpt::containers::yaml::FromText(R"(
topic: /kinect/depth/image_raw/compressedDepth
intensity_topic: /kinect/ir/image_raw
intensity_channel: ir
intensity_16bit_scale: 0.5
max_time_offset: 0.02
max_queue_length: 20
)");
  const auto [t, p] = mrpt::ros2bridge::rgbdSensorFromYaml(y, "kinect");
  EXPECT_EQ(t.depthCameraInfo, "/kinect/depth/camera_info");
  EXPECT_EQ(t.intensityCameraInfo, "/kinect/ir/camera_info");
  EXPECT_TRUE(p.withIntensity);
  EXPECT_EQ(p.intensityChannel, mrpt::obs::CObservation3DRangeScan::CH_IR);
  EXPECT_FLOAT_EQ(p.intensity16bitScale, 0.5f);
  EXPECT_DOUBLE_EQ(p.maxTimeOffset, 0.02);
  EXPECT_EQ(p.sensorLabel, "kinect");
  EXPECT_EQ(p.maxQueueLength, 20U);

  EXPECT_ANY_THROW(mrpt::ros2bridge::rgbdSensorFromYaml(
      mrpt::containers::yaml::FromText("topic: /cam/depth\nintensity_channel: thermal"), "x"));
}

TEST(RGBD, RvlRejectsOverlongIntegers)
{
  // 1x1 image, then 16 nibbles with the continuation bit set: malformed.
  std::vector<uint8_t> payload(8);
  const uint32_t one = 1;
  std::memcpy(payload.data(), &one, 4);
  std::memcpy(payload.data() + 4, &one, 4);
  const uint32_t continuation = 0x88888888;
  for (int i = 0; i < 2; i++)
  {
    payload.resize(payload.size() + 4);
    std::memcpy(payload.data() + payload.size() - 4, &continuation, 4);
  }
  EXPECT_ANY_THROW(
      mrpt::ros2bridge::depthImageFromROS(compressedDepth("16UC1; compressedDepth rvl", payload)));
}

TEST(RGBD, RvlAllZeroFrame)
{
  // A frame without any depth return compresses to a single zero run:
  const uint32_t w = 640;
  const uint32_t h = 480;
  const auto payload = rvl(std::vector<uint16_t>(static_cast<size_t>(w) * h, 0), w, h);
  EXPECT_LT(payload.size(), 32U);
  const auto out =
      mrpt::ros2bridge::depthImageFromROS(compressedDepth("16UC1; compressedDepth rvl", payload));
  ASSERT_EQ(out.cols(), 640);
  ASSERT_EQ(out.rows(), 480);
  EXPECT_EQ(out.maxCoeff(), 0);
}

TEST(RGBD, DepthImageBigEndian)
{
  sensor_msgs::msg::Image img;
  img.width = 2;
  img.height = 1;
  img.is_bigendian = 1;
  img.encoding = enc::TYPE_16UC1;
  img.step = 4;
  img.data = {0x03, 0xE8, 0x00, 0x07};  // 1000, 7
  const auto d16 = mrpt::ros2bridge::depthImageFromROS(img);
  EXPECT_EQ(d16(0, 0), 1000);
  EXPECT_EQ(d16(0, 1), 7);

  img.encoding = enc::TYPE_32FC1;
  img.step = 8;
  img.data = {0x3F, 0xC0, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00};  // 1.5f, 2.0f
  const auto d32 = mrpt::ros2bridge::depthImageFromROS(img);
  EXPECT_EQ(d32(0, 0), 1500);
  EXPECT_EQ(d32(0, 1), 2000);
}

TEST(RGBD, DepthImageValidatedBeforeAllocating)
{
  sensor_msgs::msg::Image img;
  img.width = 10000;
  img.height = 10000;
  img.encoding = enc::TYPE_16UC1;
  img.step = 0;
  EXPECT_ANY_THROW(mrpt::ros2bridge::depthImageFromROS(img));
  img.step = 20000;  // a valid stride, but no data
  EXPECT_ANY_THROW(mrpt::ros2bridge::depthImageFromROS(img));
  img.encoding = "bgr8";
  EXPECT_ANY_THROW(mrpt::ros2bridge::depthImageFromROS(img));
}

TEST(RGBD, IntensityScaleValidated)
{
  sensor_msgs::msg::Image img;
  img.width = 1;
  img.height = 1;
  img.encoding = enc::MONO16;
  img.step = 2;
  img.data = {10, 0};
  EXPECT_ANY_THROW(mrpt::ros2bridge::intensityImageFromROS(img, -1.0f));
  EXPECT_ANY_THROW(
      mrpt::ros2bridge::intensityImageFromROS(img, std::numeric_limits<float>::quiet_NaN()));
  EXPECT_EQ(mrpt::ros2bridge::intensityImageFromROS(img, 0.0f).at<uint8_t>(0, 0), 0);

  RgbdAssembler::Parameters p;
  p.intensity16bitScale = -2.0f;
  p.fixedSensorPose = mrpt::poses::CPose3D();
  EXPECT_ANY_THROW(RgbdAssembler(p, nullptr, "base_link"));
}

TEST(RGBD, CompressedIntensity16BitPng)
{
  sensor_msgs::msg::CompressedImage msg;
  msg.format = "mono16; png compressed";
  msg.data = png16({10, 200, 3000}, 3, 1);
  const auto out = mrpt::ros2bridge::intensityImageFromROS(msg, 0.5f);
  ASSERT_FALSE(out.isColor());
  EXPECT_EQ(out.getPixelDepth(), mrpt::img::PixelDepth::D8U);
  EXPECT_EQ(out.at<uint8_t>(0, 0), 5);
  EXPECT_EQ(out.at<uint8_t>(1, 0), 100);
  EXPECT_EQ(out.at<uint8_t>(2, 0), 255);  // saturated
}

TEST(RGBD, AssemblerDropsImagesBeforeTheirCameraInfo)
{
  auto tf = std::make_shared<tf2::BufferCore>();
  addStaticTf(*tf, "base_link", "cam", 0.0, 0.0, 0.5);
  RgbdAssembler::Parameters p;
  RgbdAssembler a(p, tf, "base_link");
  const std::vector<uint16_t> mm(6, 1000);

  // Depth before its camera_info is dropped, even if the intensity one comes in time:
  EXPECT_TRUE(a.onDepthImage(depth16(mm, 3, 2, 1.0, "cam")).empty());
  a.onDepthCameraInfo(cameraInfo(3, 2, "cam"));
  a.onIntensityCameraInfo(cameraInfo(3, 2, "cam"));
  EXPECT_TRUE(a.onIntensityImage(rgb8(3, 2, 1.0, "cam")).empty());
  EXPECT_EQ(a.droppedWithoutCameraInfo(), 1U);

  // A depth image after its camera_info waits for its intensity image:
  EXPECT_TRUE(a.onDepthImage(depth16(mm, 3, 2, 2.0, "cam")).empty());
  EXPECT_EQ(a.onIntensityImage(rgb8(3, 2, 2.0, "cam")).size(), 1U);
}

TEST(RGBD, AssemblerKeepsFramesThatCanStillPair)
{
  auto tf = std::make_shared<tf2::BufferCore>();
  addStaticTf(*tf, "base_link", "cam", 0.0, 0.0, 0.5);
  RgbdAssembler::Parameters p;
  p.maxTimeOffset = 0.010;
  RgbdAssembler a(p, tf, "base_link");
  a.onDepthCameraInfo(cameraInfo(3, 2, "cam"));
  a.onIntensityCameraInfo(cameraInfo(3, 2, "cam"));
  const std::vector<uint16_t> mm(6, 1000);

  // Depth at 0 and 9 ms; intensity at 8 ms takes the 9 ms one, and intensity at
  // 9.5 ms can still pair with the 0 ms depth.
  EXPECT_TRUE(a.onDepthImage(depth16(mm, 3, 2, 10.000, "cam")).empty());
  EXPECT_TRUE(a.onDepthImage(depth16(mm, 3, 2, 10.009, "cam")).empty());
  EXPECT_EQ(a.onIntensityImage(rgb8(3, 2, 10.008, "cam")).size(), 1U);
  EXPECT_EQ(a.onIntensityImage(rgb8(3, 2, 10.0095, "cam")).size(), 1U);
  EXPECT_EQ(a.droppedUnpaired(), 0U);

  // A frame that can no longer pair is dropped and counted when a match happens:
  EXPECT_TRUE(a.onDepthImage(depth16(mm, 3, 2, 20.000, "cam")).empty());
  EXPECT_TRUE(a.onDepthImage(depth16(mm, 3, 2, 20.030, "cam")).empty());
  EXPECT_EQ(a.onIntensityImage(rgb8(3, 2, 20.030, "cam")).size(), 1U);
  EXPECT_EQ(a.droppedUnpaired(), 1U);

  // At the end of a stream, frames still waiting are dropped and counted too:
  EXPECT_TRUE(a.onDepthImage(depth16(mm, 3, 2, 30.000, "cam")).empty());
  a.dropPending();
  EXPECT_EQ(a.droppedUnpaired(), 2U);
  EXPECT_TRUE(a.onIntensityImage(rgb8(3, 2, 30.000, "cam")).empty());
}

TEST(RGBD, SensorFromYamlRectified)
{
  using mrpt::ros2bridge::rgbdSensorFromYaml;
  const auto [t1, p1] = rgbdSensorFromYaml(
      mrpt::containers::yaml::FromText(
          "topic: /rs/depth/image_rect_raw\nintensity_topic: /rs/color/image_raw"),
      "rs");
  EXPECT_TRUE(p1.depthRectified);
  EXPECT_FALSE(p1.intensityRectified);

  const auto [t2, p2] = rgbdSensorFromYaml(
      mrpt::containers::yaml::FromText(
          "topic: /zed/depth/depth_registered\nrectified: true\n"
          "intensity_topic: /zed/rgb/image\nintensity_rectified: true"),
      "zed");
  EXPECT_TRUE(p2.depthRectified);
  EXPECT_TRUE(p2.intensityRectified);
}
