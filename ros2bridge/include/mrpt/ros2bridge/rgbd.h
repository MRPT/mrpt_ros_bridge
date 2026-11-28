/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2026, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

/**
 * @file   rgbd.h
 * @brief  RGB-D and depth cameras (ROS 2 depth, intensity and camera_info
 *         topics) to mrpt::obs::CObservation3DRangeScan.
 */
#pragma once

#include <mrpt/containers/yaml.h>
#include <mrpt/img/CImage.h>
#include <mrpt/img/TCamera.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/poses/CPose3D.h>

#include <deque>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <string>
#include <utility>
#include <vector>

namespace tf2
{
class BufferCore;
}

namespace mrpt::ros2bridge
{
/** \addtogroup mrpt_ros2bridge_grp
 * @{ */

/** Decodes a ROS depth image into millimeters (0: no measurement).
 *  Encodings: 16UC1 / mono16 (millimeters, REP 118) and 32FC1 (meters; NaN, inf and
 *  non-positive values are "no measurement", as published by e.g. ZED cameras), in
 *  either byte order.
 *  \exception std::exception On unsupported encodings or malformed images.
 */
mrpt::math::CMatrix_u16 depthImageFromROS(const sensor_msgs::msg::Image& img);

/** Decodes an image_transport "compressedDepth" image (format
 *  `"<16UC1|32FC1>; compressedDepth <png|rvl>"`) into millimeters (0: no measurement).
 *  \exception std::exception On unsupported formats or corrupted data.
 */
mrpt::math::CMatrix_u16 depthImageFromROS(const sensor_msgs::msg::CompressedImage& img);

/** Intensity (color or gray) image of an RGB-D camera, as an 8-bit image.
 *  16-bit gray images (mono16 / 16UC1, e.g. time-of-flight infrared) are scaled by
 *  `scale16bit` and saturated at 255. Other encodings as in fromROS(Image).
 *  \exception std::exception If `scale16bit` is negative or not finite.
 */
mrpt::img::CImage intensityImageFromROS(
    const sensor_msgs::msg::Image& img, float scale16bit = 1.0f);

/** Same, for a compressed (JPEG or PNG) intensity image. 16-bit gray PNG images are
 *  scaled to 8 bits as raw ones.
 *  \exception std::exception If the image cannot be decoded, or `scale16bit` is negative
 *  or not finite.
 */
mrpt::img::CImage intensityImageFromROS(
    const sensor_msgs::msg::CompressedImage& img, float scale16bit = 1.0f);

/** Default camera_info topic of an image topic: the "camera_info" sibling of the image,
 *  after removing any image_transport suffix (e.g. `/cam/depth/image_raw/compressedDepth`
 *  -> `/cam/depth/camera_info`).
 */
std::string cameraInfoTopicFor(const std::string& imageTopic);

/** Whether an image topic is rectified, by its name (`image_rect...`). */
bool isRectifiedImageTopic(const std::string& imageTopic);

/** Whether an image topic name ends in an image_transport compressed transport
 *  ("/compressed" or "/compressedDepth"), so its type is sensor_msgs/CompressedImage. */
bool isCompressedImageTopic(const std::string& imageTopic);

/** Builds mrpt::obs::CObservation3DRangeScan observations out of the ROS topics of an
 *  RGB-D (or depth only) camera: a depth image, an optional intensity image (color or
 *  infrared) and the camera_info of each. Raw or compressed images, so it works with
 *  live topics and with bags of Azure Kinect, ZED, RealSense, OAK-D, Orbbec... cameras.
 *
 *  Depth and intensity frames are paired by header stamp. The observation keeps the
 *  sensor geometry, following the MRPT conventions:
 *  - `rangeImage` and `cameraParams`: the depth image (millimeters) and its intrinsics.
 *  - `intensityImage` and `cameraParamsIntensity`: the intensity image and its
 *    intrinsics, which may differ from the depth ones (e.g. a color image with a higher
 *    resolution than a depth image registered to it).
 *  - `relativePoseIntensityWRTDepth`: from /tf (depth -> intensity optical frames).
 *    Depth already registered to the intensity camera (same frame, e.g. ZED or a
 *    RealSense "aligned_depth_to_color" topic) yields doDepthAndIntensityCamerasCoincide().
 *  - `sensorPose`: the depth camera on the robot, in the MRPT +X-forward convention,
 *    from /tf or from a fixed pose.
 *
 *  Frames that arrive before their camera_info, or whose /tf is not known yet, are
 *  dropped and counted.
 */
class RgbdAssembler
{
 public:
  struct Parameters
  {
    std::string sensorLabel;
    bool withIntensity = true;
    /** Maximum stamp difference between paired depth and intensity images [s] */
    double maxTimeOffset = 0.010;
    /** Stored in CObservation3DRangeScan::maxRange [m] */
    float maxRange = 10.0f;
    /** Unpaired images kept waiting for their counterpart, per stream */
    size_t maxQueueLength = 10;
    /** Scale of 16-bit intensity images into 8 bits (saturated) */
    float intensity16bitScale = 1.0f;
    /** Whether the depth and intensity images are rectified: then their intrinsics come
     *  from the camera_info P matrix, without distortion (see fromROS(CameraInfo)). */
    bool depthRectified = false;
    bool intensityRectified = false;
    mrpt::obs::CObservation3DRangeScan::TIntensityChannelID intensityChannel =
        mrpt::obs::CObservation3DRangeScan::CH_VISIBLE;
    /** If set, used as `sensorPose` instead of /tf (depth camera, +X forward) */
    std::optional<mrpt::poses::CPose3D> fixedSensorPose;
  };

  using Obs = std::vector<mrpt::obs::CObservation::Ptr>;

  /** \param tf Source of the sensor and camera extrinsics (static transforms).
   *  \param baseLinkFrame Robot frame for `sensorPose`. */
  RgbdAssembler(Parameters p, std::shared_ptr<const tf2::BufferCore> tf, std::string baseLinkFrame);

  [[nodiscard]] const Parameters& parameters() const { return params_; }

  void onDepthCameraInfo(const sensor_msgs::msg::CameraInfo& info);
  void onIntensityCameraInfo(const sensor_msgs::msg::CameraInfo& info);

  /** Each returns the assembled observation, if this image completes a frame.
   *  \exception std::exception On images that cannot be decoded. */
  Obs onDepthImage(const sensor_msgs::msg::Image& img);
  Obs onDepthImage(const sensor_msgs::msg::CompressedImage& img);
  Obs onIntensityImage(const sensor_msgs::msg::Image& img);
  Obs onIntensityImage(const sensor_msgs::msg::CompressedImage& img);

  /** Counts the images still waiting for a counterpart (e.g. at the end of a bag) as
   *  dropped, and discards them. */
  void dropPending();

  /** Images discarded without a counterpart. */
  [[nodiscard]] size_t droppedUnpaired() const { return droppedUnpaired_; }
  /** Images dropped because their camera_info had not arrived yet. */
  [[nodiscard]] size_t droppedWithoutCameraInfo() const { return droppedNoInfo_; }
  /** Frames dropped because a /tf transform was not available. */
  [[nodiscard]] size_t droppedWithoutTf() const { return droppedNoTf_; }
  /** Error message of the last missing /tf transform. */
  [[nodiscard]] const std::string& lastTfError() const { return lastTfError_; }
  /** Smallest stamp difference to the other stream seen for the last image dropped
   *  without a counterpart [s] (infinity if none arrived while it waited). */
  [[nodiscard]] double lastEvictedClosestDt() const { return lastEvictedClosestDt_; }

  /** Pose of a camera optical frame (+Z forward, ROS REP 103) in the MRPT 3D range
   *  scan sensor frame (+X forward): (0,0,0,-90deg,0,-90deg). */
  static const mrpt::poses::CPose3D& opticalFrameInSensorFrame();

 private:
  /** A decoded depth (millimeters) or intensity image, waiting to be paired. */
  struct Frame
  {
    double stamp = 0;
    std_msgs::msg::Header header;
    mrpt::math::CMatrix_u16 depth;
    mrpt::img::CImage intensity;
    /** Smallest stamp difference to any image of the other stream seen so far [s] */
    double closestDt = std::numeric_limits<double>::infinity();
  };

  Parameters params_;
  std::shared_ptr<const tf2::BufferCore> tf_;
  std::string baseLinkFrame_;

  std::optional<mrpt::img::TCamera> depthCamera_;
  std::optional<mrpt::img::TCamera> intensityCamera_;
  std::deque<Frame> depthQueue_;
  std::deque<Frame> intensityQueue_;
  size_t droppedUnpaired_ = 0;
  size_t droppedNoInfo_ = 0;
  size_t droppedNoTf_ = 0;
  std::string lastTfError_;
  double lastEvictedClosestDt_ = 0;

  /** Cached intensity-wrt-depth poses, by (depth frame, intensity frame) */
  std::map<std::pair<std::string, std::string>, mrpt::poses::CPose3D> intensityWrtDepth_;

  /** Whether the camera_info of an image has arrived; if not, counts it as dropped. */
  bool hasCameraInfo(bool isDepth);
  Obs onDepth(Frame&& f);
  /** Drops (and counts) the waiting frames older than a stamp. */
  void dropOlderThan(std::deque<Frame>& queue, double minStamp);
  Obs pairWith(Frame&& f, std::deque<Frame>& own, std::deque<Frame>& other, bool isDepth);
  /** nullptr if a camera_info or a /tf transform is still missing. */
  mrpt::obs::CObservation3DRangeScan::Ptr build(Frame&& depth, Frame* intensity);
  std::optional<mrpt::poses::CPose3D> lookupPose(
      const std::string& target, const std::string& source);
};

/** Topic names of an RGB-D camera. `intensity` is empty for a depth-only camera. */
struct RgbdTopics
{
  std::string depth;
  std::string depthCameraInfo;
  std::string intensity;
  std::string intensityCameraInfo;
};

/** Reads the YAML description of an RGB-D sensor, with the keys shared by all the
 *  MRPT and MOLA RGB-D inputs:
 *  - `topic`: depth image (sensor_msgs/Image, or CompressedImage "compressedDepth"
 *    if it ends in `/compressedDepth`).
 *  - `intensity_topic` (optional): color or infrared image (Image, or CompressedImage
 *    JPEG/PNG if it ends in `/compressed`).
 *  - `camera_info_topic`, `intensity_camera_info_topic` (optional): by default, the
 *    `camera_info` sibling of each image topic (cameraInfoTopicFor()).
 *  - `rectified`, `intensity_rectified` (optional): whether each image is rectified, by
 *    default from its topic name (isRectifiedImageTopic()).
 *  - `max_time_offset` [s], `max_range` [m], `max_queue_length`,
 *    `intensity_16bit_scale` and `intensity_channel` (`visible` or `ir`): see
 *    RgbdAssembler::Parameters.
 *  Only the sensor label and fixed sensor pose are left to the caller.
 */
std::pair<RgbdTopics, RgbdAssembler::Parameters> rgbdSensorFromYaml(
    const mrpt::containers::yaml& sensor, const std::string& sensorLabel);

/** @} */

}  // namespace mrpt::ros2bridge
