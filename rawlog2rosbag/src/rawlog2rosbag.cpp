/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2026, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

// ===========================================================================
//  Program: rawlog2rosbag
//  Intention: Convert an MRPT rawlog dataset into a ROS 2 bag (mcap, sqlite3),
//             the opposite of rosbag2rawlog.
// ===========================================================================

#include <mrpt/containers/yaml.h>
#include <mrpt/io/CCompressedInputStream.h>
#include <mrpt/maps/CGenericPointsMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/obs/CActionRobotMovement3D.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservationComment.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CObservationRobotPose.h>
#include <mrpt/obs/CObservationStereoImages.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/poses/CPose3DInterpolator.h>
#include <mrpt/ros2bridge/camera_info.h>
#include <mrpt/ros2bridge/gps.h>
#include <mrpt/ros2bridge/image.h>
#include <mrpt/ros2bridge/imu.h>
#include <mrpt/ros2bridge/laser_scan.h>
#include <mrpt/ros2bridge/odometry.h>
#include <mrpt/ros2bridge/point_cloud2.h>
#include <mrpt/ros2bridge/pose.h>
#include <mrpt/ros2bridge/time.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>
#include <mrpt/system/string_utils.h>
#include <mrpt/version.h>

#include <CLI/CLI.hpp>

// ROS:
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/qos.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_storage/topic_metadata.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

// std:
#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <map>
#include <optional>
#include <set>
#include <string>

namespace
{
struct Cli
{
  CLI::App app{"rawlog2rosbag (ROS 2) version " + mrpt::system::MRPT_getVersion()};

  std::string input_file;
  std::string output_dir;
  std::string config_file;
  std::string storage_id = "mcap";
  std::string storage_preset = "zstd_fast";
  std::string externals_dir;
  std::string base_link_frame = "base_link";
  std::string odom_frame = "odom";
  std::string image_mode = "compressed";
  int jpeg_quality = 95;
  std::string gt_tum_file;
  std::string gt_topic = "/ground_truth";
  std::string gt_frame = "map";
  bool no_latched_qos = false;
  bool overwrite = false;

  Cli()
  {
    app.add_option("-i,--input", input_file, "Input dataset (*.rawlog)")
        ->required()
        ->check(CLI::ExistingFile);
    app.add_option("-o,--output", output_dir, "Output bag directory")->required();
    app.add_option(
           "-c,--config", config_file,
           "Optional YAML file with per-sensorLabel overrides (ns, frame_id, skip)")
        ->check(CLI::ExistingFile);
    app.add_option("--storage-id", storage_id, "rosbag2 storage plugin (mcap|sqlite3)")
        ->default_val("mcap");
    app.add_option(
           "--storage-preset", storage_preset,
           "rosbag2 storage preset profile. For mcap: zstd_fast, zstd_small, fastwrite or "
           "an empty string for the plugin defaults")
        ->default_val("zstd_fast");
    app.add_option(
        "--externals-dir", externals_dir,
        "Directory of externally-stored images (default: auto-detect next to the rawlog)");
    app.add_option("-b,--base-link", base_link_frame, "Robot (vehicle) frame")
        ->default_val("base_link");
    app.add_option("--odom-frame", odom_frame, "Odometry frame")->default_val("odom");
    app.add_option(
           "--image-mode", image_mode,
           "Images as 'compressed' (sensor_msgs/CompressedImage, JPEG/PNG files copied "
           "verbatim) or 'raw' (sensor_msgs/Image)")
        ->check(CLI::IsMember({"compressed", "raw"}))
        ->default_val("compressed");
    app.add_option(
           "--jpeg-quality", jpeg_quality,
           "JPEG quality, for images that must be encoded in 'compressed' mode")
        ->default_val(95);
    app.add_option(
           "--gt-tum", gt_tum_file,
           "Optional ground-truth trajectory (TUM format: t x y z qx qy qz qw), written as "
           "nav_msgs/Odometry")
        ->check(CLI::ExistingFile);
    app.add_option("--gt-topic", gt_topic, "Topic of the ground truth")
        ->default_val("/ground_truth");
    app.add_option("--gt-frame", gt_frame, "Reference frame of the ground truth")
        ->default_val("map");
    app.add_flag(
        "--no-latched-qos", no_latched_qos,
        "Do not store the transient_local QoS of /tf_static and /rawlog_comment. Bags "
        "written with QoS profiles by ROS 2 Jazzy or newer cannot be read by stock Humble.");
    app.add_flag("-w,--overwrite", overwrite, "Overwrite the output directory if it exists");
  }
};

// Message (topic) name suffixes of each kind of output, appended to the sensor namespace:
constexpr const char* SUFFIX_SCAN = "scan";
constexpr const char* SUFFIX_IMU = "imu";
constexpr const char* SUFFIX_FIX = "fix";
constexpr const char* SUFFIX_POINTS = "points";
constexpr const char* SUFFIX_ODOM = "odom";
constexpr const char* SUFFIX_POSE = "pose";
constexpr const char* TOPIC_COMMENT = "/rawlog_comment";

bool posesDiffer(const mrpt::poses::CPose3D& a, const mrpt::poses::CPose3D& b)
{
  constexpr double tolerance = 1e-6;
  const auto A = a.getHomogeneousMatrix();
  const auto B = b.getHomogeneousMatrix();
  for (int r = 0; r < 3; r++)
  {
    for (int c = 0; c < 4; c++)
    {
      if (std::abs(A(r, c) - B(r, c)) > tolerance)
      {
        return true;
      }
    }
  }
  return false;
}

// Valid ROS name from an MRPT sensor label: lower case, [a-z0-9_] only, not starting with a
// digit.
std::string labelToRosName(const std::string& label)
{
  std::string s;
  for (const char c : label)
  {
    const auto uc = static_cast<unsigned char>(c);
    s.push_back(std::isalnum(uc) ? static_cast<char>(std::tolower(uc)) : '_');
  }
  if (s.empty() || std::isdigit(static_cast<unsigned char>(s.front())))
  {
    s = "sensor_" + s;
  }
  return s;
}

struct SensorOutput
{
  std::string ns;  //!< Topic namespace, e.g. "/lms_sick_left"
  std::string frame_id;
  bool skip = false;
};

struct SensorStats
{
  std::string className;
  size_t count = 0;
  std::optional<mrpt::poses::CPose3D> firstPose;
  size_t poseChanges = 0;
};

class Converter
{
 public:
  explicit Converter(const Cli& cli) : cli_(cli) {}

  void run()
  {
    loadConfig();
    setImagesDirectory();

    std::cout << "[rawlog2rosbag] Pass 1/2: scanning '" << cli_.input_file << "'...\n";
    forEachEntry([this](const mrpt::obs::CObservation::Ptr& o) { scanObservation(*o); }, {});
    printSummaryOfInput();

    openWriter();
    writeStaticTransforms();

    std::cout << "[rawlog2rosbag] Pass 2/2: writing '" << cli_.output_dir << "'...\n";
    forEachEntry(
        [this](const mrpt::obs::CObservation::Ptr& o) { writeObservation(*o); },
        [this](const mrpt::obs::CActionCollection& a) { writeAction(a); });

    writeGroundTruth();
    writer_.reset();  // close the bag

    printSummaryOfOutput();
  }

 private:
  const Cli& cli_;
  std::map<std::string, SensorOutput> userOutputs_;
  std::map<std::string, SensorStats> stats_;
  std::map<std::string, mrpt::poses::CPose3D> staticFrames_;  //!< frame -> pose wrt base link
  std::optional<mrpt::Clock::time_point> firstStamp_;
  std::optional<mrpt::Clock::time_point> lastStamp_;

  std::unique_ptr<rosbag2_cpp::Writer> writer_;
  std::set<std::string> createdTopics_;
  std::map<std::string, size_t> writtenPerTopic_;
  std::map<std::string, size_t> unsupported_;  //!< class name -> count

  // Odometry integrated from the actions of SensoryFrame-based rawlogs:
  mrpt::poses::CPose3D actionOdometry_;
  std::optional<std::string> odomTfLabel_;

  using ObsCallback = std::function<void(const mrpt::obs::CObservation::Ptr&)>;
  using ActionCallback = std::function<void(const mrpt::obs::CActionCollection&)>;

  void loadConfig()
  {
    if (cli_.config_file.empty())
    {
      return;
    }
    const auto cfg = mrpt::containers::yaml::FromFile(cli_.config_file);
    if (!cfg.has("sensors"))
    {
      return;
    }
    for (const auto& [k, v] : cfg["sensors"].asMap())
    {
      const mrpt::containers::yaml node(v);
      const auto label = k.as<std::string>();
      SensorOutput out;
      out.ns = node.getOrDefault<std::string>("ns", "");
      out.frame_id = node.getOrDefault<std::string>("frame_id", "");
      out.skip = node.getOrDefault<bool>("skip", false);
      userOutputs_[label] = out;
    }
  }

  void setImagesDirectory() const
  {
    const std::string dir = cli_.externals_dir.empty()
                                ? mrpt::obs::CRawlog::detectImagesDirectory(cli_.input_file)
                                : cli_.externals_dir;
    mrpt::img::CImage::setImagesPathBase(dir);
  }

  /// Topic namespace and frame of a sensor label, with the user overrides applied.
  SensorOutput outputFor(const std::string& label) const
  {
    SensorOutput out;
    const std::string name = labelToRosName(label);
    out.ns = "/" + name;
    out.frame_id = name;
    if (const auto it = userOutputs_.find(label); it != userOutputs_.end())
    {
      if (!it->second.ns.empty())
      {
        out.ns = it->second.ns;
      }
      if (!it->second.frame_id.empty())
      {
        out.frame_id = it->second.frame_id;
      }
      out.skip = it->second.skip;
    }
    return out;
  }

  /// Iterates over all entries (observations, or actions + sensory frames) of the rawlog.
  void forEachEntry(const ObsCallback& onObs, const ActionCallback& onAction) const
  {
    mrpt::io::CCompressedInputStream f(cli_.input_file);
    auto arch = mrpt::serialization::archiveFrom(f);
    const auto fileSize = static_cast<double>(f.getTotalBytesCount());  // NOLINT

    size_t entry = 0;
    size_t lastPct = 0;
    for (;;)
    {
      mrpt::obs::CActionCollection::Ptr acts;
      mrpt::obs::CSensoryFrame::Ptr sf;
      mrpt::obs::CObservation::Ptr obs;
      if (!mrpt::obs::CRawlog::getActionObservationPairOrObservation(arch, acts, sf, obs, entry))
      {
        break;
      }
      if (obs)
      {
        onObs(obs);
      }
      if (acts && onAction)
      {
        onAction(*acts);
      }
      if (sf)
      {
        for (const auto& o : *sf)
        {
          onObs(o);
        }
      }

      const auto pct = static_cast<size_t>(100.0 * f.getPosition() / std::max(1.0, fileSize));
      if (pct >= lastPct + 10)
      {
        lastPct = pct;
        std::cout << "  " << pct << "%\n" << std::flush;
      }
    }
  }

  // ------------------------------------------------------------------
  //  Pass 1
  // ------------------------------------------------------------------
  void scanObservation(const mrpt::obs::CObservation& o)
  {
    auto& st = stats_[o.sensorLabel];
    st.className = o.GetRuntimeClass()->className;
    st.count++;

    // Comments may carry an unrelated timestamp (e.g. when the file was written):
    if (dynamic_cast<const mrpt::obs::CObservationComment*>(&o) != nullptr)
    {
      return;
    }

    if (o.timestamp != INVALID_TIMESTAMP)
    {
      firstStamp_ = firstStamp_ ? std::min(*firstStamp_, o.timestamp) : o.timestamp;
      lastStamp_ = lastStamp_ ? std::max(*lastStamp_, o.timestamp) : o.timestamp;
    }

    if (dynamic_cast<const mrpt::obs::CObservationOdometry*>(&o) != nullptr)
    {
      if (!odomTfLabel_)
      {
        odomTfLabel_ = o.sensorLabel;
      }
      return;  // no sensor frame
    }
    if (dynamic_cast<const mrpt::obs::CObservationRobotPose*>(&o) != nullptr)
    {
      return;  // no sensor frame
    }

    const mrpt::poses::CPose3D pose = o.getSensorPose();
    if (!st.firstPose)
    {
      st.firstPose = pose;
      const auto out = outputFor(o.sensorLabel);
      if (!out.skip)
      {
        registerSensorFrames(o, out.frame_id, pose);
      }
    }
    else if (posesDiffer(*st.firstPose, pose))
    {
      st.poseChanges++;
    }
  }

  void registerSensorFrames(
      const mrpt::obs::CObservation& o, const std::string& frame, const mrpt::poses::CPose3D& pose)
  {
    if (const auto* st = dynamic_cast<const mrpt::obs::CObservationStereoImages*>(&o); st)
    {
      staticFrames_[frame + "_left"] = pose;
      staticFrames_[frame + "_right"] = pose + mrpt::poses::CPose3D(st->rightCameraPose);
      return;
    }
    staticFrames_[frame] = pose;
  }

  void printSummaryOfInput() const
  {
    std::cout << "[rawlog2rosbag] Input summary:\n";
    for (const auto& [label, st] : stats_)
    {
      std::cout << "  - '" << label << "': " << st.count << " x " << st.className;
      if (outputFor(label).skip)
      {
        std::cout << " (skipped)";
      }
      std::cout << "\n";
      if (st.poseChanges > 0)
      {
        std::cerr << "    Warning: its sensor pose changes " << st.poseChanges
                  << " times. Only the first one is written to /tf_static.\n";
      }
    }
    ASSERTMSG_(firstStamp_.has_value(), "The rawlog has no observation with a valid timestamp.");
    std::cout << std::fixed << std::setprecision(3)
              << "  Time span: " << mrpt::Clock::toDouble(*firstStamp_) << " - "
              << mrpt::Clock::toDouble(*lastStamp_) << " ("
              << mrpt::system::timeDifference(*firstStamp_, *lastStamp_) << " s)\n";
  }

  // ------------------------------------------------------------------
  //  Pass 2
  // ------------------------------------------------------------------
  // True if `path` is `dir` or is inside it.
  static bool isInside(const std::string& path, const std::string& dir)
  {
    const auto p = std::filesystem::weakly_canonical(path);
    const auto d = std::filesystem::weakly_canonical(dir);
    const auto rel = p.lexically_relative(d);
    return !rel.empty() && *rel.begin() != "..";
  }

  void openWriter()
  {
    ASSERTMSG_(
        !isInside(cli_.input_file, cli_.output_dir) &&
            !isInside(mrpt::img::CImage::getImagesPathBase(), cli_.output_dir),
        "The output directory must not contain the input rawlog nor its external files.");

    if (mrpt::system::directoryExists(cli_.output_dir))
    {
      ASSERTMSG_(
          cli_.overwrite, "Output directory '" + cli_.output_dir +
                              "' already exists. Use --overwrite to replace it.");
      mrpt::system::deleteFilesInDirectory(cli_.output_dir, true);
    }

    rosbag2_storage::StorageOptions so;
    so.uri = cli_.output_dir;
    so.storage_id = cli_.storage_id;
    if (cli_.storage_id == "mcap")
    {
      so.storage_preset_profile = cli_.storage_preset;
    }
    writer_ = std::make_unique<rosbag2_cpp::Writer>();
    writer_->open(so, rosbag2_cpp::ConverterOptions{"cdr", "cdr"});
  }

  template <class MSG>
  void write(const MSG& msg, const std::string& topic, const rclcpp::Time& t, bool latched = false)
  {
    if (createdTopics_.insert(topic).second)
    {
      rosbag2_storage::TopicMetadata md;
      md.name = topic;
      md.type = rosidl_generator_traits::name<MSG>();
      md.serialization_format = "cdr";
      if (latched && !cli_.no_latched_qos)
      {
#if ROSBAG2_PRE_IRON
        md.offered_qos_profiles =
            "- history: 1\n  depth: 1\n  reliability: 1\n  durability: 1\n"
            "  deadline:\n    sec: 2147483647\n    nsec: 4294967295\n"
            "  lifespan:\n    sec: 2147483647\n    nsec: 4294967295\n"
            "  liveliness: 1\n"
            "  liveliness_lease_duration:\n    sec: 2147483647\n    nsec: 4294967295\n"
            "  avoid_ros_namespace_conventions: false\n";
#else
        md.offered_qos_profiles.push_back(rclcpp::QoS(1).reliable().transient_local());
#endif
      }
      writer_->create_topic(md);
    }
    writer_->write(msg, topic, t);
    writtenPerTopic_[topic]++;
  }

  rclcpp::Time stampOf(const mrpt::obs::CObservation& o) const
  {
    return mrpt::ros2bridge::toROS(
        o.timestamp != INVALID_TIMESTAMP ? o.timestamp : firstStamp_.value());
  }

  std_msgs::msg::Header headerOf(const mrpt::obs::CObservation& o, const std::string& frame) const
  {
    std_msgs::msg::Header h;
    h.stamp = stampOf(o);
    h.frame_id = frame;
    return h;
  }

  void writeStaticTransforms()
  {
    tf2_msgs::msg::TFMessage tfs;
    for (const auto& [frame, pose] : staticFrames_)
    {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = mrpt::ros2bridge::toROS(*firstStamp_);
      tf.header.frame_id = cli_.base_link_frame;
      tf.child_frame_id = frame;
      const auto p = mrpt::ros2bridge::toROS_Pose(pose);
      tf.transform.translation.x = p.position.x;
      tf.transform.translation.y = p.position.y;
      tf.transform.translation.z = p.position.z;
      tf.transform.rotation = p.orientation;
      tfs.transforms.push_back(tf);
    }
    if (!tfs.transforms.empty())
    {
      write(tfs, "/tf_static", mrpt::ros2bridge::toROS(*firstStamp_), true);
    }
  }

  /// \param projTx The P[3] term of CameraInfo: -fx * baseline for the right camera of a
  /// stereo pair, zero otherwise.
  void writeImage(
      const mrpt::img::CImage& img,
      const mrpt::img::TCamera& cam,
      const std::string& ns,
      const std_msgs::msg::Header& h,
      double projTx = 0)
  {
    if (cli_.image_mode == "compressed")
    {
      write(
          mrpt::ros2bridge::toROS_Compressed(img, h, cli_.jpeg_quality),
          ns + "/image_raw/compressed", h.stamp);
    }
    else
    {
      write(mrpt::ros2bridge::toROS(img, h), ns + "/image_raw", h.stamp);
    }
    auto ci = mrpt::ros2bridge::toROS(cam, h);
    ci.p[3] = projTx;
    write(ci, ns + "/camera_info", h.stamp);
    img.unload();  // free memory of externally-stored images
  }

  void writeObservation(const mrpt::obs::CObservation& o)
  {
    using namespace mrpt::obs;

    const auto out = outputFor(o.sensorLabel);
    if (out.skip)
    {
      return;
    }

    // Externally stored data (e.g. point clouds) must be loaded before converting it. Images
    // are excluded: their files are copied without decoding them.
    const bool isImage = dynamic_cast<const CObservationImage*>(&o) != nullptr ||
                         dynamic_cast<const CObservationStereoImages*>(&o) != nullptr;
    if (!isImage)
    {
      o.load();
    }
    writeObservationContents(o, out);
    if (!isImage)
    {
      o.unload();
    }
  }

  void writeObservationContents(const mrpt::obs::CObservation& o, const SensorOutput& out)
  {
    using namespace mrpt::obs;

    const auto h = headerOf(o, out.frame_id);

    if (const auto* c = dynamic_cast<const CObservationComment*>(&o); c)
    {
      std_msgs::msg::String msg;
      msg.data = c->text;
      write(msg, TOPIC_COMMENT, mrpt::ros2bridge::toROS(*firstStamp_), true);
    }
    else if (const auto* scan = dynamic_cast<const CObservation2DRangeScan*>(&o); scan)
    {
      sensor_msgs::msg::LaserScan msg;
      if (mrpt::ros2bridge::toROS(*scan, msg))
      {
        msg.header = h;
        write(msg, out.ns + "/" + SUFFIX_SCAN, h.stamp);
      }
    }
    else if (const auto* imu = dynamic_cast<const CObservationIMU*>(&o); imu)
    {
      sensor_msgs::msg::Imu msg;
      mrpt::ros2bridge::toROS(*imu, h, msg);
      write(msg, out.ns + "/" + SUFFIX_IMU, h.stamp);
    }
    else if (const auto* gps = dynamic_cast<const CObservationGPS*>(&o); gps)
    {
      sensor_msgs::msg::NavSatFix msg;
      if (gps->hasMsgClass<gnss::Message_NMEA_GGA>() && mrpt::ros2bridge::toROS(*gps, h, msg))
      {
        write(msg, out.ns + "/" + SUFFIX_FIX, h.stamp);
      }
    }
    else if (const auto* im = dynamic_cast<const CObservationImage*>(&o); im)
    {
      writeImage(im->image, im->cameraParams, out.ns, h);
    }
    else if (const auto* st = dynamic_cast<const CObservationStereoImages*>(&o); st)
    {
      writeImage(
          st->imageLeft, st->leftCamera, out.ns + "/left", headerOf(o, out.frame_id + "_left"));
      if (st->hasImageRight)
      {
        writeImage(
            st->imageRight, st->rightCamera, out.ns + "/right",
            headerOf(o, out.frame_id + "_right"), -st->rightCamera.fx() * st->rightCameraPose.x());
      }
    }
    else if (const auto* pc = dynamic_cast<const CObservationPointCloud*>(&o); pc)
    {
      writePointCloud(pc->pointcloud, out.ns, h);
    }
    else if (const auto* odo = dynamic_cast<const CObservationOdometry*>(&o); odo)
    {
      auto hOdo = h;
      hOdo.frame_id = cli_.odom_frame;
      write(
          mrpt::ros2bridge::toROS(*odo, hOdo, cli_.base_link_frame), out.ns + "/" + SUFFIX_ODOM,
          h.stamp);
      if (odomTfLabel_ == o.sensorLabel)
      {
        writeOdomTf(mrpt::poses::CPose3D(odo->odometry), h.stamp);
      }
    }
    else if (const auto* rp = dynamic_cast<const CObservationRobotPose*>(&o); rp)
    {
      geometry_msgs::msg::PoseWithCovarianceStamped msg;
      msg.header = h;
      msg.header.frame_id = cli_.gt_frame;
      msg.pose = mrpt::ros2bridge::toROS_Pose(rp->pose);
      write(msg, out.ns + "/" + SUFFIX_POSE, h.stamp);
    }
    else
    {
      // Any other observation that can be inserted into a point cloud (3D range scans,
      // rotating lidars,...) is written as points in the vehicle frame:
      auto pts = mrpt::maps::CGenericPointsMap::Create();
      if (o.insertObservationInto(*pts) && !pts->empty())
      {
        auto hb = h;
        hb.frame_id = cli_.base_link_frame;
        writePointCloud(pts, out.ns, hb);
      }
      else
      {
        unsupported_[o.GetRuntimeClass()->className]++;
      }
    }
  }

  void writePointCloud(
      const mrpt::maps::CPointsMap::Ptr& pts, const std::string& ns, const std_msgs::msg::Header& h)
  {
    if (!pts)
    {
      return;
    }
    sensor_msgs::msg::PointCloud2 msg;
    bool ok = false;
    if (const auto g = std::dynamic_pointer_cast<mrpt::maps::CGenericPointsMap>(pts); g)
    {
      ok = mrpt::ros2bridge::toROS(*g, h, msg);
    }
    else
    {
      mrpt::maps::CSimplePointsMap s;
      s.insertAnotherMap(pts.get(), mrpt::poses::CPose3D::Identity());
      ok = mrpt::ros2bridge::toROS(s, h, msg);
    }
    if (ok)
    {
      write(msg, ns + "/" + SUFFIX_POINTS, h.stamp);
    }
  }

  void writeOdomTf(const mrpt::poses::CPose3D& pose, const rclcpp::Time& t)
  {
    tf2_msgs::msg::TFMessage tfs;
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = t;
    tf.header.frame_id = cli_.odom_frame;
    tf.child_frame_id = cli_.base_link_frame;
    const auto p = mrpt::ros2bridge::toROS_Pose(pose);
    tf.transform.translation.x = p.position.x;
    tf.transform.translation.y = p.position.y;
    tf.transform.translation.z = p.position.z;
    tf.transform.rotation = p.orientation;
    tfs.transforms.push_back(tf);
    write(tfs, "/tf", t);
  }

  // Actions of SensoryFrame-based rawlogs: integrated into an odometry topic + /tf.
  void writeAction(const mrpt::obs::CActionCollection& acts)
  {
    mrpt::Clock::time_point stamp = INVALID_TIMESTAMP;
    if (const auto a3 = acts.getActionByClass<mrpt::obs::CActionRobotMovement3D>(); a3)
    {
      actionOdometry_ = actionOdometry_ + a3->poseChange.mean;
      stamp = a3->timestamp;
    }
    else if (const auto a2 = acts.getBestMovementEstimation(); a2)
    {
      actionOdometry_ = actionOdometry_ + mrpt::poses::CPose3D(a2->poseChange->getMeanVal());
      stamp = a2->timestamp;
    }
    else
    {
      return;
    }
    if (stamp == INVALID_TIMESTAMP)
    {
      return;
    }
    if (odomTfLabel_)
    {
      return;  // the rawlog also has odometry observations: they own odom->base_link
    }

    const auto t = mrpt::ros2bridge::toROS(stamp);
    nav_msgs::msg::Odometry msg;
    msg.header.stamp = t;
    msg.header.frame_id = cli_.odom_frame;
    msg.child_frame_id = cli_.base_link_frame;
    msg.pose.pose = mrpt::ros2bridge::toROS_Pose(actionOdometry_);
    write(msg, "/odom", t);
    writeOdomTf(actionOdometry_, t);
  }

  void writeGroundTruth()
  {
    if (cli_.gt_tum_file.empty())
    {
      return;
    }
    mrpt::poses::CPose3DInterpolator gt;
    ASSERTMSG_(
        gt.loadFromTextFile_TUM(cli_.gt_tum_file),
        "Cannot load ground truth TUM file: " + cli_.gt_tum_file);

    for (const auto& [stamp, pose] : gt)
    {
      nav_msgs::msg::Odometry msg;
      msg.header.stamp = mrpt::ros2bridge::toROS(stamp);
      msg.header.frame_id = cli_.gt_frame;
      msg.child_frame_id = cli_.base_link_frame;
      msg.pose.pose = mrpt::ros2bridge::toROS_Pose(pose);
      write(msg, cli_.gt_topic, msg.header.stamp);
    }
  }

  void printSummaryOfOutput() const
  {
    std::cout << "[rawlog2rosbag] Written topics:\n";
    for (const auto& [topic, n] : writtenPerTopic_)
    {
      std::cout << "  - " << topic << ": " << n << " messages\n";
    }
    for (const auto& [className, n] : unsupported_)
    {
      std::cerr << "  Warning: " << n << " observations of unsupported class '" << className
                << "' were not written.\n";
    }
  }
};

}  // namespace

int main(int argc, char** argv)
{
  try
  {
    Cli cli;
    CLI11_PARSE(cli.app, argc, argv);

    Converter conv(cli);
    conv.run();
    return 0;
  }
  catch (const std::exception& e)
  {
    std::cerr << mrpt::exception_to_str(e) << std::endl;
    return 1;
  }
}
