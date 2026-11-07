/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2026, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

// Writes a synthetic rawlog, converts it with rawlog2rosbag, and checks the bag contents.

#include <gtest/gtest.h>
#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/io/vector_loadsave.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservationComment.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CObservationStereoImages.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/ros2bridge/camera_info.h>
#include <mrpt/ros2bridge/laser_scan.h>
#include <mrpt/ros2bridge/time.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/filesystem.h>

#include <cstdlib>
#include <fstream>
#include <map>
#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

namespace
{
constexpr double T0 = 1226321738.0;
constexpr size_t NUM_SCANS = 5;

int64_t bagStamp(const rosbag2_storage::SerializedBagMessage& m)
{
#if ROSBAG2_PRE_IRON
  return m.time_stamp;
#else
  return m.recv_timestamp;
#endif
}

template <class MSG>
MSG deserialize(const rosbag2_storage::SerializedBagMessage& m)
{
  rclcpp::SerializedMessage ser(*m.serialized_data);
  rclcpp::Serialization<MSG> s;
  MSG msg;
  s.deserialize_message(&ser, &msg);
  return msg;
}

struct Fixture
{
  Fixture(const Fixture&) = delete;
  Fixture& operator=(const Fixture&) = delete;

  std::string dir = mrpt::system::getTempFileName() + "_rawlog2rosbag_test";
  std::string rawlog = dir + "/dataset.rawlog";
  std::string imagesDir = dir + "/Images";
  std::string imageFile = "CAMERA_1.jpg";
  std::string bag = dir + "/bag";
  mrpt::poses::CPose3D scanPose{0.5, 0.1, 0.2, 0.3, 0, 0};

  Fixture()
  {
    mrpt::system::createDirectory(dir);
    mrpt::system::createDirectory(imagesDir);

    mrpt::img::CImage img(16, 8, mrpt::img::CH_RGB);
    img.filledRectangle({0, 0}, {15, 7}, mrpt::img::TColor(100, 50, 25));
    EXPECT_TRUE(img.saveToFile(imagesDir + "/" + imageFile));

    mrpt::io::CFileGZOutputStream f(rawlog);
    auto arch = mrpt::serialization::archiveFrom(f);

    // A comment with an unrelated timestamp, as in old datasets:
    auto comment = mrpt::obs::CObservationComment::Create();
    comment->timestamp = mrpt::Clock::fromDouble(T0 + 1e6);
    comment->text = "dataset notes";
    arch << *comment;

    for (size_t i = 0; i < NUM_SCANS; i++)
    {
      const double t = T0 + 0.1 * static_cast<double>(i);

      auto scan = mrpt::obs::CObservation2DRangeScan::Create();
      scan->sensorLabel = "LASER 1";
      scan->timestamp = mrpt::Clock::fromDouble(t);
      scan->sensorPose = scanPose;
      scan->aperture = static_cast<float>(M_PI);
      scan->maxRange = 30.0f;
      scan->resizeScan(181);
      for (size_t k = 0; k < 181; k++)
      {
        scan->setScanRange(k, 2.0f + 0.01f * static_cast<float>(k));
        scan->setScanRangeValidity(k, k != 10);
      }
      arch << *scan;

      auto imu = mrpt::obs::CObservationIMU::Create();
      imu->sensorLabel = "IMU";
      imu->timestamp = mrpt::Clock::fromDouble(t + 0.01);
      imu->set(mrpt::obs::IMU_YAW, 0.5);
      imu->set(mrpt::obs::IMU_PITCH, 0.0);
      imu->set(mrpt::obs::IMU_ROLL, 0.0);
      imu->set(mrpt::obs::IMU_Z_ACC, 9.8);
      arch << *imu;
    }

    auto gps = mrpt::obs::CObservationGPS::Create();
    gps->sensorLabel = "GPS";
    gps->timestamp = mrpt::Clock::fromDouble(T0 + 0.2);
    mrpt::obs::gnss::Message_NMEA_GGA gga;
    gga.fields.latitude_degrees = 36.71;
    gga.fields.longitude_degrees = -4.47;
    gga.fields.altitude_meters = 40.0;
    gga.fields.fix_quality = 4;
    gps->setMsg(gga);
    arch << *gps;

    auto cam = mrpt::obs::CObservationImage::Create();
    cam->sensorLabel = "CAMERA";
    cam->timestamp = mrpt::Clock::fromDouble(T0 + 0.3);
    cam->cameraParams.ncols = 16;
    cam->cameraParams.nrows = 8;
    cam->cameraParams.setIntrinsicParamsFromValues(10, 11, 8, 4);
    cam->cameraParams.setDistortionPlumbBob(-0.3, 0.1, 0, 0);
    cam->image.setExternalStorage(imageFile);
    arch << *cam;

    // A point cloud stored in an external text file:
    {
      std::ofstream pts(imagesDir + "/points.txt");
      pts << "1 2 3\n4 5 6\n7 8 9\n";
    }
    auto pc = mrpt::obs::CObservationPointCloud::Create();
    pc->sensorLabel = "LIDAR";
    pc->timestamp = mrpt::Clock::fromDouble(T0 + 0.35);
    pc->setAsExternalStorage(
        "points.txt", mrpt::obs::CObservationPointCloud::ExternalStorageFormat::PlainTextFile);
    arch << *pc;

    // A stereo pair with a 0.12 m baseline:
    auto stereo = mrpt::obs::CObservationStereoImages::Create();
    stereo->sensorLabel = "STEREO";
    stereo->timestamp = mrpt::Clock::fromDouble(T0 + 0.36);
    stereo->imageLeft = mrpt::img::CImage(16, 8, mrpt::img::CH_GRAY);
    stereo->imageRight = mrpt::img::CImage(16, 8, mrpt::img::CH_GRAY);
    stereo->hasImageRight = true;
    stereo->leftCamera.setIntrinsicParamsFromValues(10, 10, 8, 4);
    stereo->rightCamera.setIntrinsicParamsFromValues(10, 10, 8, 4);
    stereo->rightCameraPose = mrpt::poses::CPose3DQuat(mrpt::poses::CPose3D(0.12, 0, 0, 0, 0, 0));
    arch << *stereo;

    auto odo = mrpt::obs::CObservationOdometry::Create();
    odo->sensorLabel = "ODOMETRY";
    odo->timestamp = mrpt::Clock::fromDouble(T0 + 0.4);
    odo->odometry = mrpt::poses::CPose2D(1, 2, 0.1);
    arch << *odo;
  }

  ~Fixture() { mrpt::system::deleteFilesInDirectory(dir, true); }
};
}  // namespace

TEST(rawlog2rosbag, RoundTrip)
{
  const char* exe = std::getenv("RAWLOG2ROSBAG_EXE");
  ASSERT_NE(exe, nullptr);

  Fixture fx;
  const std::string cmd = std::string(exe) + " -i " + fx.rawlog + " -o " + fx.bag + " -w";
  ASSERT_EQ(std::system(cmd.c_str()), 0) << cmd;

  rosbag2_cpp::Reader reader;
  reader.open(fx.bag);

  std::map<std::string, size_t> counts;
  std::map<std::string, std::string> types;
  for (const auto& t : reader.get_all_topics_and_types())
  {
    types[t.name] = t.type;
  }
  EXPECT_EQ(types["/laser_1/scan"], "sensor_msgs/msg/LaserScan");
  EXPECT_EQ(types["/camera/image_raw/compressed"], "sensor_msgs/msg/CompressedImage");
  EXPECT_EQ(types["/odometry/odom"], "nav_msgs/msg/Odometry");

  bool tfStaticChecked = false;
  while (reader.has_next())
  {
    const auto m = reader.read_next();
    counts[m->topic_name]++;
    const double t = static_cast<double>(bagStamp(*m)) * 1e-9;
    EXPECT_GE(t, T0 - 1e-3) << m->topic_name;
    EXPECT_LT(t, T0 + 1.0) << m->topic_name;

    if (m->topic_name == "/laser_1/scan")
    {
      const auto msg = deserialize<sensor_msgs::msg::LaserScan>(*m);
      EXPECT_EQ(msg.header.frame_id, "laser_1");
      ASSERT_EQ(msg.ranges.size(), 181u);
      EXPECT_TRUE(std::isinf(msg.ranges[10]));
      EXPECT_FLOAT_EQ(msg.ranges[20], 2.2f);

      mrpt::obs::CObservation2DRangeScan back;
      ASSERT_TRUE(mrpt::ros2bridge::fromROS(msg, fx.scanPose, back));
      EXPECT_FALSE(back.getScanRangeValidity(10));
      EXPECT_TRUE(back.getScanRangeValidity(20));
    }
    else if (m->topic_name == "/imu/imu")
    {
      const auto msg = deserialize<sensor_msgs::msg::Imu>(*m);
      EXPECT_NEAR(msg.orientation.z, std::sin(0.25), 1e-9);
    }
    else if (m->topic_name == "/gps/fix")
    {
      const auto msg = deserialize<sensor_msgs::msg::NavSatFix>(*m);
      EXPECT_DOUBLE_EQ(msg.latitude, 36.71);
    }
    else if (m->topic_name == "/camera/image_raw/compressed")
    {
      const auto msg = deserialize<sensor_msgs::msg::CompressedImage>(*m);
      std::vector<uint8_t> fileBytes;
      ASSERT_TRUE(mrpt::io::loadBinaryFile(fileBytes, fx.imagesDir + "/" + fx.imageFile));
      EXPECT_EQ(msg.data, fileBytes);
    }
    else if (m->topic_name == "/camera/camera_info")
    {
      const auto cam = mrpt::ros2bridge::fromROS(deserialize<sensor_msgs::msg::CameraInfo>(*m));
      EXPECT_DOUBLE_EQ(cam.fy(), 11.0);
      EXPECT_DOUBLE_EQ(cam.k1(), -0.3);
    }
    else if (m->topic_name == "/lidar/points")
    {
      EXPECT_EQ(deserialize<sensor_msgs::msg::PointCloud2>(*m).width, 3u);
    }
    else if (m->topic_name == "/stereo/right/camera_info")
    {
      EXPECT_NEAR(deserialize<sensor_msgs::msg::CameraInfo>(*m).p[3], -10.0 * 0.12, 1e-9);
    }
    else if (m->topic_name == "/stereo/left/camera_info")
    {
      EXPECT_EQ(deserialize<sensor_msgs::msg::CameraInfo>(*m).p[3], 0.0);
    }
    else if (m->topic_name == "/rawlog_comment")
    {
      EXPECT_EQ(deserialize<std_msgs::msg::String>(*m).data, "dataset notes");
    }
    else if (m->topic_name == "/tf_static")
    {
      const auto msg = deserialize<tf2_msgs::msg::TFMessage>(*m);
      for (const auto& tf : msg.transforms)
      {
        if (tf.child_frame_id == "laser_1")
        {
          EXPECT_EQ(tf.header.frame_id, "base_link");
          EXPECT_NEAR(tf.transform.translation.x, 0.5, 1e-9);
          EXPECT_NEAR(tf.transform.rotation.z, std::sin(0.15), 1e-9);
          tfStaticChecked = true;
        }
      }
    }
  }
  EXPECT_TRUE(tfStaticChecked);
  EXPECT_EQ(counts["/laser_1/scan"], NUM_SCANS);
  EXPECT_EQ(counts["/imu/imu"], NUM_SCANS);
  EXPECT_EQ(counts["/gps/fix"], 1u);
  EXPECT_EQ(counts["/camera/image_raw/compressed"], 1u);
  EXPECT_EQ(counts["/camera/camera_info"], 1u);
  EXPECT_EQ(counts["/odometry/odom"], 1u);
  EXPECT_EQ(counts["/tf"], 1u);
  EXPECT_EQ(counts["/rawlog_comment"], 1u);
  EXPECT_EQ(counts["/lidar/points"], 1u);
  EXPECT_EQ(counts["/stereo/left/image_raw/compressed"], 1u);
  EXPECT_EQ(counts["/stereo/left/camera_info"], 1u);
  EXPECT_EQ(counts["/stereo/right/camera_info"], 1u);
}

// An output directory holding the input data must be rejected, not wiped with --overwrite.
TEST(rawlog2rosbag, RejectsOutputContainingInput)
{
  const char* exe = std::getenv("RAWLOG2ROSBAG_EXE");
  ASSERT_NE(exe, nullptr);

  Fixture fx;
  const std::string errFile = fx.dir + "/stderr.txt";
  const std::string cmd =
      std::string(exe) + " -i " + fx.rawlog + " -o " + fx.dir + " -w 2> " + errFile;
  EXPECT_NE(std::system(cmd.c_str()), 0) << cmd;
  EXPECT_NE(
      mrpt::io::file_get_contents(errFile).find("must not contain the input"), std::string::npos);
  EXPECT_TRUE(mrpt::system::fileExists(fx.rawlog));
  EXPECT_TRUE(mrpt::system::fileExists(fx.imagesDir + "/" + fx.imageFile));
}

// SensoryFrame-based rawlogs: the actions become an integrated /odom topic (+ /tf).
TEST(rawlog2rosbag, SensoryFrameRawlog)
{
  const char* exe = std::getenv("RAWLOG2ROSBAG_EXE");
  ASSERT_NE(exe, nullptr);

  const std::string dir = mrpt::system::getTempFileName() + "_rawlog2rosbag_sf_test";
  mrpt::system::createDirectory(dir);
  const std::string rawlog = dir + "/sf.rawlog";
  constexpr size_t NUM_STEPS = 4;
  {
    mrpt::io::CFileGZOutputStream f(rawlog);
    auto arch = mrpt::serialization::archiveFrom(f);
    for (size_t i = 0; i < NUM_STEPS; i++)
    {
      const auto t = mrpt::Clock::fromDouble(T0 + static_cast<double>(i));

      mrpt::obs::CActionRobotMovement2D mov;
      mov.computeFromOdometry(mrpt::poses::CPose2D(1.0, 0, M_PI / 2), {});
      mov.timestamp = t;
      mrpt::obs::CActionCollection acts;
      acts.insert(mov);

      auto scan = mrpt::obs::CObservation2DRangeScan::Create();
      scan->sensorLabel = "LASER";
      scan->timestamp = t;
      scan->resizeScanAndAssign(10, 1.0f, true);
      mrpt::obs::CSensoryFrame sf;
      sf.insert(scan);

      arch << acts << sf;
    }
  }

  const std::string bag = dir + "/bag";
  const std::string cmd = std::string(exe) + " -i " + rawlog + " -o " + bag + " -w";
  ASSERT_EQ(std::system(cmd.c_str()), 0) << cmd;

  rosbag2_cpp::Reader reader;
  reader.open(bag);
  std::map<std::string, size_t> counts;
  std::optional<nav_msgs::msg::Odometry> lastOdom;
  while (reader.has_next())
  {
    const auto m = reader.read_next();
    counts[m->topic_name]++;
    if (m->topic_name == "/odom")
    {
      lastOdom = deserialize<nav_msgs::msg::Odometry>(*m);
    }
  }
  reader.close();
  mrpt::system::deleteFilesInDirectory(dir, true);

  EXPECT_EQ(counts["/laser/scan"], NUM_STEPS);
  EXPECT_EQ(counts["/odom"], NUM_STEPS);
  EXPECT_EQ(counts["/tf"], NUM_STEPS);
  // Four steps of (1 m forward, +90 deg) go around a square, back to the origin:
  ASSERT_TRUE(lastOdom.has_value());
  EXPECT_NEAR(lastOdom->pose.pose.position.x, 0.0, 1e-6);
  EXPECT_NEAR(lastOdom->pose.pose.position.y, 0.0, 1e-6);
}
