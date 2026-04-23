/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2026, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/ros2bridge/gps.h>
#include <mrpt/ros2bridge/time.h>

#include <sensor_msgs/msg/nav_sat_fix.hpp>

// Helper: build a minimal NavSatFix with known coordinates and fix status.
static sensor_msgs::msg::NavSatFix makeNavSatFix(double lat, double lon, double alt, int8_t status)
{
  sensor_msgs::msg::NavSatFix msg;
  msg.header.stamp.sec = 1000;
  msg.header.stamp.nanosec = 0;
  msg.latitude = lat;
  msg.longitude = lon;
  msg.altitude = alt;
  msg.status.status = status;
  msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
  msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
  msg.position_covariance[0] = 4.0;
  msg.position_covariance[4] = 4.0;
  msg.position_covariance[8] = 9.0;
  return msg;
}

// -----------------------------------------------------------------------
// Round-trip: NavSatFix → CObservationGPS → NavSatFix
// Verifies the fix that prevented lat/lon/alt from surviving the round-trip
// when the observation comes from fromROS() (i.e. GGA is present).
// -----------------------------------------------------------------------
TEST(GPS, NavSatFix_RoundTrip_Coordinates)
{
  const double lat = 36.825004;  // 3649.50027 N in decimal degrees
  const double lon = -2.439740;  // 00226.38438 W in decimal degrees
  const double alt = 23.0;

  auto in_msg = makeNavSatFix(lat, lon, alt, /*STATUS_FIX=*/0);

  mrpt::obs::CObservationGPS obs;
  ASSERT_TRUE(mrpt::ros2bridge::fromROS(in_msg, obs));

  // Confirm GGA was stored — this is what toROS() depends on:
  ASSERT_TRUE(obs.hasMsgClass<mrpt::obs::gnss::Message_NMEA_GGA>())
      << "fromROS must store a GGA message so toROS can retrieve coordinates";

  std_msgs::msg::Header hdr;
  hdr.stamp = in_msg.header.stamp;
  sensor_msgs::msg::NavSatFix out_msg;
  const bool valid = mrpt::ros2bridge::toROS(obs, hdr, out_msg);

  ASSERT_TRUE(valid) << "toROS must return true when a GGA message is present";

  EXPECT_NEAR(out_msg.latitude, lat, 1e-9) << "Latitude must survive round-trip";
  EXPECT_NEAR(out_msg.longitude, lon, 1e-9) << "Longitude must survive round-trip";
  EXPECT_NEAR(out_msg.altitude, alt, 1e-6) << "Altitude must survive round-trip";
}

// -----------------------------------------------------------------------
// toROS must NOT publish zero coordinates when no GGA is present.
// Before the fix, valid=true was set unconditionally due to a brace bug.
// -----------------------------------------------------------------------
TEST(GPS, NavSatFix_ToROS_ReturnsFalse_WhenNoGGA)
{
  mrpt::obs::CObservationGPS obs;  // empty — no GGA stored

  std_msgs::msg::Header hdr;
  sensor_msgs::msg::NavSatFix out_msg;
  const bool valid = mrpt::ros2bridge::toROS(obs, hdr, out_msg);

  EXPECT_FALSE(valid) << "toROS must return false (not publish zeros) when no GGA is present";
  EXPECT_DOUBLE_EQ(out_msg.latitude, 0.0);
  EXPECT_DOUBLE_EQ(out_msg.longitude, 0.0);
}

// -----------------------------------------------------------------------
// Fix status round-trips correctly for all NavSatFix status codes.
// -----------------------------------------------------------------------
struct StatusRoundTripParam
{
  int8_t status;
};

class GPS_StatusRoundTrip : public ::testing::TestWithParam<StatusRoundTripParam>
{
};

TEST_P(GPS_StatusRoundTrip, StatusSurvivesRoundTrip)
{
  auto in_msg = makeNavSatFix(1.0, 2.0, 3.0, GetParam().status);

  mrpt::obs::CObservationGPS obs;
  ASSERT_TRUE(mrpt::ros2bridge::fromROS(in_msg, obs));

  std_msgs::msg::Header hdr;
  sensor_msgs::msg::NavSatFix out_msg;
  ASSERT_TRUE(mrpt::ros2bridge::toROS(obs, hdr, out_msg));

  EXPECT_EQ(out_msg.status.status, GetParam().status);
}

INSTANTIATE_TEST_SUITE_P(
    AllFixTypes,
    GPS_StatusRoundTrip,
    ::testing::Values(
        StatusRoundTripParam{-1},  // STATUS_NO_FIX
        StatusRoundTripParam{0},   // STATUS_FIX
        StatusRoundTripParam{1},   // STATUS_SBAS_FIX
        StatusRoundTripParam{2}    // STATUS_GBAS_FIX
        ));

// -----------------------------------------------------------------------
// Covariance round-trip
// -----------------------------------------------------------------------
TEST(GPS, NavSatFix_RoundTrip_Covariance)
{
  auto in_msg = makeNavSatFix(1.0, 2.0, 3.0, 0);

  mrpt::obs::CObservationGPS obs;
  ASSERT_TRUE(mrpt::ros2bridge::fromROS(in_msg, obs));
  ASSERT_TRUE(obs.covariance_enu.has_value());

  std_msgs::msg::Header hdr;
  sensor_msgs::msg::NavSatFix out_msg;
  ASSERT_TRUE(mrpt::ros2bridge::toROS(obs, hdr, out_msg));

  EXPECT_EQ(out_msg.position_covariance_type, sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_KNOWN);
  EXPECT_NEAR(out_msg.position_covariance[0], 4.0, 1e-9);
  EXPECT_NEAR(out_msg.position_covariance[4], 4.0, 1e-9);
  EXPECT_NEAR(out_msg.position_covariance[8], 9.0, 1e-9);
}