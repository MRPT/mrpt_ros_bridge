/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2025, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

/*---------------------------------------------------------------
  APPLICATION: mrpt_ros bridge
  FILE: gps.cpp
  AUTHOR: Raghavender Sahdev <raghavendersahdev@gmail.com>
  ---------------------------------------------------------------*/

#include <mrpt/ros2bridge/gps.h>
#include <mrpt/ros2bridge/time.h>

#include <cmath>

namespace
{
// Conversion constants
constexpr double KNOTS_TO_MPS = 0.514444;
constexpr double MPS_TO_KNOTS = 1.0 / KNOTS_TO_MPS;
constexpr double MPS_TO_KMH = 3.6;

// Map GPSStatus status values to GGA fix_quality
// GPSStatus:
//   STATUS_NO_FIX=-1, STATUS_FIX=0, STATUS_SBAS_FIX=1, STATUS_GBAS_FIX=2,
//   STATUS_DGPS_FIX=18, STATUS_RTK_FIX=19, STATUS_RTK_FLOAT=20, STATUS_WAAS_FIX=33
// GGA fix_quality:
//   0=invalid, 1=GPS fix, 2=DGPS, 3=PPS, 4=RTK fixed, 5=RTK float,
//   6=estimated, 7=manual, 8=simulation
uint8_t gpsStatusToGgaFixQuality(int16_t status)
{
  switch (status)
  {
    case -1:  // STATUS_NO_FIX
      return 0;
    case 0:  // STATUS_FIX
      return 1;
    case 1:      // STATUS_SBAS_FIX
    case 33:     // STATUS_WAAS_FIX (WAAS is a type of SBAS)
      return 2;  // DGPS-like augmentation
    case 2:      // STATUS_GBAS_FIX
      return 2;  // Ground-based augmentation
    case 18:     // STATUS_DGPS_FIX
      return 2;
    case 19:  // STATUS_RTK_FIX
      return 4;
    case 20:  // STATUS_RTK_FLOAT
      return 5;
    default:
      return 1;  // Default to standard GPS fix
  }
}

// Map GGA fix_quality to GPSStatus status values
int16_t ggaFixQualityToGpsStatus(uint8_t fix_quality)
{
  switch (fix_quality)
  {
    case 0:
      return -1;  // STATUS_NO_FIX
    case 1:
      return 0;  // STATUS_FIX
    case 2:
      return 18;  // STATUS_DGPS_FIX
    case 3:
      return 0;  // PPS -> STATUS_FIX (no direct equivalent)
    case 4:
      return 19;  // STATUS_RTK_FIX
    case 5:
      return 20;  // STATUS_RTK_FLOAT
    case 6:
    case 7:
    case 8:
      return 0;  // Estimated/manual/simulation -> STATUS_FIX
    default:
      return 0;
  }
}

}  // namespace

bool mrpt::ros2bridge::fromROS(
    const sensor_msgs::msg::NavSatFix& msg, mrpt::obs::CObservationGPS& obj)
{
  mrpt::obs::gnss::Message_NMEA_GGA gga;
  gga.fields.altitude_meters = msg.altitude;
  gga.fields.latitude_degrees = msg.latitude;
  gga.fields.longitude_degrees = msg.longitude;

  switch (msg.status.status)
  {
    case -1:
      gga.fields.fix_quality = 0;
      break;
    case 0:
      gga.fields.fix_quality = 1;
      break;
    case 2:
      gga.fields.fix_quality = 2;
      break;
    case 1:
      gga.fields.fix_quality = 3;
      break;
    default:
      gga.fields.fix_quality = 0;  // never going to execute default
  }
  obj.setMsg(gga);

  obj.timestamp = mrpt::ros2bridge::fromROS(msg.header.stamp);

  if (msg.position_covariance_type != sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN)
  {
    auto& cov = obj.covariance_enu.emplace();
    for (int r = 0, i = 0; r < 3; r++)
    {
      for (int c = 0; c < 3; c++)
      {
        cov(r, c) = msg.position_covariance.at(static_cast<size_t>(i++));
      }
    }
  }

  return true;
}

bool mrpt::ros2bridge::toROS(
    const mrpt::obs::CObservationGPS& obj,
    const std_msgs::msg::Header& msg_header,
    sensor_msgs::msg::NavSatFix& msg)
{
  bool valid = false;

  // 1) sensor_msgs::NavSatFix:: header
  msg.header = msg_header;

  // 2) other NavSatFix Parameters, the following 3 could be wrong too

  if (obj.hasMsgClass<mrpt::obs::gnss::Message_NMEA_GGA>())
  {
    const mrpt::obs::gnss::Message_NMEA_GGA& gga =
        obj.getMsgByClass<mrpt::obs::gnss::Message_NMEA_GGA>();
    msg.altitude = gga.fields.altitude_meters;
    msg.latitude = gga.fields.latitude_degrees;
    msg.longitude = gga.fields.longitude_degrees;

    /// following parameter assigned as per
    /// http://mrpt.ual.es/reference/devel/structmrpt_1_1obs_1_1gnss_1_1_message___n_m_e_a___g_g_a_1_1content__t.html#a33415dc947663d43015605c41b0f66cb
    /// http://mrpt.ual.es/reference/devel/gnss__messages__ascii__nmea_8h_source.html
    switch (gga.fields.fix_quality)
    {
      case 0:
        msg.status.status = -1;
        break;
      case 1:
        msg.status.status = 0;
        break;
      case 2:
        msg.status.status = 2;
        break;
      case 3:
        msg.status.status = 1;
        break;
      default:
        // this is based on literature available on GPS as the number of
        // types in ROS and MRPT are not same
        msg.status.status = 0;
    }
    // this might be incorrect as there is not matching field in mrpt
    // message type
    msg.status.service = 1;

    valid = true;
  }

  // cov:
  if (obj.covariance_enu.has_value())
  {
    msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_KNOWN;

    for (int r = 0, i = 0; r < 3; r++)
    {
      for (int c = 0; c < 3; c++)
      {
        msg.position_covariance.at(static_cast<size_t>(i++)) = (*obj.covariance_enu)(r, c);
      }
    }
  }
  else
  {
    msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
  }

  return valid;
}

bool mrpt::ros2bridge::fromROS(const gps_msgs::msg::GPSFix& msg, mrpt::obs::CObservationGPS& obj)
{
  // Clear any existing messages
  obj.clear();

  // Set timestamp from header
  obj.timestamp = mrpt::ros2bridge::fromROS(msg.header.stamp);
  obj.originalReceivedTimestamp = obj.timestamp;

  // --- Populate GGA message ---
  {
    mrpt::obs::gnss::Message_NMEA_GGA gga;
    gga.fields.latitude_degrees = msg.latitude;
    gga.fields.longitude_degrees = msg.longitude;
    gga.fields.altitude_meters = msg.altitude;

    // Convert GPSFix status to GGA fix_quality
    gga.fields.fix_quality = gpsStatusToGgaFixQuality(msg.status.status);

    // Satellites used
    gga.fields.satellitesUsed = msg.status.satellites_used;

    // HDOP
    if (msg.hdop > 0)
    {
      gga.fields.thereis_HDOP = true;
      gga.fields.HDOP = static_cast<float>(msg.hdop);
    }

    // Set UTC time from the GPS time field if available
    if (msg.time > 0)
    {
      // GPSFix.time is seconds since epoch (as float64)
      // Convert to UTC_time structure
      const auto time_t_val = static_cast<time_t>(msg.time);
      struct tm utc_tm;
#ifdef _WIN32
      gmtime_s(&utc_tm, &time_t_val);
#else
      gmtime_r(&time_t_val, &utc_tm);
#endif
      gga.fields.UTCTime.hour = static_cast<uint8_t>(utc_tm.tm_hour);
      gga.fields.UTCTime.minute = static_cast<uint8_t>(utc_tm.tm_min);
      gga.fields.UTCTime.sec =
          static_cast<double>(utc_tm.tm_sec) + std::fmod(msg.time, 1.0);  // Add fractional seconds
    }

    obj.setMsg(gga);
  }

  // --- Populate RMC message (speed and track) ---
  if (std::isfinite(msg.speed) || std::isfinite(msg.track))
  {
    mrpt::obs::gnss::Message_NMEA_RMC rmc;
    rmc.fields.latitude_degrees = msg.latitude;
    rmc.fields.longitude_degrees = msg.longitude;

    // Speed: GPSFix uses m/s, RMC uses knots
    if (std::isfinite(msg.speed))
    {
      rmc.fields.speed_knots = msg.speed * MPS_TO_KNOTS;
    }

    // Track/direction in degrees
    if (std::isfinite(msg.track))
    {
      rmc.fields.direction_degrees = msg.track;
    }

    // Validity based on fix status
    rmc.fields.validity_char = (msg.status.status >= 0) ? 'A' : 'V';

    // Set positioning mode based on status
    if (msg.status.status == -1)
    {
      rmc.fields.positioning_mode = 'N';  // Not valid
    }
    else if (msg.status.status == 18)
    {
      rmc.fields.positioning_mode = 'D';  // Differential
    }
    else if (msg.status.status == 19 || msg.status.status == 20)
    {
      rmc.fields.positioning_mode = 'D';  // RTK is differential-like
    }
    else
    {
      rmc.fields.positioning_mode = 'A';  // Autonomous
    }

    // Set date/time if available
    if (msg.time > 0)
    {
      const auto time_t_val = static_cast<time_t>(msg.time);
      struct tm utc_tm;
#ifdef _WIN32
      gmtime_s(&utc_tm, &time_t_val);
#else
      gmtime_r(&time_t_val, &utc_tm);
#endif
      rmc.fields.UTCTime.hour = static_cast<uint8_t>(utc_tm.tm_hour);
      rmc.fields.UTCTime.minute = static_cast<uint8_t>(utc_tm.tm_min);
      rmc.fields.UTCTime.sec = static_cast<double>(utc_tm.tm_sec) + std::fmod(msg.time, 1.0);
      rmc.fields.date_day = static_cast<uint8_t>(utc_tm.tm_mday);
      rmc.fields.date_month = static_cast<uint8_t>(utc_tm.tm_mon + 1);
      rmc.fields.date_year = static_cast<uint8_t>((utc_tm.tm_year + 1900) % 100);
    }

    obj.setMsg(rmc);
  }

  // --- Populate VTG message (ground speed and track) ---
  if (std::isfinite(msg.speed) || std::isfinite(msg.track))
  {
    mrpt::obs::gnss::Message_NMEA_VTG vtg;
    vtg.fields.true_track = msg.track;
    vtg.fields.ground_speed_knots = msg.speed * MPS_TO_KNOTS;
    vtg.fields.ground_speed_kmh = msg.speed * MPS_TO_KMH;
    obj.setMsg(vtg);
  }

  // --- Populate GSA message (DOP values) ---
  if (msg.pdop > 0 || msg.hdop > 0 || msg.vdop > 0)
  {
    mrpt::obs::gnss::Message_NMEA_GSA gsa;
    gsa.fields.PDOP = msg.pdop;
    gsa.fields.HDOP = msg.hdop;
    gsa.fields.VDOP = msg.vdop;

    // Set fix type based on status
    if (msg.status.status < 0)
    {
      gsa.fields.fix_2D_3D = '1';  // No fix
    }
    else
    {
      // Assume 3D fix if we have altitude, 2D otherwise
      gsa.fields.fix_2D_3D = std::isfinite(msg.altitude) ? '3' : '2';
    }

    gsa.fields.auto_selection_fix = 'A';  // Automatic

    // Copy satellite PRNs if available (up to 12)
    const size_t num_prns = std::min(msg.status.satellite_used_prn.size(), size_t(12));
    for (size_t i = 0; i < num_prns; ++i)
    {
      int prn = msg.status.satellite_used_prn[i];
      gsa.fields.PRNs[i][0] = static_cast<char>('0' + (prn / 10) % 10);
      gsa.fields.PRNs[i][1] = static_cast<char>('0' + prn % 10);
    }

    obj.setMsg(gsa);
  }

  // --- Set covariance if available ---
  if (msg.position_covariance_type != gps_msgs::msg::GPSFix::COVARIANCE_TYPE_UNKNOWN)
  {
    auto& cov = obj.covariance_enu.emplace();
    for (int r = 0, i = 0; r < 3; r++)
    {
      for (int c = 0; c < 3; c++)
      {
        cov(r, c) = msg.position_covariance.at(static_cast<size_t>(i++));
      }
    }
  }

  // Set has_satellite_timestamp if we have a valid time
  obj.has_satellite_timestamp = (msg.time > 0);

  return true;
}

bool mrpt::ros2bridge::toROS(
    const mrpt::obs::CObservationGPS& obj,
    const std_msgs::msg::Header& msg_header,
    gps_msgs::msg::GPSFix& msg)
{
  // Set header
  msg.header = msg_header;

  bool valid = false;

  // --- Extract from GGA message (primary position source) ---
  if (obj.hasMsgClass<mrpt::obs::gnss::Message_NMEA_GGA>())
  {
    const auto& gga = obj.getMsgByClass<mrpt::obs::gnss::Message_NMEA_GGA>();

    msg.latitude = gga.fields.latitude_degrees;
    msg.longitude = gga.fields.longitude_degrees;
    msg.altitude = gga.fields.altitude_meters;

    // Convert fix_quality to GPSStatus
    msg.status.status = ggaFixQualityToGpsStatus(gga.fields.fix_quality);
    msg.status.satellites_used = static_cast<uint16_t>(gga.fields.satellitesUsed);

    // HDOP
    if (gga.fields.thereis_HDOP)
    {
      msg.hdop = static_cast<double>(gga.fields.HDOP);
    }
    else
    {
      msg.hdop = -1.0;  // Unknown
    }

    // Position source is GPS
    msg.status.position_source = gps_msgs::msg::GPSStatus::SOURCE_GPS;

    valid = true;
  }

  // --- Extract from RMC message (speed and track) ---
  if (obj.hasMsgClass<mrpt::obs::gnss::Message_NMEA_RMC>())
  {
    const auto& rmc = obj.getMsgByClass<mrpt::obs::gnss::Message_NMEA_RMC>();

    // Speed: RMC uses knots, GPSFix uses m/s
    msg.speed = rmc.fields.speed_knots * KNOTS_TO_MPS;
    msg.track = rmc.fields.direction_degrees;

    // Motion source
    msg.status.motion_source = gps_msgs::msg::GPSStatus::SOURCE_GPS;
  }
  else if (obj.hasMsgClass<mrpt::obs::gnss::Message_NMEA_VTG>())
  {
    // Fallback to VTG if RMC not available
    const auto& vtg = obj.getMsgByClass<mrpt::obs::gnss::Message_NMEA_VTG>();

    msg.speed = vtg.fields.ground_speed_knots * KNOTS_TO_MPS;
    msg.track = vtg.fields.true_track;

    msg.status.motion_source = gps_msgs::msg::GPSStatus::SOURCE_GPS;
  }
  else
  {
    // No velocity info available
    msg.speed = std::nan("");
    msg.track = std::nan("");
    msg.status.motion_source = gps_msgs::msg::GPSStatus::SOURCE_NONE;
  }

  // --- Extract from GSA message (DOP values) ---
  if (obj.hasMsgClass<mrpt::obs::gnss::Message_NMEA_GSA>())
  {
    const auto& gsa = obj.getMsgByClass<mrpt::obs::gnss::Message_NMEA_GSA>();

    msg.pdop = gsa.fields.PDOP;
    msg.hdop = gsa.fields.HDOP;
    msg.vdop = gsa.fields.VDOP;
  }
  else
  {
    // HDOP might have been set from GGA, set others to unknown
    msg.pdop = -1.0;
    msg.vdop = -1.0;
  }

  // GDOP and TDOP are typically not available in standard NMEA messages
  msg.gdop = -1.0;
  msg.tdop = -1.0;

  // --- Set vertical speed (climb) ---
  // Not available in standard NMEA messages
  msg.climb = std::nan("");

  // --- Set orientation fields ---
  // These are not available in standard NMEA (would need IMU/INS data)
  msg.pitch = std::nan("");
  msg.roll = std::nan("");
  msg.dip = std::nan("");
  msg.status.orientation_source = gps_msgs::msg::GPSStatus::SOURCE_NONE;

  // --- Set time field ---
  // Convert MRPT timestamp to seconds since epoch
  if (obj.timestamp != INVALID_TIMESTAMP)
  {
    msg.time = mrpt::Clock::toDouble(obj.timestamp);
  }
  else
  {
    msg.time = 0.0;
  }

  // --- Set uncertainty fields ---
  // These can be derived from covariance if available
  if (obj.covariance_enu.has_value())
  {
    const auto& cov = *obj.covariance_enu;

    // Horizontal error (from East and North variances)
    double var_e = cov(0, 0);
    double var_n = cov(1, 1);
    double var_u = cov(2, 2);

    // 95% confidence (roughly 2 sigma)
    msg.err_horz = 2.0 * std::sqrt(var_e + var_n);
    msg.err_vert = 2.0 * std::sqrt(var_u);
    msg.err = 2.0 * std::sqrt(var_e + var_n + var_u);

    // Copy full covariance matrix
    msg.position_covariance_type = gps_msgs::msg::GPSFix::COVARIANCE_TYPE_KNOWN;
    for (int r = 0, i = 0; r < 3; r++)
    {
      for (int c = 0; c < 3; c++)
      {
        msg.position_covariance.at(static_cast<size_t>(i++)) = cov(r, c);
      }
    }
  }
  else
  {
    msg.err = -1.0;
    msg.err_horz = -1.0;
    msg.err_vert = -1.0;
    msg.position_covariance_type = gps_msgs::msg::GPSFix::COVARIANCE_TYPE_UNKNOWN;
  }

  // Set other uncertainty fields as unknown
  msg.err_track = -1.0;
  msg.err_speed = -1.0;
  msg.err_climb = -1.0;
  msg.err_time = -1.0;
  msg.err_pitch = -1.0;
  msg.err_roll = -1.0;
  msg.err_dip = -1.0;

  return valid;
}

/// NavSatFix ROS message
/*
uint8 COVARIANCE_TYPE_UNKNOWN=0
uint8 COVARIANCE_TYPE_APPROXIMATED=1
uint8 COVARIANCE_TYPE_DIAGONAL_KNOWN=2
uint8 COVARIANCE_TYPE_KNOWN=3
std_msgs/Header header
sensor_msgs/NavSatStatus status
float64 latitude
float64 longitude
float64 altitude
float64[9] position_covariance
uint8 position_covariance_type
*/

/// NavSatStatus ROS message
/*
int8 STATUS_NO_FIX=-1
int8 STATUS_FIX=0
int8 STATUS_SBAS_FIX=1
int8 STATUS_GBAS_FIX=2
uint16 SERVICE_GPS=1
uint16 SERVICE_GLONASS=2
uint16 SERVICE_COMPASS=4
uint16 SERVICE_GALILEO=8
int8 status
uint16 service
 */