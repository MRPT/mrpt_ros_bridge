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
  FILE: GPS.h
  AUTHOR: Raghavender Sahdev <raghavendersahdev@gmail.com>
  ---------------------------------------------------------------*/

#pragma once

#include <mrpt/obs/CObservationGPS.h>

#include <gps_msgs/msg/gps_fix.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

/// ROS message:    http://docs.ros.org/api/sensor_msgs/html/msg/NavSatFix.html
/// MRPT message:
/// https://github.com/MRPT/mrpt/blob/master/libs/obs/include/mrpt/obs/CObservationGPS.h

/** Conversion functions between ROS 1 <-> MRPT types.
 * \ingroup mrpt_ros2bridge_grp
 */
namespace mrpt::ros2bridge
{
/** \addtogroup mrpt_ros2bridge_grp
 * @{ */

/** Convert sensor_msgs/NavSatFix -> mrpt::obs::CObservationGPS
 * \return true on successful conversion, false on any error.
 */
bool fromROS(const sensor_msgs::msg::NavSatFix& msg, mrpt::obs::CObservationGPS& obj);

/** Convert mrpt::obs::CObservationGPS -> sensor_msgs/NavSatFix
 *  The user must supply the "msg_header" field to be copied into the output
 * message object, since that part does not appear in MRPT classes.
 *
 * \return true on successful conversion, only if the input observation has a GGA
 * message.
 */
bool toROS(
    const mrpt::obs::CObservationGPS& obj,
    const std_msgs::msg::Header& msg_header,
    sensor_msgs::msg::NavSatFix& msg);

/** Convert gps_msgs/GPSFix -> mrpt::obs::CObservationGPS
 *
 * This function populates multiple GNSS messages in the CObservationGPS object:
 * - Message_NMEA_GGA: position, altitude, fix quality, HDOP, satellites
 * - Message_NMEA_RMC: speed (converted from m/s to knots), track/direction
 * - Message_NMEA_VTG: ground speed, track
 * - Message_NMEA_GSA: DOP values (PDOP, HDOP, VDOP)
 *
 * The covariance_enu field is also populated if the covariance type is known.
 *
 * \return true on successful conversion, false on any error.
 */
bool fromROS(const gps_msgs::msg::GPSFix& msg, mrpt::obs::CObservationGPS& obj);

/** Convert mrpt::obs::CObservationGPS -> gps_msgs/GPSFix
 *
 * The user must supply the "msg_header" field to be copied into the output
 * message object, since that part does not appear in MRPT classes.
 *
 * This function extracts data from multiple GNSS messages if available:
 * - Message_NMEA_GGA: position, altitude, fix quality, HDOP, satellites
 * - Message_NMEA_RMC: speed, track/direction
 * - Message_NMEA_VTG: ground speed, track (used if RMC not available)
 * - Message_NMEA_GSA: DOP values
 *
 * Fields not available in MRPT messages will be set to 0 or NaN as appropriate.
 *
 * \return true if at least GGA message is present, false otherwise.
 */
bool toROS(
    const mrpt::obs::CObservationGPS& obj,
    const std_msgs::msg::Header& msg_header,
    gps_msgs::msg::GPSFix& msg);

/** @} */

}  // namespace mrpt::ros2bridge