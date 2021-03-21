#pragma once

#include <filesystem>

#include <serial/serial.h>
#include <rtklib.h>
#include <Eigen/Core>

#include <SatelliteObs.hpp>

namespace fs = std::filesystem;

/** \brief Class to read and process raw GNSS measurements.
 * Sends pseudo-measurements as ROS2 msgs to a back end.
 * */
class CpoFrontEnd {
 public:

  /** \brief Constructor */
  CpoFrontEnd(const std::string &port_path, unsigned long baud);



//  void publishTdcp();

//  void updateSats();

  /** \brief The serial port that listens for GNSS measurements */
  serial::Serial serial_port;

  /** \brief RTKLIB struct that stores observations, ephemerides */
  rtcm_t rtcm;

  /** \brief The set of observations at the last timestamp */
  std::shared_ptr<std::unordered_map<uint8_t, SatelliteObs>> prev_sats;

  /** \brief The set of observations at the current timestamp */
  std::shared_ptr<std::unordered_map<uint8_t, SatelliteObs>> curr_sats;

  /** \brief Keep track of which GPS satellites we have ephemeris info for */
  bool eph_set_gps[NSATGPS] = { false };

  /** \brief Number of  GPS satellites we have ephemeris info for */
  uint eph_count_gps = 0;

 private:

  /** \brief The last single-point positioning (pseudorange) estimate */
  Eigen::Vector3d latest_code_solution_;

  /** \brief Position of the local East-North-Up frame origin in ECEF coordinates */
  Eigen::Vector3d enu_origin_;

  /** \brief SO(3) rotation matrix between the ENU and ECEF frames */
  Eigen::Matrix3d C_enu_ecef_;

  /** \brief Publishes the TDCP pseudo-measurement ROS2 msg
   * \note NOT IMPLEMENTED YET
   * */
//  rclcpp::Publisher publisher_;

};
