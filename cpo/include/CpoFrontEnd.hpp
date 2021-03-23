#pragma once

#include <filesystem>

#include <serial/serial.h>
#include <rtklib.h>
#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>

#include <SatelliteObs.hpp>

#define FROM_FILE 1         // reads data from binary file rather than over serial port

namespace fs = std::filesystem;

/** \brief Class to read and process raw GNSS measurements.
 * Sends pseudo-measurements as ROS2 msgs to a back end.
 * */
class CpoFrontEnd : public rclcpp::Node {
 public:

  /** \brief Constructor */
  CpoFrontEnd(const std::string &port_path, unsigned long baud);



//  void publishTdcp();

//  void updateSats();

  /** \brief Set ECEF coordinates of our local ENU frame
   * Should usually only be called once
   * */
  void setEnuOrigin(double *rr);

  /** \brief Update the latest code solution we calculated */
  void update_code_pos(double *rr);

#if !FROM_FILE
  /** \brief The serial port that listens for GNSS measurements */
  serial::Serial serial_port;
#endif

  /** \brief RTKLIB struct that stores observations, ephemerides */
  rtcm_t rtcm;

  /** \brief RTKLIB struct that sets parameters for single-point positioning */
  prcopt_t code_positioning_options;

  /** \brief The set of observations at the last timestamp */
  std::shared_ptr<std::unordered_map<uint8_t, SatelliteObs>> prev_sats;

  /** \brief The set of observations at the current timestamp */
  std::shared_ptr<std::unordered_map<uint8_t, SatelliteObs>> curr_sats;

  /** \brief Keep track of which GPS satellites we have ephemeris info for */
//  bool eph_set_gps[NSATGPS] = { false };
  bool eph_set_gps[64] = {false};       // todo: figure out weird memory things happening to eph_count_gps

  /** \brief Number of  GPS satellites we have ephemeris info for */
  uint eph_count_gps = 0;

  /** \brief Track whether we've calculated a code solution yet */
  bool enu_origin_set = false;

 private:

  /** \brief The current single-point positioning (pseudorange) estimate */
  // todo - what frame do we want this? will use ENU (g) for now
  Eigen::Vector3d curr_code_solution_;

  /** \brief The previous single-point positioning (pseudorange) estimate */
  Eigen::Vector3d prev_code_solution_;

  /** \brief Position of the local East-North-Up frame origin in ECEF coordinates */
  Eigen::Vector3d enu_origin_;

  /** \brief SO(3) rotation matrix between the ENU and ECEF frames */
  Eigen::Matrix3d C_enu_ecef_;

  /** \brief Publishes the TDCP pseudo-measurement ROS2 msg
   * \note NOT IMPLEMENTED YET
   * */
//  rclcpp::Publisher publisher_;

};
