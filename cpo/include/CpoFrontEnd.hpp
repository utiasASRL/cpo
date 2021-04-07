#pragma once

#include <filesystem>

#include <serial/serial.h>
#include <rtklib.h>
#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>

#include <SatelliteObs.hpp>
#include <cpo_interfaces/msg/tdcp.hpp>

namespace fs = std::filesystem;

/** \brief Class to read and process raw GNSS measurements.
 * Sends pseudo-measurements as ROS2 msgs to a back end.
 * */
class CpoFrontEnd : public rclcpp::Node {
 public:

  /** \brief Constructor */
  CpoFrontEnd();

  /** \brief Calculate vector from previous code solution to satellite at given time
   * \param sat_no      The satellite PRN
   * \param time        The time the vector should be taken at
   * \param eph_time    Time to look for the ephemeris msg
   * \param r_sa_g      [out] The vector we want
   *
   * \return status     [0 error; 1 success]
   * */
  int getSatelliteVector(int sat_no, gtime_t time, gtime_t eph_time, Eigen::Vector3d &r_sa_g);

  /** \brief Publishes message with phases, vectors in it */
  void publishTdcp(const cpo_interfaces::msg::TDCP &message);

  /** \brief Moves current satellites, positions to previous */
  void stepForward();

  /** \brief Set ECEF coordinates of our local ENU frame
   * Should usually only be called once
   * */
  void setEnuOrigin(double *rr);

  /** \brief Get ECEF coordinates of our local ENU frame */
  Eigen::Vector3d getEnuOrigin() {
    return enu_origin_;
  }

  /** \brief Get geodetic (lat/long/alt) coordinates of our local ENU frame */
  Eigen::Vector3d getGeodeticEnuOrigin() {
        return geodetic_enu_origin_;
  }

  /** \brief Update the latest code solution we calculated */
  void updateCodePos(double *rr);

  /** \brief Get current code position in ENU */
  Eigen::Vector3d getCurrentCodePos() {
    return curr_code_solution_;
  }

  /** \brief Get previous code position in ENU */
  Eigen::Vector3d getPreviousCodePos() {
    return prev_code_solution_;
  }

  /** \brief The serial port that listens for GNSS measurements */
  std::shared_ptr<serial::Serial> serial_port;

  /** \brief RTKLIB struct that stores observations, ephemerides */
  rtcm_t rtcm;

  /** \brief RTKLIB struct that sets parameters for single-point positioning */
  prcopt_t code_positioning_options;

  /** \brief The set of observations at the last timestamp */
  std::shared_ptr<std::unordered_map<uint8_t, SatelliteObs>> prev_sats;

  /** \brief The set of observations at the current timestamp */
  std::shared_ptr<std::unordered_map<uint8_t, SatelliteObs>> curr_sats;

  /** \brief Keep track of which GPS satellites we have ephemeris info for */
  bool eph_set_gps[64] = {false};

  /** \brief Number of  GPS satellites we have ephemeris info for */
  uint eph_count_gps = 0;

  /** \brief Track whether we've calculated a code solution yet */
  bool enu_origin_set = false;

  /** \brief True if running live over serial, false if we are reading a logged dataset */
  bool from_serial;

  /** Location to find binary RTCM file when reading logged data */
  std::string rtcm_path;

 private:

  /** \brief The current single-point positioning (pseudorange) estimate in the ENU frame */
  Eigen::Vector3d curr_code_solution_;

  /** \brief The previous single-point positioning (pseudorange) estimate */
  Eigen::Vector3d prev_code_solution_;

  /** \brief Position of the local East-North-Up frame origin in ECEF coordinates */
  Eigen::Vector3d enu_origin_;

  /** \brief Position of the local East-North-Up frame origin in geodetic coordinates */
  Eigen::Vector3d geodetic_enu_origin_;

  /** \brief SO(3) rotation matrix between the ENU and ECEF frames */
  Eigen::Matrix3d C_enu_ecef_;

  /** \brief Publishes the TDCP pseudo-measurement ROS2 msg */
  rclcpp::Publisher<cpo_interfaces::msg::TDCP>::SharedPtr publisher_;

  std::string port_path_;

  unsigned long baud_;

};
