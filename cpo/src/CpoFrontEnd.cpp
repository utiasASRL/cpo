#include <unistd.h>

#include <iostream>     // for debugging logging

#include <CpoFrontEnd.hpp>

CpoFrontEnd::CpoFrontEnd()
    : Node("cpo_front_end") {

  this->declare_parameter("port_path", "/dev/ttyUSB0");
  this->declare_parameter("baud", 57600);
  this->declare_parameter("from_serial", true);
  this->declare_parameter("log_serial", true);
  this->declare_parameter("data_path", "/home/ben/CLionProjects/gpso/data/rtcm3/feb15c.BIN");
  this->declare_parameter("approximate_time", -1);

  port_path_ = this->get_parameter("port_path").as_string();
  baud_ = this->get_parameter("baud").as_int();
  from_serial = this->get_parameter("from_serial").as_bool();
  log_serial = this->get_parameter("log_serial").as_bool();
  rtcm_path = this->get_parameter("data_path").as_string();

  log_serial_path = "/home/ben/CLionProjects/ros2-ws/src/cpo/data/log.bin";   // todo: different file names

  if (!from_serial && !fs::exists(rtcm_path)) {
    throw std::runtime_error("RTCM data file not found.");
  }

  if (from_serial && !fs::exists(port_path_)) {
    throw std::runtime_error("Serial connection not found. Check that USB is plugged in.");
  }

  if (from_serial) {
    serial_port = std::make_shared<serial::Serial>(port_path_, baud_, serial::Timeout::simpleTimeout(1000));
  } else {
    serial_port = nullptr;    // don't need serial port if reading from file
  }

  publisher_ = this->create_publisher<cpo_interfaces::msg::TDCP>("tdcp", 10);

  // clear anything in serial buffer
  usleep(50000);
  if (from_serial) {
    serial_port->flushInput();
  }

  init_rtcm(&rtcm);

  long temp_time = this->get_parameter("approximate_time").as_int();
  long approximate_time;
  if (temp_time < 0) {
    // setting to negative value indicates we want to use the live time
    approximate_time = std::time(nullptr);
  } else {
    approximate_time = temp_time;
  }

  rtcm.time = {.time = approximate_time};

  code_positioning_options.navsys = SYS_GPS;    // using GPS only for this for now
  code_positioning_options.maxgdop = 15.0;      // a big value so we get a solution even if not super accurate
  code_positioning_options.sateph = EPHOPT_BRDC;

  prev_sats = std::make_shared<std::unordered_map<uint8_t, SatelliteObs>>();
  curr_sats = std::make_shared<std::unordered_map<uint8_t, SatelliteObs>>();
}

int CpoFrontEnd::getSatelliteVector(int sat_no, gtime_t time, gtime_t eph_time, Eigen::Vector3d &r_sa_g) {
  double *rs;                 // satellite positions and velocities at previous time
  double *dts;                // satellite clocks
  double *var;                // variances on positions and clock errors
  int svh[MAXOBS];            // satellite health flags
  rs = mat(6, 1);
  dts = mat(2, 1);
  var = mat(1, 1);
  int pos_status = satpos(time, eph_time, sat_no, EPHOPT_BRDC, &rtcm.nav, rs, dts, var, svh);
  if (!pos_status)
    std::cout << "WARNING: Positioning error for satellite " << sat_no << std::endl;

  Eigen::Vector3d r_1c_c{rs[0], rs[1], rs[2]};
  Eigen::Vector3d r_1g_g = C_enu_ecef_ * (r_1c_c - enu_origin_);
  r_sa_g = r_1g_g - prev_code_solution_;

  return pos_status;
}

void CpoFrontEnd::setEnuOrigin(double *rr) {

  enu_origin_ = Eigen::Vector3d{rr[0], rr[1], rr[2]};

  // convert to lat/long as sanity check
  double geo_init[3];
  ecef2pos(rr, geo_init);

  // get rotation matrix between ECEF and ENU frames
  double *C;
  C = mat(3, 3);
  xyz2enu(geo_init, C);        // RTKLIB uses column-major matrices
  C_enu_ecef_ << C[0], C[3], C[6], C[1], C[4], C[7], C[2], C[5], C[8];
  enu_origin_set = true;

  geodetic_enu_origin_ = Eigen::Vector3d{geo_init[0] * R2D, geo_init[1] * R2D, geo_init[2]};

  std::cout << "ENU origin set to " << std::setprecision(10) << geodetic_enu_origin_[0] << " deg lat, "
            << geodetic_enu_origin_[1] << " deg lat, " << std::setprecision(6) << geodetic_enu_origin_[2] << " m alt."
            << std::endl;
}

void CpoFrontEnd::updateCodePos(double *rr) {
  Eigen::Vector3d r_vc_inc(rr[0], rr[1], rr[2]);
  curr_code_solution_ = C_enu_ecef_ * (r_vc_inc - enu_origin_);
}

void CpoFrontEnd::publishTdcp(const cpo_interfaces::msg::TDCP &message) {
  publisher_->publish(message);

  if (!from_serial) {
    usleep(0.2e6);      // sleep to replay at ~5x when reading data from file
  }
}

void CpoFrontEnd::stepForward() {
  // move current sats to previous sats and create empty map for current sats to be filled next loop
  prev_sats = curr_sats;
  curr_sats.reset();
  curr_sats = std::make_shared<std::unordered_map<uint8_t, SatelliteObs>>();

  // move current code solution to previous and set current solution to zero
  prev_code_solution_ = curr_code_solution_;
  curr_code_solution_ = Eigen::Vector3d::Zero();
}
