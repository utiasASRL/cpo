#include <unistd.h>

#include <iostream>     // for debugging logging

#include <CpoFrontEnd.hpp>

CpoFrontEnd::CpoFrontEnd(const std::string &port_path, unsigned long baud)
    : Node("cpo_front_end")
#if !FROM_FILE
, serial_port(port_path, baud, serial::Timeout::simpleTimeout(1000))
#endif
{
  publisher_ = this->create_publisher<cpo_interfaces::msg::TDCP>("tdcp", 10);

  // clear anything in serial buffer
  usleep(50000);
#if !FROM_FILE
  serial_port.flushInput();
#endif

  init_rtcm(&rtcm);

  long approximate_time = std::time(nullptr);     // todo: switch to this when online
  approximate_time = 1613000000;      // setting manually for development

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

  Eigen::Vector3d rr_vec(rr[0], rr[1], rr[2]);
  enu_origin_ = rr_vec;

  // convert to lat/long as sanity check
  double geo_init[3];
  ecef2pos(rr, geo_init);

  // get rotation matrix between ECEF and ENU frames
  double *C;
  C = mat(3, 3);
  xyz2enu(geo_init, C);        // RTKLIB uses column-major matrices
  C_enu_ecef_ << C[0], C[3], C[6], C[1], C[4], C[7], C[2], C[5], C[8];
  enu_origin_set = true;

  std::cout << "ENU origin set to " << std::setprecision(10) << geo_init[0] * R2D << " deg lat, " << geo_init[1] * R2D << " deg lat, "
      << std::setprecision(6) << geo_init[2] << " m alt." << std::endl;
}

void CpoFrontEnd::updateCodePos(double *rr) {
  Eigen::Vector3d r_vc_inc(rr[0], rr[1], rr[2]);
  curr_code_solution_ = C_enu_ecef_ * (r_vc_inc - enu_origin_);
//  std::cout << "curr_code_solution_ " << curr_code_solution_.transpose() << std::endl;    // DEBUG
}

void CpoFrontEnd::publishTdcp(const cpo_interfaces::msg::TDCP &message) {
  publisher_->publish(message);

#if FROM_FILE
  usleep(1e6);      // temporary: sleep while developing so have a chance to watch
#endif
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
