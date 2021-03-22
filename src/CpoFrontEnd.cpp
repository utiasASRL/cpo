#include <unistd.h>

#include <iostream>     // for debugging logging

#include <CpoFrontEnd.hpp>

CpoFrontEnd::CpoFrontEnd(const std::string &port_path, unsigned long baud)
    : serial_port(port_path, baud, serial::Timeout::simpleTimeout(1000)) {

  // clear anything in serial buffer
  usleep(50000);
  serial_port.flushInput();

  init_rtcm(&rtcm);

  long approximate_time = std::time(nullptr);
  approximate_time = 1613000000;      // setting manually for development

  std::cout << "Approx time " << approximate_time << std::endl;   // debug. Todo: clean up
  rtcm.time = {.time = approximate_time};

  code_positioning_options.navsys = SYS_GPS;    // using GPS only for this for now
  code_positioning_options.maxgdop = 15.0;      // a big value so we get a solution even if not super accurate

  prev_sats = std::make_shared<std::unordered_map<uint8_t, SatelliteObs>>();
  curr_sats = std::make_shared<std::unordered_map<uint8_t, SatelliteObs>>();
}
void CpoFrontEnd::setOrigin(double *rr) {

  double geo_init[3];

  ecef2pos(rr, geo_init);
  Eigen::Vector3d rr_vec(rr[0], rr[1], rr[2]);

  // get rotation matrix between ECEF and ENU frames
  double *C;
  C = mat(3, 3);
  xyz2enu(geo_init, C);        // RTKLIB uses column-major matrices
  C_enu_ecef_ << C[0], C[3], C[6], C[1], C[4], C[7], C[2], C[5], C[8];

  enu_origin_ = rr_vec;

  enu_origin_set = true;

  std::cout << "Debug: enu_origin_ set to " << enu_origin_.transpose() << std::endl;
}
