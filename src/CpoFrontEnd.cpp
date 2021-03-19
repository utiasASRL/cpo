#include <unistd.h>

#include <CpoFrontEnd.hpp>

CpoFrontEnd::CpoFrontEnd(const std::string &port_path, unsigned long baud)
    : serial_port(port_path, baud, serial::Timeout::simpleTimeout(1000)) {

  // clear anything in serial buffer
  usleep(50000);
  serial_port.flushInput();

  init_rtcm(&rtcm);

  prev_sats = std::make_shared<std::unordered_map<uint8_t, SatelliteObs>>();
  curr_sats = std::make_shared<std::unordered_map<uint8_t, SatelliteObs>>();
}
