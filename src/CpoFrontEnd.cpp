#include <unistd.h>

#include <CpoFrontEnd.hpp>


CpoFrontEnd::CpoFrontEnd(const std::string& port_path, unsigned long baud) {

  if (!fs::exists(port_path)) {          // todo: should this warn and keep trying to connect instead?
    throw std::runtime_error("Serial connection not found. Check that USB is plugged in.");
  }

  // open serial connection and clear anything in buffer
  serial::Serial port0(port_path, baud, serial::Timeout::simpleTimeout(1000));
  usleep(50000);
  port0.flushInput();

  init_rtcm(&rtcm);
}
