#pragma once

#include <filesystem>

#include "serial/serial.h"
#include "../deps/RTKLIB/src/rtklib.h"

namespace fs = std::filesystem;

/** \brief Class to read and process raw GNSS measurements.
 * Sends pseudo-measurements as ROS2 msgs to a back end.
 * */
class CpoFrontEnd {
 public:

  /** \brief Constructor */
  CpoFrontEnd(const std::string& port_path, unsigned long baud);

  /** \brief The serial port that listens for GNSS measurements */
  serial::Serial serial_port;

  /** \brief RTKLIB struct that stores observations, ephemerides */
  rtcm_t rtcm;

 private:

};
