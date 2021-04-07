#include <unistd.h>

#include <iostream>
#include <filesystem>

#include "serial/serial.h"

namespace fs = std::filesystem;

/// @brief Opens a binary file of logged RTCM messages and sends the data over a serial port
int main() {

  // specify location of binary RTCM file
  std::string data_directory = "/home/ben/CLionProjects/gpso/data/rtcm3/";
  std::string data_file = "feb10a.BIN";
  fs::path data_path{data_directory + data_file};
  if (!fs::exists(data_path)) {
    throw std::invalid_argument("File path not found");
  }

  int bytes_per_second = 1000;     // how fast to publish

  // configure serial port
  unsigned long baud = 57600;
  std::string port1_path = "/dev/ttyUSB1";
  if (!fs::exists(port1_path)) {
    throw std::runtime_error("Serial connection not found. Check that USB is plugged in.");
  }
  serial::Serial port1(port1_path, baud, serial::Timeout::simpleTimeout(1000));
  usleep(50000);
  port1.flushOutput();

  // open file
  auto fp = fopen(data_path.c_str(), "r");
  if (!fp) {
    throw std::runtime_error("Couldn't open file.");
  }

  // read in whole file byte by byte
  int data_int;
  uint bytes_sent = 0;
  while ((data_int = fgetc(fp)) != EOF) {
    // send over serial port
    uint8_t data = data_int;
    port1.write(&data, 1);

    // don't want to send all the data at once
    if (bytes_sent++ % bytes_per_second == 0) {     // todo - figure out best way to time
      usleep(1e6);
    }
  }

  return 0;
}
