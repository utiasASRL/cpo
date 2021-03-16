#include <unistd.h>

#include <iostream>
#include <filesystem>

#include "serial/serial.h"
#include "rtklib.h"

namespace fs = std::filesystem;

int main() {

  std::string port0_path = "/dev/ttyUSB0";
  if (!fs::exists(port0_path)) {
    throw std::runtime_error("Serial connection not found. Check that USB is plugged in.");
  }
  unsigned long baud = 9600;

  // open serial connection and clear anything in buffer
  serial::Serial port0(port0_path, baud, serial::Timeout::simpleTimeout(1000));
  usleep(50000);
  port0.flushInput();

  unsigned char byte_in;

  size_t total_bytes_read = 0;
  while (true) {
    size_t bytes_read = port0.read(&byte_in, 1);
    total_bytes_read += bytes_read;

    std::cout << bytes_read << ", byte read: " << byte_in << std::endl;

    if (total_bytes_read > 100) break;      // temporary
  }

  return 0;
}
