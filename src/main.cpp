#include <iostream>

#include <CpoFrontEnd.hpp>

int main() {

  // set up RTKLIB logging
  int trace_level = 3;
  fs::path trace_path{"/home/ben/Desktop/debug.trace"}; // todo: better location for this (may not want in final)
  if (trace_level > 0) {
    traceopen(trace_path.c_str());
    tracelevel(trace_level);
  }

  // serial port parameters
  std::string port0_path = "/dev/ttyUSB0";
  unsigned long baud = 9600;

  CpoFrontEnd node(port0_path, baud);

  unsigned char byte_in;          // todo: add option to log raw RTCM to file?
  size_t total_bytes_read = 0;
  int status;
  while (true) {
    size_t bytes_read = node.serial_port.read(&byte_in, 1);
    total_bytes_read += bytes_read;

    if (bytes_read)
      std::cout << bytes_read << ", byte read: " << byte_in << std::endl;

    status = input_rtcm3(&(node.rtcm), byte_in);

    if (total_bytes_read > 4000 * 1024) break;      // temporary so doesn't complain about endless loops
  }

  return 0;
}
