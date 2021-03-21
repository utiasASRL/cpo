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

  if (!fs::exists(port0_path)) {          // todo: should this warn and keep trying to connect instead?
    throw std::runtime_error("Serial connection not found. Check that USB is plugged in.");
  }

  CpoFrontEnd node(port0_path, baud);

  // read serial port
  unsigned char byte_in;          // todo: add option to log raw RTCM to file?
  size_t total_bytes_read = 0;
  int status;
  while (true) {
    size_t bytes_read = node.serial_port.read(&byte_in, 1);

    if (!bytes_read) continue;

    total_bytes_read += bytes_read;

    status = input_rtcm3(&(node.rtcm), byte_in);

    // phase observation received
    if (status == 1) {
      for (unsigned i = 0; i < node.rtcm.obs.n; ++i) {
        obsd_t &obs = node.rtcm.obs.data[i];

        // check the observation is from a positioning satellite (as opposed to SBAS)
        if (obs.sat < MINPRNGPS || obs.sat > MAXPRNGPS) continue;        // todo: assuming GPS right now

        node.curr_sats->insert(std::make_pair(obs.sat, SatelliteObs(obs)));

        if (obs.LLI[0]) {
          trace(2, "Lock loss for satellite %i.\n", obs.sat);
        }
      }

      std::cout << "Found " << node.curr_sats->size() << " satellites!" << std::endl; // debugging - count includes sbas
    }

    // ephemeris message received. Keep track of which satellites have ephemeris
    if (status == 2) {
      const auto & current_eph_sat = node.rtcm.ephsat;
      if (!node.eph_set_gps[current_eph_sat]) {
        node.eph_count_gps++;
      }
      node.eph_set_gps[current_eph_sat] = true;
    }

    if (total_bytes_read > 4000 * 1024) break;      // temporary so doesn't complain about endless loops
  }

  return 0;
}
