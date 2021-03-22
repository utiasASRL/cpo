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

#if !FROM_FILE
  if (!fs::exists(port0_path)) {          // todo: should this warn and keep trying to connect instead?
    throw std::runtime_error("Serial connection not found. Check that USB is plugged in.");
  }
#endif

  CpoFrontEnd node(port0_path, baud);

  // read serial port
  unsigned char byte_in;          // todo: add option to log raw RTCM to file?
  size_t total_bytes_read = 0;
  int status;
#if !FROM_FILE
  while (true) {
    size_t bytes_read = node.serial_port.read(&byte_in, 1);

    if (!bytes_read) continue;
#else
  auto fp = fopen("/home/ben/CLionProjects/gpso/data/rtcm3/feb10a.BIN", "r");

  int data_int;
  while ((data_int = fgetc(fp)) != EOF) {
    byte_in = data_int;
    size_t bytes_read = 0;
#endif

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

      std::cout << node.eph_count_gps << "   Found " << node.curr_sats->size() << " satellites!   "; // debugging
      for (const auto &sat : *node.curr_sats) std::cout << (int) sat.first << ", ";
      std::cout << std::endl;

      if (node.eph_count_gps >= 5) {      // todo: may not need this if we're checking pntpos() return value
        std::cout << "TODO: pntpos() " << node.eph_count_gps << std::endl;

//        // get approximate start position through single-point (pseudo-range) positioning
//        // rtcm_t wavelengths need to be filled in to get solution so we do manually
//        for (int i = 0; i < 32; ++i) {
//          rtcm.nav.lam[i][0] = CLIGHT / FREQ1;    // not sure if this stuff actually needed
//          rtcm.nav.lam[i][1] = CLIGHT / FREQ2;
//        }

//        // pntpos() requires obsds to be sequential in memory
//        std::vector<obsd_t> curr_obs;
//        for (const auto & sat : *node.curr_sats) {
//          curr_obs.push_back(sat.second.observation);
//        }

        sol_t init_solution;
        char error_msg[128];
        bool success = pntpos(node.rtcm.obs.data,     // todo - debug
                              node.rtcm.obs.n,
                              &node.rtcm.nav,
                              &node.code_positioning_options,
                              &init_solution,
                              nullptr,
                              nullptr,
                              error_msg);

        for (const auto &ch : error_msg) std::cout << (char) ch;
        std::cout << std::endl;

        if (success && !node.enu_origin_set) {
          node.setOrigin(&init_solution.rr[0]);
        }

      }

    }

    // ephemeris message received. Keep track of which satellites have ephemeris
    if (status == 2) {
      const auto &current_eph_sat = node.rtcm.ephsat;
      if (!node.eph_set_gps[current_eph_sat - MINPRNGPS]) {
        std::cout << "Eph set for  " << current_eph_sat << std::endl;   // debug

        node.eph_count_gps++;
      }
      node.eph_set_gps[current_eph_sat] = true;
    }

    if (total_bytes_read > 4000 * 1024) break;      // temporary so doesn't complain about endless loops
  }

  return 0;
}
