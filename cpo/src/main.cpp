#include <iostream>

#include <CpoFrontEnd.hpp>

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);

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

        // get approximate start position through single-point (pseudo-range) positioning
        sol_t init_solution;
        char error_msg[128];
        bool success = pntpos(node.rtcm.obs.data,
                              node.rtcm.obs.n,
                              &node.rtcm.nav,
                              &node.code_positioning_options,
                              &init_solution,
                              nullptr,
                              nullptr,
                              error_msg);

        if (success) {
          // define the ENU origin if this is the first solution calculated
          if (!node.enu_origin_set) {
            node.setEnuOrigin(&init_solution.rr[0]);
          }
          node.updateCodePos(&init_solution.rr[0]);
        }
      }

      if (node.prev_sats->size() >= 2) {
        // todo - the meat




        // dummy msg for testing right now
        cpo_interfaces::msg::TDCP test_msg;
        test_msg.t_a = 1615000017222000000;
        test_msg.t_b = 1615000018000000000;

        // publish the pseudo-measurement to be used by the back-end
        node.publishTdcp(test_msg);
      }

      // current {sats, code_pos} -> previous {sats, code_pos}
      node.stepForward();
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
