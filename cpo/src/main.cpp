#include <iostream>

#include <CpoFrontEnd.hpp>

int main(int argc, char **argv) {

  // initialize ROS2
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
  int rtcm_status;
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

    rtcm_status = input_rtcm3(&(node.rtcm), byte_in);

    // phase observation received
    if (rtcm_status == 1) {
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

      // only attempt TDCP if we have enough ephemeris info
      if (node.eph_count_gps >= 4) {

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

        // perform TDCP
        if (node.prev_sats->size() >= 2) {
          // iterate through map to find common satellites
          std::vector<int> matches;
          for (auto &[id, curr_sat] : *node.curr_sats) {
            // check for valid ephemeris and continuous phase lock
            if (node.eph_set_gps[id] && curr_sat.isPhaseLocked()) {
              // check that we saw the same satellite last msg
              if (node.prev_sats->count(id) == 1) {
                matches.push_back(id);
              }
            }
          }

          // check that we have at least 2 satellite matches so we can create a double-difference
          if (matches.size() < 2) continue;

          // if we have gotten this far, we plan on publishing a message
          cpo_interfaces::msg::TDCP meas_msg;

          // calculate necessary values for each pair to fill in SatPair msg
          // we'll use the 1st match as the 1st satellite for all pairs and the j^th match for the 2nd
          int sat_1_id = matches[0];
          const auto &sat_1a = node.prev_sats->at(sat_1_id);
          const auto &sat_1b = node.curr_sats->at(sat_1_id);

          gtime_t prev_time = sat_1a.getTimestamp();
          gtime_t curr_time = sat_1b.getTimestamp();
          meas_msg.t_a = 1e9 * (prev_time.time + prev_time.sec);  // todo: may lose precision here but shouldn't matter
          meas_msg.t_b = 1e9 * (curr_time.time + prev_time.sec);

          // calculate vectors to the 1st satellite
          Eigen::Vector3d r_1a_a;
          Eigen::Vector3d r_1a_b;
          node.getSatelliteVector(sat_1_id, prev_time, prev_time, r_1a_a);    //todo: should check return value
          node.getSatelliteVector(sat_1_id, curr_time, prev_time, r_1a_b);

          for (unsigned int j = 1; j < matches.size(); ++j) {
            int sat_2_id = matches[j];
            const auto &sat_2a = node.prev_sats->at(sat_2_id);
            const auto &sat_2b = node.curr_sats->at(sat_2_id);

            double phi_dd = (sat_2b.getAdjPhaseRange() - sat_2a.getAdjPhaseRange())
                - (sat_1b.getAdjPhaseRange() - sat_1a.getAdjPhaseRange());

            // calculate vectors to 2nd satellite
            Eigen::Vector3d r_2a_a = Eigen::Vector3d::Zero();   // placeholder
            Eigen::Vector3d r_2a_b = Eigen::Vector3d::Zero();   // placeholder
            node.getSatelliteVector(sat_2_id, prev_time, prev_time, r_2a_a);
            node.getSatelliteVector(sat_2_id, curr_time, prev_time, r_2a_b);

            cpo_interfaces::msg::SatPair pair_msg;
            pair_msg.phi_measured = phi_dd;
            pair_msg.r_1a_a.set__x(r_1a_a.x());   // more efficient way?
            pair_msg.r_1a_a.set__y(r_1a_a.y());
            pair_msg.r_1a_a.set__z(r_1a_a.z());
            pair_msg.r_1a_b.set__x(r_1a_b.x());
            pair_msg.r_1a_b.set__y(r_1a_b.y());
            pair_msg.r_1a_b.set__z(r_1a_b.z());
            pair_msg.r_2a_a.set__x(r_2a_a.x());
            pair_msg.r_2a_a.set__y(r_2a_a.y());
            pair_msg.r_2a_a.set__z(r_2a_a.z());
            pair_msg.r_2a_b.set__x(r_2a_b.x());
            pair_msg.r_2a_b.set__y(r_2a_b.y());
            pair_msg.r_2a_b.set__z(r_2a_b.z());

            meas_msg.pairs.push_back(pair_msg);
          }
          // publish the pseudo-measurement to be used by the back-end
          node.publishTdcp(meas_msg);
        }
      }
      // current {sats, code_pos} -> previous {sats, code_pos}
      node.stepForward();
    }

    // ephemeris message received. Keep track of which satellites have ephemeris
    if (rtcm_status == 2) {
      const auto &current_eph_sat = node.rtcm.ephsat;
      if (!node.eph_set_gps[current_eph_sat - MINPRNGPS]) {
        node.eph_count_gps++;
      }
      node.eph_set_gps[current_eph_sat] = true;
    }

    if (total_bytes_read > 4000 * 1024) break;      // temporary so doesn't complain about endless loops
  }

  rclcpp::shutdown();
  return 0;
}
