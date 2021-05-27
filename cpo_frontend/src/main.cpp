#include <iostream>
#include <fstream>

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

  auto node = std::make_shared<CpoFrontEnd>();

  unsigned char byte_in;
  int rtcm_status;

  _IO_FILE *fp;
  std::ofstream fs;

  if (!node->from_serial) {
    fp = fopen(node->rtcm_path.c_str(), "r");
  } else if (node->log_serial) {
    fs = std::ofstream (node->log_serial_path, std::ios::out | std::ios::binary);
  }

  while (rclcpp::ok()) {
    rclcpp::spin_some(node);

    if (node->from_serial) {
      // attempt to read byte from serial, if no data, continue
      size_t bytes_read = node->serial_port->read(&byte_in, 1);
      if (!bytes_read) continue;

      if (node->log_serial) {
        fs.write(reinterpret_cast<const char *>(&byte_in), 1);
      }
    } else {
      // attempt to read byte from file, if at end-of-file, break
      int data_int = fgetc(fp);
      if (data_int == EOF)
        break;
      else
        byte_in = data_int;
    }

    rtcm_status = input_rtcm3(&(node->rtcm), byte_in);

    // phase observation received
    if (rtcm_status == 1) {
      if (node->use_sim_time && node->rtcm.obs.data->time.time - LEAP_SECONDS < (node->get_clock()->now().seconds() - 5)){
        std::cout << "Found old message. Continuing. " <<  (node->get_clock()->now().seconds() - node->rtcm.obs.data->time.time - LEAP_SECONDS) << std::endl;
        continue;     // todo: sorta hacky and not well tested
      }

      for (unsigned i = 0; i < node->rtcm.obs.n; ++i) {
        obsd_t &obs = node->rtcm.obs.data[i];

        // check the observation is from a positioning satellite (as opposed to SBAS)
        if (obs.sat < MINPRNGPS || obs.sat > MAXPRNGPS) continue;        // todo: assuming GPS right now

        double now_time = node->get_clock()->now().seconds();     // UTC time
        node->curr_sats->insert(std::make_pair(obs.sat, SatelliteObs(obs, now_time)));

        if (obs.LLI[0]) {
          trace(2, "Lock loss for satellite %i.\n", obs.sat);
        }
      }

      std::cout << "Observed " << node->curr_sats->size() << " satellites:  ";
      for (const auto &sat : *node->curr_sats) std::cout << (int) sat.first << ", ";
      std::cout << std::endl;

      // get approximate start position through single-point (pseudo-range) positioning
      sol_t init_solution;
      char error_msg[128];
      bool success = pntpos(node->rtcm.obs.data,
                            node->rtcm.obs.n,
                            &node->rtcm.nav,
                            &node->code_positioning_options,
                            &init_solution,
                            nullptr,
                            nullptr,
                            error_msg);

      if (success) {
        // define the ENU origin if this is the first solution calculated
        if (!node->enu_origin_set) {
          node->setEnuOrigin(&init_solution.rr[0]);
        }
        node->updateCodePos(&init_solution.rr[0]);
      }

      // perform TDCP
      if (node->prev_sats->size() >= 2 && node->enu_origin_set) {
        // iterate through map to find common satellites
        std::vector<int> matches;
        for (auto &[id, curr_sat] : *node->curr_sats) {
          // check for valid ephemeris and continuous phase lock
          if (node->eph_set_gps[id] && curr_sat.isPhaseLocked()) {
            // check that we saw the same satellite last msg
            if (node->prev_sats->count(id) == 1) {
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
        auto &sat_1a = node->prev_sats->at(sat_1_id);
        auto &sat_1b = node->curr_sats->at(sat_1_id);

        gtime_t prev_time_gps = sat_1a.getMeasTimestamp();
        gtime_t curr_time_gps = sat_1b.getMeasTimestamp();

        if (node->enable_tropospheric_correction) {
          // calculate atmospheric corrections here to ensure we have an ephemeris
          double *sat_pos_vel_a;
          double *sat_pos_vel_b;
          node->getSatellitePosition(sat_1_id, prev_time_gps, prev_time_gps, sat_pos_vel_a);
          node->getSatellitePosition(sat_1_id, curr_time_gps, curr_time_gps, sat_pos_vel_b);

          // only rough receiver position needed so we reuse init_solution
          sat_1a.estimateTroposphericDelay(sat_pos_vel_a, init_solution.rr);
          sat_1b.estimateTroposphericDelay(sat_pos_vel_b, init_solution.rr);
        }

        // convert to UTC time for use by other packages
        meas_msg.t_a = 1e9 * (prev_time_gps.time + prev_time_gps.sec - LEAP_SECONDS);  // todo: may lose precision here but shouldn't matter
        meas_msg.t_b = 1e9 * (curr_time_gps.time + curr_time_gps.sec - LEAP_SECONDS);

        Eigen::Vector3d current_code = node->getCurrentCodePos();
        meas_msg.enu_pos.set__x(current_code.x());
        meas_msg.enu_pos.set__y(current_code.y());
        meas_msg.enu_pos.set__z(current_code.z());

        Eigen::Vector3d enu_origin = node->getGeodeticEnuOrigin();
        meas_msg.enu_origin.set__x(enu_origin.x());
        meas_msg.enu_origin.set__y(enu_origin.y());
        meas_msg.enu_origin.set__z(enu_origin.z());

        // calculate vectors to the 1st satellite
        Eigen::Vector3d r_1a_a;
        Eigen::Vector3d r_1a_b;
        node->getSatelliteVector(sat_1_id, prev_time_gps, prev_time_gps, r_1a_a);    //todo: should check return value
        node->getSatelliteVector(sat_1_id, curr_time_gps, prev_time_gps, r_1a_b);

        for (unsigned int j = 1; j < matches.size(); ++j) {
          int sat_2_id = matches[j];
          auto &sat_2a = node->prev_sats->at(sat_2_id);
          auto &sat_2b = node->curr_sats->at(sat_2_id);

          if (node->enable_tropospheric_correction) {
            double *sat_pos_vel_a;
            double *sat_pos_vel_b;
            node->getSatellitePosition(sat_2_id, prev_time_gps, prev_time_gps, sat_pos_vel_a);
            node->getSatellitePosition(sat_2_id, curr_time_gps, curr_time_gps, sat_pos_vel_b);
            sat_2a.estimateTroposphericDelay(sat_pos_vel_a, init_solution.rr);
            sat_2b.estimateTroposphericDelay(sat_pos_vel_b, init_solution.rr);
          }

          double phi_dd = (sat_2b.getAdjPhaseRange() - sat_2a.getAdjPhaseRange())
              - (sat_1b.getAdjPhaseRange() - sat_1a.getAdjPhaseRange());

          // calculate vectors to 2nd satellite
          Eigen::Vector3d r_2a_a = Eigen::Vector3d::Zero();   // placeholder
          Eigen::Vector3d r_2a_b = Eigen::Vector3d::Zero();   // placeholder
          node->getSatelliteVector(sat_2_id, prev_time_gps, prev_time_gps, r_2a_a);
          node->getSatelliteVector(sat_2_id, curr_time_gps, prev_time_gps, r_2a_b);

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

          pair_msg.set__prn1(sat_1_id);
          pair_msg.set__prn2(sat_2_id);

          meas_msg.pairs.push_back(pair_msg);
        }
        // publish the pseudo-measurement to be used by the back-end
        if (node->use_sim_time) {
          // todo: cutoff for older messages?

          auto time_diff = node->get_clock()->now().seconds() - meas_msg.t_b * 1e-9;     // todo: this may not be useful
          if (abs(time_diff) > 30) {
            std::cout << "Warning: " << time_diff << " second differential between sim_time and current measurement." << std::endl;
            std::cout << "Simulation time:  " << std::setprecision(12) << node->get_clock()->now().seconds() << std::endl;
            std::cout << "Measurement time: " << std::setprecision(12) << meas_msg.t_b * 1e-9 << std::endl;
          }

          while (node->get_clock()->now().seconds() < meas_msg.t_b * 1e-9){
            rclcpp::sleep_for(std::chrono::nanoseconds((long)1e6));
            rclcpp::spin_some(node);    // todo: may be better way
          }
        }
        node->publishTdcp(meas_msg);
      }
      // current {sats, code_pos} -> previous {sats, code_pos}
      node->stepForward();
    }

    // ephemeris message received. Keep track of which satellites have ephemeris
    if (rtcm_status == 2) {
      const auto &current_eph_sat = node->rtcm.ephsat;
      if (!node->eph_set_gps[current_eph_sat - MINPRNGPS]) {
        node->eph_count_gps++;
      }
      node->eph_set_gps[current_eph_sat] = true;
    }
  }

  rclcpp::shutdown();
  return 0;
}
