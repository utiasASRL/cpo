#include <iostream>
#include <fstream>

#include <CpoFrontEnd.hpp>

int main(int argc, char **argv) {

  // initialize ROS2 and create our node
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CpoFrontEnd>();

  unsigned char byte_in;
  int rtcm_status;

  _IO_FILE *fp;
  std::ofstream fs;

  if (!node->from_serial) {
    fp = fopen(node->rtcm_path.c_str(), "r");
  } else if (node->log_serial) {
    fs = std::ofstream(node->log_serial_path, std::ios::out | std::ios::binary);
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
      if (satsys(node->rtcm.obs.data->sat, nullptr) != SYS_GPS) {
        std::cout << "Only GPS supported currently. Not using this observation."
                  << std::endl;
      }

      if (node->use_sim_time) {
        // if using simulated time, check that msg times in the ballpark
        auto diff = (long) node->get_clock()->now().seconds()
            - (node->rtcm.obs.data->time.time - LEAP_SECONDS);
        if (diff > 10) {
          std::cout << "Current message time is " << diff
                    << " seconds older than current simulated time so will skip to next message. "
                    << std::endl;
          continue;
        } else if (diff < -100) {
          std::cout << "Current message time is " << -diff
                    << " seconds ahead of simulated time. Check sim_time started at correct value."
                    << std::endl;
        }
      }

      for (unsigned i = 0; i < (unsigned)node->rtcm.obs.n; ++i) {
        obsd_t &obs = node->rtcm.obs.data[i];

        // check the observation is from a GPS positioning satellite
        // (as opposed to SBAS). Currently GPS only constellation supported.
        if (obs.sat < MINPRNGPS || obs.sat > MAXPRNGPS) continue;

        double now_time = node->get_clock()->now().seconds();     // UTC time
        node->curr_sats->insert(std::make_pair(obs.sat,
                                               SatelliteObs(obs, now_time)));
      }

      std::cout << "Observed " << node->curr_sats->size() << " satellites:  ";
      for (const auto &sat : *node->curr_sats)
        std::cout << (int) sat.first << ", ";
      std::cout << std::endl;

      // get approximate position through single-point (pseudo-range) positioning
      sol_t code_solution;
      char error_msg[128];
      bool success = pntpos(node->rtcm.obs.data,
                            node->rtcm.obs.n,
                            &node->rtcm.nav,
                            &node->code_positioning_options,
                            &code_solution,
                            nullptr,
                            nullptr,
                            error_msg);

      if (success) {
        // define the ENU origin if this is the first solution calculated
        if (!node->enu_origin_set) {
          node->setEnuOrigin(&code_solution.rr[0]);
        }
        node->updateCodePos(&code_solution.rr[0]);
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

        gtime_t t_a_gps = sat_1a.getMeasTimestamp();
        gtime_t t_b_gps = sat_1b.getMeasTimestamp();

        if (node->enable_tropospheric_correction) {
          // calculate atmospheric corrections here to ensure we have an ephemeris
          double *sat_pos_vel_a;
          double *sat_pos_vel_b;
          node->getSatellitePosition(sat_1_id,
                                     t_a_gps,
                                     t_a_gps,
                                     sat_pos_vel_a);
          node->getSatellitePosition(sat_1_id,
                                     t_b_gps,
                                     t_b_gps,
                                     sat_pos_vel_b);

          // only rough receiver position needed so we reuse code_solution
          sat_1a.estimateTroposphericDelay(sat_pos_vel_a, code_solution.rr);
          sat_1b.estimateTroposphericDelay(sat_pos_vel_b, code_solution.rr);
        }

        // convert to UTC time for use by other packages
        const long sec_to_ns = 1e9;
        meas_msg.t_a = (sec_to_ns * t_a_gps.time) + (long) (1e9 * t_a_gps.sec)
            - (sec_to_ns * LEAP_SECONDS);
        meas_msg.t_b = (sec_to_ns * t_b_gps.time) + (long) (1e9 * t_b_gps.sec)
            - (sec_to_ns * LEAP_SECONDS);

        Eigen::Vector3d current_code = node->getCurrentCodePos();
        meas_msg.enu_pos.set__x(current_code.x());
        meas_msg.enu_pos.set__y(current_code.y());
        meas_msg.enu_pos.set__z(current_code.z());
        Eigen::Vector3d previous_code = node->getPreviousCodePos();
        meas_msg.prev_enu_pos.set__x(previous_code.x());
        meas_msg.prev_enu_pos.set__y(previous_code.y());
        meas_msg.prev_enu_pos.set__z(previous_code.z());

        Eigen::Vector3d enu_origin = node->getGeodeticEnuOrigin();
        meas_msg.enu_origin.set__x(enu_origin.x());
        meas_msg.enu_origin.set__y(enu_origin.y());
        meas_msg.enu_origin.set__z(enu_origin.z());

        // calculate vectors to the 1st satellite
        Eigen::Vector3d r_1a_a;
        Eigen::Vector3d r_1a_b;
        node->getSatelliteVector(sat_1_id, t_a_gps, t_a_gps, r_1a_a);
        node->getSatelliteVector(sat_1_id, t_b_gps, t_a_gps, r_1a_b);

        for (unsigned int j = 1; j < matches.size(); ++j) {
          int sat_2_id = matches[j];
          auto &sat_2a = node->prev_sats->at(sat_2_id);
          auto &sat_2b = node->curr_sats->at(sat_2_id);

          if (node->enable_tropospheric_correction) {
            double *sat_pos_vel_a;
            double *sat_pos_vel_b;
            node->getSatellitePosition(sat_2_id,
                                       t_a_gps,
                                       t_a_gps,
                                       sat_pos_vel_a);
            node->getSatellitePosition(sat_2_id,
                                       t_b_gps,
                                       t_b_gps,
                                       sat_pos_vel_b);
            sat_2a.estimateTroposphericDelay(sat_pos_vel_a, code_solution.rr);
            sat_2b.estimateTroposphericDelay(sat_pos_vel_b, code_solution.rr);
          }

          double phi_dd =
              (sat_2b.getAdjPhaseRange() - sat_2a.getAdjPhaseRange())
                  - (sat_1b.getAdjPhaseRange() - sat_1a.getAdjPhaseRange());

          // calculate vectors to 2nd satellite
          Eigen::Vector3d r_2a_a = Eigen::Vector3d::Zero();   // placeholder
          Eigen::Vector3d r_2a_b = Eigen::Vector3d::Zero();   // placeholder
          node->getSatelliteVector(sat_2_id, t_a_gps, t_a_gps, r_2a_a);
          node->getSatelliteVector(sat_2_id, t_b_gps, t_a_gps, r_2a_b);

          cpo_interfaces::msg::SatPair pair_msg = node->fillPairMsg(phi_dd,
                                                                    r_1a_a,
                                                                    r_1a_b,
                                                                    r_2a_a,
                                                                    r_2a_b,
                                                                    sat_1_id,
                                                                    sat_2_id);
          meas_msg.pairs.push_back(pair_msg);
        }
        // publish the pseudo-measurement to be used by the back-end
        if (node->use_sim_time) {
          // if using simulated time, wait for /clock to match timestamp b
          while (node->get_clock()->now().seconds() < meas_msg.t_b * 1e-9) {
            rclcpp::sleep_for(std::chrono::nanoseconds((long) 1e6));
            rclcpp::spin_some(node);
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
