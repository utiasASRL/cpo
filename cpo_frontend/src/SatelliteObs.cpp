#include <SatelliteObs.hpp>

SatelliteObs::SatelliteObs(obsd_t obs, double stamp_in) : observation_(obs), in_stamp_(stamp_in) {}

void SatelliteObs::estimateTroposphericDelay(const double *satellite_pos_vel, const double *receiver_pos) {
  // estimate the tropospheric delay. this method uses average zenith delay from
  // Collins1996 and Niell mapping function

  // get approximate receiver position (lat, long, height)
  double receiver_pos_geodetic[3];
  ecef2pos(receiver_pos, receiver_pos_geodetic);

  // calculate elevation
  double unit_vec_ecef[3];
  double distance = geodist(satellite_pos_vel, receiver_pos, unit_vec_ecef);   // if ephemeris data missing, will get distance = -1
  if (distance < 0)
    return;
  double unit_vec_enu[3];
  ecef2enu(receiver_pos_geodetic, unit_vec_ecef, unit_vec_enu);
  double azel[2] = {-1, M_PI_2 - acos(unit_vec_enu[2])};

  double *mapfw = mat(1, 1);
  double niell_mf = tropmapf(observation_.time, receiver_pos_geodetic, azel, mapfw);
  tropospheric_delay_ = 2.3 * niell_mf + 0.14 * (*mapfw);

  printf("Tropospheric delay set to %f \n", tropospheric_delay_);
}

double SatelliteObs::getAdjPhaseRange() const {
  double phase_L1_rad = observation_.L[0];
  // note: assumes using GPS
  double wavelength_L1_GPS = CLIGHT / FREQ1;
  double phase_L1_m = phase_L1_rad * wavelength_L1_GPS;

  return phase_L1_m - tropospheric_delay_ + ionospheric_advance_;
}

bool SatelliteObs::isPhaseLocked() const {
  return !observation_.LLI[0];
}
