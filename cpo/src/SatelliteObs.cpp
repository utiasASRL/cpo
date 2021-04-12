#include <SatelliteObs.hpp>

SatelliteObs::SatelliteObs(obsd_t obs, double stamp_in) : observation_(obs), in_stamp_(stamp_in) {

  tropospheric_delay_ = 0;      // todo
  ionospheric_advance_ = 0;
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
