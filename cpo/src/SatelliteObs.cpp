#include <SatelliteObs.hpp>

SatelliteObs::SatelliteObs(obsd_t obs) : observation(obs) {

  tropospheric_delay = 0;
  ionospheric_advance = 0;
}
