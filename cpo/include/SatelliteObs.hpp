#pragma once

#include <rtklib.h>

/** \brief Stores data related to one satellite observation at one timestamp
 * Reuses RTKLIB's obsd_t struct with some additional calculated values
 * */
class SatelliteObs {

 public:
  explicit SatelliteObs(obsd_t obs);

  /** \brief Struct containing phase, pseudorange measurements among other things */
  obsd_t observation;

  /** \brief The estimated tropospheric delay affecting the phase measurement [m] */
  double tropospheric_delay;

  /** \brief The estimated ionospheric advance affecting the phase measurement [m] */
  double ionospheric_advance;

};
