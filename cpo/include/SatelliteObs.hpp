#pragma once

#include <rtklib.h>

/** \brief Stores data related to one satellite observation at one timestamp
 * Reuses RTKLIB's obsd_t struct with some additional calculated values
 * */
class SatelliteObs {

 public:
  explicit SatelliteObs(obsd_t obs);

  /** \brief Returns phase measurement (adjusted for atmosphere if applicable) in metres */
  double getAdjPhaseRange() const;

  /** \brief Returns negation of loss-of-lock indicator */
  bool isPhaseLocked() const;

  /** \brief Return timestamp of observation */
  gtime_t getTimestamp() const {
    return observation_.time;
  }

 private:

  /** \brief Struct containing phase, pseudorange measurements among other things */
  obsd_t observation_;

  /** \brief The estimated tropospheric delay affecting the phase measurement [m] */
  double tropospheric_delay_;

  /** \brief The estimated ionospheric advance affecting the phase measurement [m] */
  double ionospheric_advance_;

};
