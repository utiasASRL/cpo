#pragma once

#include <rtklib.h>

#define LEAP_SECONDS 18   // todo: may want to use rtcm->nav.utc_gps[4] instead

/** \brief Stores data related to one satellite observation at one timestamp
 * Reuses RTKLIB's obsd_t struct with some additional calculated values
 * */
class SatelliteObs {

 public:
  explicit SatelliteObs(obsd_t obs, double stamp_in);

  /** \brief Returns phase measurement (adjusted for atmosphere if applicable) in metres */
  double getAdjPhaseRange() const;

  /** \brief Returns negation of loss-of-lock indicator */
  bool isPhaseLocked() const;

  /** \brief Return timestamp of observation as measured by the GNSS receiver */
  gtime_t getMeasTimestamp() const {
    return observation_.time;
  }

  /** \brief Return timestamp of when we received the observation */
  double getInTimestamp() const {
    return in_stamp_;
  }

 private:

  /** \brief Unix timestamp of when we received the phase measurement. Allows us to use consistent ROS time. */
  double in_stamp_;

  /** \brief Struct containing phase, pseudorange measurements among other things */
  obsd_t observation_;

  /** \brief The estimated tropospheric delay affecting the phase measurement [m] */
  double tropospheric_delay_;

  /** \brief The estimated ionospheric advance affecting the phase measurement [m] */
  double ionospheric_advance_;

};
