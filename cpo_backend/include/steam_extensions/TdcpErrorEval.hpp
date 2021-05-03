#pragma once

#include <steam.hpp>

namespace steam {

/** \brief Time-Differenced Carrier Phase Error Evaluator for STEAM optimization */
class TdcpErrorEval : public ErrorEvaluator<1, 6>::type {
 public:
  /** \brief Constructor */
  TdcpErrorEval(double phi_dd,
                se3::PositionEvaluator::ConstPtr &r_ba_ina,
                se3::TransformEvaluator::ConstPtr &T_ag,
                Eigen::Vector3d r_1a_ing_ata,
                Eigen::Vector3d r_1a_ing_atb,
                Eigen::Vector3d r_2a_ing_ata,
                Eigen::Vector3d r_2a_ing_atb);

  /** \brief Returns whether or not an evaluator contains unlocked state variables */
  virtual bool isActive() const;

  /** \brief Evaluate the 1-d measurement error */
  virtual Eigen::Matrix<double, 1, 1> evaluate() const;

  /** \brief Evaluate the 1-d measurement error and Jacobians */
  virtual Eigen::Matrix<double, 1, 1> evaluate(const Eigen::Matrix<double, 1, 1> &lhs,
                                               std::vector<Jacobian<1, 6> > *jacs) const;

 private:
  /** The double-differenced phase-range pseudo-measurement */
  const double phi_dd_;

  /** Vector between the vehicle position at the two times in the local frame */
  se3::PositionEvaluator::ConstPtr r_ba_ina_;

  /** Estimated rotation between the vehicle frame at a and the ENU frame */
  se3::TransformEvaluator::ConstPtr T_ag_;

  /** Vector to satellite 1 from position a at time a in ENU frame */
  const Eigen::Vector3d r_1a_ing_ata_;

  /** Vector to satellite 1 from position a at time b in ENU frame */
  const Eigen::Vector3d r_1a_ing_atb_;

  /** Vector to satellite 2 from position a at time a in ENU frame */
  const Eigen::Vector3d r_2a_ing_ata_;

  /** Vector to satellite 2 from position a at time b in ENU frame */
  const Eigen::Vector3d r_2a_ing_atb_;

  /** Unit vector version of  (r_2a_ing_ata_ - r_1a_ing_ata_). Used in Jacobian */
  const Eigen::Vector3d u_a21_;
};
} // steam