#pragma once

#include <steam.hpp>

namespace steam {

/** \brief Time-Differenced Carrier Phase Error Evaluator for STEAM optimization */
class TdcpErrorEval : public ErrorEvaluator<1, 6>::type {     // todo: not sure if this is still 6
 public:

  /** \brief Constructor */
//  TdcpErrorEval(...); //todo

  /** \brief Returns whether or not an evaluator contains unlocked state variables */
  virtual bool isActive() const;

  /** \brief Evaluate the 1-d measurement error */
  virtual Eigen::Matrix<double, 1, 1> evaluate() const;

  /** \brief Evaluate the 1-d measurement error and Jacobians */
  virtual Eigen::Matrix<double, 1, 1> evaluate(const Eigen::Matrix<double, 1, 1> &lhs,
                                               std::vector<Jacobian<1, 6> > *jacs) const;

 private:

  // todo

};

} // steam