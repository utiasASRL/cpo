#pragma once

#include <steam.hpp>

namespace steam {

/** \brief Error evaluator for unicycle model */
class UnicycleErrorEval : public ErrorEvaluator<4, 6>::type {
 public:

  // Convenience typedefs
  using Ptr = boost::shared_ptr<UnicycleErrorEval>;
  using ConstPtr = boost::shared_ptr<const UnicycleErrorEval>;
  
  /** \brief Constructor - Error between y,z,roll,pitch velocities and zero */
  explicit UnicycleErrorEval(const VectorSpaceStateVar::Ptr &state_vec);
  
  /** \brief Returns whether or not an evaluator contains unlocked state variables */
  virtual bool isActive() const;
  
  /** \brief Evaluate the 4-d measurement error */
  virtual Eigen::Matrix<double, 4, 1> evaluate() const;
  
  /** \brief Evaluate the 4-d measurement error and Jacobian */
  virtual Eigen::Matrix<double, 4, 1> evaluate(const Eigen::Matrix<double, 4, 4> &lhs,
                                               std::vector<Jacobian<4, 6>
                                               > *jacs) const;

 private:

  /** \brief Velocity state vector */
  VectorSpaceStateVar::ConstPtr state_vec_;

};

} // steam
