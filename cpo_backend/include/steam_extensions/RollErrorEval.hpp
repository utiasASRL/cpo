//////////////////////////////////////////////////////////////////////////////////////////////
/// \file RollErrorEval.hpp
///
/// \author
//////////////////////////////////////////////////////////////////////////////////////////////

#ifndef STEAM_ROLL_ERROR_EVALUATOR_HPP
#define STEAM_ROLL_ERROR_EVALUATOR_HPP

#include <steam.hpp>
#include <steam_extensions/LogMapSo3Evaluator.hpp>

namespace steam {

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Transformation error function evaluator
//////////////////////////////////////////////////////////////////////////////////////////////
class RollErrorEval : public ErrorEvaluator<1,6>::type
{
 public:

  /// Convenience typedefs
  typedef boost::shared_ptr<RollErrorEval> Ptr;
  typedef boost::shared_ptr<const RollErrorEval> ConstPtr;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Constructor - error is difference between roll of C and zero (in Lie algebra space)
  //////////////////////////////////////////////////////////////////////////////////////////////
  RollErrorEval(const so3::RotationEvaluator::ConstPtr& C);

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Returns whether or not an evaluator contains unlocked state variables
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual bool isActive() const;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Evaluate the 1-d measurement error
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual Eigen::Matrix<double,1,1> evaluate() const;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Evaluate the 1-d measurement error and Jacobians
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual Eigen::Matrix<double,1,1> evaluate(const Eigen::Matrix<double,1,1>& lhs,
                                             std::vector<Jacobian<1,6> >* jacs) const;

 private:

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Error evaluator
  //////////////////////////////////////////////////////////////////////////////////////////////
  so3::LogMapSo3Evaluator::ConstPtr errorEvaluator_;
};

} // steam

#endif // STEAM_ROLL_ERROR_EVALUATOR_HPP