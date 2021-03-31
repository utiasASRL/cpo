//////////////////////////////////////////////////////////////////////////////////////////////
/// \file RollErrorEval.cpp
///
/// \author 
//////////////////////////////////////////////////////////////////////////////////////////////

#include <steam_extensions/RollErrorEval.hpp>

namespace steam {

RollErrorEval::RollErrorEval(const so3::RotationEvaluator::ConstPtr& C) {
  errorEvaluator_ = so3::LogMapSo3Evaluator::MakeShared(C);
}

bool RollErrorEval::isActive() const {
  return errorEvaluator_->isActive();
}

Eigen::Matrix<double,1,1> RollErrorEval::evaluate() const {
  Eigen::Vector3d proj{1,0,0};
  Eigen::Matrix<double,1,1> error = proj.transpose() * errorEvaluator_->evaluate();
  return error;
}

Eigen::Matrix<double,1,1> RollErrorEval::evaluate(const Eigen::Matrix<double,1,1>& lhs,
                                                       std::vector<Jacobian<1,6> >* jacs) const {

  // Check and initialize jacobian array
  if (jacs == NULL) {
    throw std::invalid_argument("Null pointer provided to return-input 'jacs' in evaluate");
  }
  jacs->clear();

  // Get evaluation tree
  EvalTreeHandle<Eigen::Matrix<double,3,1> > blkAutoEvalLogOfTransformDiff =
      errorEvaluator_->getBlockAutomaticEvaluation();


  // Get Jacobians
  Eigen::Vector3d proj{1,0,0};
//  Eigen::Matrix<double,6,1> proj;
//  proj << 1, 0, 0, 0, 0, 0;
  errorEvaluator_->appendBlockAutomaticJacobians(lhs * proj.transpose(), blkAutoEvalLogOfTransformDiff.getRoot(), jacs);

  // Return evaluation
  return evaluate();
}

} // steam
