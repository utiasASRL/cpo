//////////////////////////////////////////////////////////////////////////////////////////////
/// \file RotationStateEvaluator.cpp
///
/// \author Ben Congram, based on Sean Anderson's TransformStateEvaluator.cpp
//////////////////////////////////////////////////////////////////////////////////////////////

#include <RotationStateEvaluator.hpp>

namespace steam {
namespace so3 {

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Constructor
//////////////////////////////////////////////////////////////////////////////////////////////
RotationStateEvaluator::RotationStateEvaluator(const RotationStateVar::Ptr& rotation) : rotation_(rotation) {
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Pseudo constructor - return a shared pointer to a new instance
//////////////////////////////////////////////////////////////////////////////////////////////
RotationStateEvaluator::Ptr RotationStateEvaluator::MakeShared(const RotationStateVar::Ptr& rotation) {
  return RotationStateEvaluator::Ptr(new RotationStateEvaluator(rotation));
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Returns whether or not an evaluator contains unlocked state variables
//////////////////////////////////////////////////////////////////////////////////////////////
bool RotationStateEvaluator::isActive() const {
  return !rotation_->isLocked();
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Adds references (shared pointers) to active state variables to the map output
//////////////////////////////////////////////////////////////////////////////////////////////
void RotationStateEvaluator::getActiveStateVariables(
    std::map<unsigned int, steam::StateVariableBase::Ptr>* outStates) const {
  if (this->isActive()) {
    (*outStates)[rotation_->getKey().getID()] = rotation_;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Evaluate the transformation matrix
//////////////////////////////////////////////////////////////////////////////////////////////
lgmath::so3::Rotation RotationStateEvaluator::evaluate() const {
  return rotation_->getValue();
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Evaluate the transformation matrix tree
//////////////////////////////////////////////////////////////////////////////////////////////
EvalTreeNode<lgmath::so3::Rotation>* RotationStateEvaluator::evaluateTree() const {

  // Make new leaf node -- note we get memory from the pool
  EvalTreeNode<lgmath::so3::Rotation>* result = EvalTreeNode<lgmath::so3::Rotation>::pool.getObj();
  result->setValue(rotation_->getValue());
  return result;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Implementation for Block Automatic Differentiation
//////////////////////////////////////////////////////////////////////////////////////////////
template<int LHS_DIM, int INNER_DIM, int MAX_STATE_SIZE>
void RotationStateEvaluator::appendJacobiansImpl(
    const Eigen::Matrix<double,LHS_DIM,INNER_DIM>& lhs,
    EvalTreeNode<lgmath::so3::Rotation>* evaluationTree,
    std::vector<Jacobian<LHS_DIM,MAX_STATE_SIZE> >* outJacobians) const {
  if (!rotation_->isLocked()) {
    outJacobians->push_back(Jacobian<LHS_DIM,MAX_STATE_SIZE>(rotation_->getKey(), lhs));
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Evaluate the Jacobian tree
//////////////////////////////////////////////////////////////////////////////////////////////
void RotationStateEvaluator::appendBlockAutomaticJacobians(const Eigen::MatrixXd& lhs,
                                                            EvalTreeNode<lgmath::so3::Rotation>* evaluationTree,
                                                            std::vector<Jacobian<> >* outJacobians) const {
  this->appendJacobiansImpl(lhs,evaluationTree, outJacobians);
}

void RotationStateEvaluator::appendBlockAutomaticJacobians(const Eigen::Matrix<double,1,3>& lhs,
                                                            EvalTreeNode<lgmath::so3::Rotation>* evaluationTree,
                                                            std::vector<Jacobian<1,3> >* outJacobians) const {
  this->appendJacobiansImpl(lhs,evaluationTree, outJacobians);
}

void RotationStateEvaluator::appendBlockAutomaticJacobians(const Eigen::Matrix<double,2,3>& lhs,
                                                            EvalTreeNode<lgmath::so3::Rotation>* evaluationTree,
                                                            std::vector<Jacobian<2,3> >* outJacobians) const {
  this->appendJacobiansImpl(lhs,evaluationTree, outJacobians);
}

void RotationStateEvaluator::appendBlockAutomaticJacobians(const Eigen::Matrix<double,3,3>& lhs,
                                                            EvalTreeNode<lgmath::so3::Rotation>* evaluationTree,
                                                            std::vector<Jacobian<3,3> >* outJacobians) const {
  this->appendJacobiansImpl(lhs,evaluationTree, outJacobians);
}

void RotationStateEvaluator::appendBlockAutomaticJacobians(const Eigen::Matrix<double,4,3>& lhs,
                                                            EvalTreeNode<lgmath::so3::Rotation>* evaluationTree,
                                                            std::vector<Jacobian<4,3> >* outJacobians) const {
  this->appendJacobiansImpl(lhs,evaluationTree, outJacobians);
}

void RotationStateEvaluator::appendBlockAutomaticJacobians(const Eigen::Matrix<double,6,3>& lhs,
                                                            EvalTreeNode<lgmath::so3::Rotation>* evaluationTree,
                                                            std::vector<Jacobian<6,3> >* outJacobians) const {
  this->appendJacobiansImpl(lhs,evaluationTree, outJacobians);
}

} // se3
} // steam
