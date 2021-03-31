//////////////////////////////////////////////////////////////////////////////////////////////
/// \file LogMapSo3Evaluator.cpp
///
/// \author
//////////////////////////////////////////////////////////////////////////////////////////////

#include <steam_extensions/LogMapSo3Evaluator.hpp>

#include <lgmath.hpp>

namespace steam {
namespace so3 {

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Constructor
//////////////////////////////////////////////////////////////////////////////////////////////
LogMapSo3Evaluator::LogMapSo3Evaluator(const RotationEvaluator::ConstPtr& rotation) : rotation_(rotation) {
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Pseudo constructor - return a shared pointer to a new instance
//////////////////////////////////////////////////////////////////////////////////////////////
LogMapSo3Evaluator::Ptr LogMapSo3Evaluator::MakeShared(const RotationEvaluator::ConstPtr& rotation) {
  return LogMapSo3Evaluator::Ptr(new LogMapSo3Evaluator(rotation));
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Returns whether or not an evaluator contains unlocked state variables
//////////////////////////////////////////////////////////////////////////////////////////////
bool LogMapSo3Evaluator::isActive() const {
  return rotation_->isActive();
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Adds references (shared pointers) to active state variables to the map output
//////////////////////////////////////////////////////////////////////////////////////////////
void LogMapSo3Evaluator::getActiveStateVariables(
    std::map<unsigned int, steam::StateVariableBase::Ptr>* outStates) const {
  rotation_->getActiveStateVariables(outStates);
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Evaluate the resultant 3x1 vector belonging to the so(3) algebra
//////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<double,3,1> LogMapSo3Evaluator::evaluate() const {
  return rotation_->evaluate().vec();
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Evaluate the resultant 6x1 vector belonging to the se(3) algebra and
///        sub-tree of evaluations
//////////////////////////////////////////////////////////////////////////////////////////////
EvalTreeNode<Eigen::Matrix<double,3,1> >* LogMapSo3Evaluator::evaluateTree() const {

  // Evaluate sub-trees
  EvalTreeNode<lgmath::so3::Rotation>* rotation = rotation_->evaluateTree();

  // Make new root node -- note we get memory from the pool
  EvalTreeNode<Eigen::Matrix<double,3,1> >* root = EvalTreeNode<Eigen::Matrix<double,3,1> >::pool.getObj();  //todo ??
  root->setValue(rotation->getValue().vec());

  // Add children
  root->addChild(rotation);

  // Return new root node
  return root;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Implementation for Block Automatic Differentiation
//////////////////////////////////////////////////////////////////////////////////////////////
template<int LHS_DIM, int INNER_DIM, int MAX_STATE_SIZE>
void LogMapSo3Evaluator::appendJacobiansImpl(
    const Eigen::Matrix<double,LHS_DIM,INNER_DIM>& lhs,
    EvalTreeNode<Eigen::Matrix<double,3,1> >* evaluationTree,
    std::vector<Jacobian<LHS_DIM,MAX_STATE_SIZE> >* outJacobians) const {

  // Check if rotation is active
  if (rotation_->isActive()) {
    Eigen::Matrix<double,LHS_DIM,INNER_DIM> newLhs = lhs * lgmath::so3::vec2jacinv(evaluationTree->getValue());
    rotation_->appendBlockAutomaticJacobians(newLhs,
                                              static_cast<EvalTreeNode<lgmath::so3::Rotation>*>(evaluationTree->childAt(0)),
                                              outJacobians);    // todo: look at this function, see if it works
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Evaluate the Jacobian tree
//////////////////////////////////////////////////////////////////////////////////////////////
void LogMapSo3Evaluator::appendBlockAutomaticJacobians(const Eigen::MatrixXd& lhs,
                                                    EvalTreeNode<Eigen::Matrix<double,3,1> >* evaluationTree,
                                                    std::vector<Jacobian<> >* outJacobians) const {
  this->appendJacobiansImpl(lhs,evaluationTree, outJacobians);
}

void LogMapSo3Evaluator::appendBlockAutomaticJacobians(const Eigen::Matrix<double,1,3>& lhs,
                                                    EvalTreeNode<Eigen::Matrix<double,3,1> >* evaluationTree,
                                                    std::vector<Jacobian<1,6> >* outJacobians) const {
  this->appendJacobiansImpl(lhs,evaluationTree, outJacobians);
}

void LogMapSo3Evaluator::appendBlockAutomaticJacobians(const Eigen::Matrix<double,2,3>& lhs,
                                                    EvalTreeNode<Eigen::Matrix<double,3,1> >* evaluationTree,
                                                    std::vector<Jacobian<2,6> >* outJacobians) const {
  this->appendJacobiansImpl(lhs,evaluationTree, outJacobians);
}

void LogMapSo3Evaluator::appendBlockAutomaticJacobians(const Eigen::Matrix<double,3,3>& lhs,
                                                    EvalTreeNode<Eigen::Matrix<double,3,1> >* evaluationTree,
                                                    std::vector<Jacobian<3,6> >* outJacobians) const {
  this->appendJacobiansImpl(lhs,evaluationTree, outJacobians);
}

void LogMapSo3Evaluator::appendBlockAutomaticJacobians(const Eigen::Matrix<double,4,3>& lhs,
                                                    EvalTreeNode<Eigen::Matrix<double,3,1> >* evaluationTree,
                                                    std::vector<Jacobian<4,6> >* outJacobians) const {
  this->appendJacobiansImpl(lhs,evaluationTree, outJacobians);
}

void LogMapSo3Evaluator::appendBlockAutomaticJacobians(const Eigen::Matrix<double,6,3>& lhs,
                                                    EvalTreeNode<Eigen::Matrix<double,3,1> >* evaluationTree,
                                                    std::vector<Jacobian<6,6> >* outJacobians) const {
  this->appendJacobiansImpl(lhs,evaluationTree, outJacobians);
}

} // so3
} // steam
