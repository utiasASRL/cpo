//////////////////////////////////////////////////////////////////////////////////////////////
/// \file RotationStateEvaluator.hpp
///
/// \author Ben Congram, based on Sean Anderson's RotationStateEvaluator.cpp
//////////////////////////////////////////////////////////////////////////////////////////////

#ifndef STEAM_ROTATION_STATE_EVALUATOR_HPP
#define STEAM_ROTATION_STATE_EVALUATOR_HPP

#include <RotationEvaluator.hpp>

using RotationStateVar = steam::LieGroupStateVar<lgmath::so3::Rotation, 3>;

namespace steam {
namespace so3 {

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Simple transform evaluator for a transformation state variable
//////////////////////////////////////////////////////////////////////////////////////////////
class RotationStateEvaluator : public RotationEvaluator
{
 public:

  /// Convenience typedefs
  typedef boost::shared_ptr<RotationStateEvaluator> Ptr;
  typedef boost::shared_ptr<const RotationStateEvaluator> ConstPtr;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Constructor
  //////////////////////////////////////////////////////////////////////////////////////////////
  RotationStateEvaluator(const RotationStateVar::Ptr& rotation);

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Pseudo constructor - return a shared pointer to a new instance
  //////////////////////////////////////////////////////////////////////////////////////////////
  static Ptr MakeShared(const RotationStateVar::Ptr& rotation);

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Returns whether or not an evaluator contains unlocked state variables
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual bool isActive() const;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Adds references (shared pointers) to active state variables to the map output
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual void getActiveStateVariables(
      std::map<unsigned int, steam::StateVariableBase::Ptr>* outStates) const;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Evaluate the transformation matrix
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual lgmath::so3::Rotation evaluate() const;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Evaluate the transformation matrix tree
  ///
  /// ** Note that the returned pointer belongs to the memory pool EvalTreeNode<TYPE>::pool,
  ///    and should be given back to the pool, rather than being deleted.
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual EvalTreeNode<lgmath::so3::Rotation>* evaluateTree() const;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Evaluate the Jacobian tree
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual void appendBlockAutomaticJacobians(const Eigen::MatrixXd& lhs,
                                             EvalTreeNode<lgmath::so3::Rotation>* evaluationTree,
                                             std::vector<Jacobian<> >* outJacobians) const;

  virtual void appendBlockAutomaticJacobians(const Eigen::Matrix<double,1,3>& lhs,
                                             EvalTreeNode<lgmath::so3::Rotation>* evaluationTree,
                                             std::vector<Jacobian<1,3> >* outJacobians) const;

  virtual void appendBlockAutomaticJacobians(const Eigen::Matrix<double,2,3>& lhs,
                                             EvalTreeNode<lgmath::so3::Rotation>* evaluationTree,
                                             std::vector<Jacobian<2,3> >* outJacobians) const;

  virtual void appendBlockAutomaticJacobians(const Eigen::Matrix<double,3,3>& lhs,
                                             EvalTreeNode<lgmath::so3::Rotation>* evaluationTree,
                                             std::vector<Jacobian<3,3> >* outJacobians) const;

  virtual void appendBlockAutomaticJacobians(const Eigen::Matrix<double,4,3>& lhs,
                                             EvalTreeNode<lgmath::so3::Rotation>* evaluationTree,
                                             std::vector<Jacobian<4,3> >* outJacobians) const;

  virtual void appendBlockAutomaticJacobians(const Eigen::Matrix<double,6,3>& lhs,
                                             EvalTreeNode<lgmath::so3::Rotation>* evaluationTree,
                                             std::vector<Jacobian<6,3> >* outJacobians) const;

 private:

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Implementation for Block Automatic Differentiation
  //////////////////////////////////////////////////////////////////////////////////////////////
  template<int LHS_DIM, int INNER_DIM, int MAX_STATE_SIZE>
  void appendJacobiansImpl(const Eigen::Matrix<double,LHS_DIM,INNER_DIM>& lhs,
                           EvalTreeNode<lgmath::so3::Rotation>* evaluationTree,
                           std::vector<Jacobian<LHS_DIM,MAX_STATE_SIZE> >* outJacobians) const;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Rotation matrix state variable
  //////////////////////////////////////////////////////////////////////////////////////////////
  RotationStateVar::Ptr rotation_;

};

} // so3
} // steam

#endif // STEAM_ROTATION_STATE_EVALUATOR_HPP
