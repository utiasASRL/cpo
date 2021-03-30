
#include <TdcpErrorEval.hpp>

namespace steam {

TdcpErrorEval::TdcpErrorEval(double phi_dd,
                             se3::PositionEvaluator::ConstPtr &r_ba_ina,
                             so3::RotationEvaluator::ConstPtr &C_ag,   // temporary
//                const so3::ComposeRotationEvaluator::ConstPtr& C_ag,   //todo
                             const Eigen::Vector3d &r_1a_ing_ata,
                             const Eigen::Vector3d &r_1a_ing_atb,
                             const Eigen::Vector3d &r_2a_ing_ata,
                             const Eigen::Vector3d &r_2a_ing_atb)
    : phi_dd_(phi_dd),
      r_ba_ina_(r_ba_ina),
      C_ag_(C_ag),
      r_1a_ing_ata_(r_1a_ing_ata),
      r_1a_ing_atb_(r_1a_ing_atb),
      r_2a_ing_ata_(r_2a_ing_ata),
      r_2a_ing_atb_(r_2a_ing_atb),
      u_a21_((r_2a_ing_ata_ - r_1a_ing_ata_).normalized()) {
}

bool TdcpErrorEval::isActive() const {
  return r_ba_ina_->isActive() || C_ag_->isActive();
}

Eigen::Matrix<double, 1, 1> TdcpErrorEval::evaluate() const {

  double rho_1a = r_1a_ing_ata_.norm();
  double rho_2a = r_2a_ing_ata_.norm();
  double rho_1b = (r_1a_ing_atb_ - C_ag_->evaluate() * r_ba_ina_->evaluate()).norm();
  double rho_2b = (r_2a_ing_atb_ - C_ag_->evaluate() * r_ba_ina_->evaluate()).norm();

  double rho_dd = (rho_2b - rho_2a) - (rho_1b - rho_1a);
  double error = phi_dd_ - rho_dd;

  return Eigen::Matrix<double, 1, 1>{error};
}

Eigen::Matrix<double, 1, 1> TdcpErrorEval::evaluate(const Eigen::Matrix<double, 1, 1> &lhs,
                                                    std::vector<Jacobian<1, 6>> *jacs) const {
// Check and initialize Jacobian array
  if (jacs == nullptr) {
    throw std::invalid_argument("Null pointer provided to return-input 'jacs' in evaluate");
  }
  jacs->clear();

  // If current pose unlocked, add Jacobian from perturbing it
  if (r_ba_ina_->isActive()) {

    // Get evaluation tree
    EvalTreeHandle<Eigen::Vector3d> blkAutoEvalPosition = r_ba_ina_->getBlockAutomaticEvaluation();

    // Get evaluation from tree
    const Eigen::Vector3d &r_ba = blkAutoEvalPosition.getValue();

    // Get Jacobians
    Eigen::Matrix<double, 1, 3> J_1 = u_a21_.transpose() * C_ag_->evaluate().matrix().transpose();

    Eigen::Matrix<double, 1, 3> newLhs = -1 * lhs * J_1;
    r_ba_ina_->appendBlockAutomaticJacobians(newLhs, blkAutoEvalPosition.getRoot(), jacs);
  }

  // If vehicle orientation is unlocked, add Jacobian from perturbing it
  if (C_ag_->isActive()) {
    // Get evaluation tree
    EvalTreeHandle<lgmath::so3::Rotation> blkAutoEvalRotation = C_ag_->getBlockAutomaticEvaluation();

    // Get evaluation from tree
    const lgmath::so3::Rotation &C_ag = blkAutoEvalRotation.getValue();

    // manual implementation of skew-symmetric operator
    Eigen::Vector3d r = r_ba_ina_->evaluate();
    Eigen::Matrix3d r_hat;
    r_hat << 0, -r(2), r(1),
        r(2), 0, -r(0),
        -r(1), r(0), 0;

    // Get Jacobians
    Eigen::Matrix<double, 1, 3> J_2 = u_a21_.transpose() * C_ag_->evaluate().matrix().transpose() * r_hat;

    Eigen::Matrix<double, 1, 3> newLhs = -1 * lhs * J_2;
    C_ag_->appendBlockAutomaticJacobians(newLhs, blkAutoEvalRotation.getRoot(), jacs);
  }

  return evaluate();
}

} // steam
