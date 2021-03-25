
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
  return false;   // todo
}

Eigen::Matrix<double, 1, 1> TdcpErrorEval::evaluate() const {
  return Eigen::Matrix<double, 1, 1>();     // todo
}

Eigen::Matrix<double, 1, 1> TdcpErrorEval::evaluate(const Eigen::Matrix<double, 1, 1> &lhs,
                                                    std::vector<Jacobian<1, 6>> *jacs) const {
  return Eigen::Matrix<double, 1, 1>();     // todo
}

} // steam
