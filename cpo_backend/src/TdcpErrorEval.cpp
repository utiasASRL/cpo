
#include <cpo_backend/TdcpErrorEval.hpp>

namespace steam {

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
