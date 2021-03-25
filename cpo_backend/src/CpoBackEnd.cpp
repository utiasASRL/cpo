
#include <cpo_backend/CpoBackEnd.hpp>

CpoBackEnd::CpoBackEnd() : Node("cpo_back_end") {

  subscription_ = this->create_subscription<cpo_interfaces::msg::TDCP>(
      "tdcp", 10, std::bind(&CpoBackEnd::_tdcpCallback, this, std::placeholders::_1));

  publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovariance>("cpo_odometry", 10);

}

void CpoBackEnd::_tdcpCallback(const cpo_interfaces::msg::TDCP::SharedPtr msg) {

  std::cout << "pass " << msg->t_a << std::endl;

  /// TdcpErrorEval stuff
  // todo: will have to save(?) set of ErrorEvals/costs/the problem as member of CpoBackEnd. Still need to figure out

  if ("some_condition"){
    /// set up steam problem
    /// optimize
    /// call publisher if successful
  }

}
