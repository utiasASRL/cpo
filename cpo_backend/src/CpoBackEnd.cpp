
#include <cpo_backend/CpoBackEnd.hpp>

CpoBackEnd::CpoBackEnd() : Node("cpo_back_end") {

  subscription_ = this->create_subscription<cpo_interfaces::msg::TDCP>(
      "tdcp", 10, std::bind(&CpoBackEnd::_tdcpCallback, this, std::placeholders::_1));

}

void CpoBackEnd::_tdcpCallback(const cpo_interfaces::msg::TDCP::SharedPtr msg) {

  std::cout << "pass " << msg->t_a << std::endl;
}
