#include <CpoBackEnd.hpp>

int main(int argc, char **argv) {

  // initialize ROS2
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<CpoBackEnd>());
  rclcpp::shutdown();
  return 0;
}
