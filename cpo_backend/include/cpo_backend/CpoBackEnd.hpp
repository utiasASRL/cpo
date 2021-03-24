#pragma once

#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>

#include <cpo_interfaces/msg/tdcp.hpp>


/** \brief Class that subscribes to TDCP pseudo-measurements and outputs odometry estimates */
class CpoBackEnd : public rclcpp::Node {
 public:

  /** \brief Constructor */
  CpoBackEnd();

 private:

  /** \brief Callback for TDCP msgs */
  void _tdcpCallback(cpo_interfaces::msg::TDCP::SharedPtr msg);

  /** \brief Subscriber for TDCP msgs */
  rclcpp::Subscription<cpo_interfaces::msg::TDCP>::SharedPtr subscription_;


};
