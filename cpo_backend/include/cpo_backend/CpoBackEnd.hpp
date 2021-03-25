#pragma once

#include <Eigen/Core>
#include <steam.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>

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

  /** \brief Publisher of odometry transforms (relative vehicle frame poses) */
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovariance>::SharedPtr publisher_;

};
