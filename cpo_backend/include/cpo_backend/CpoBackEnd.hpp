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

  /** \brief Get parameters from ROS2 and set appropriate fields */
  void getParams();

  /** \brief Helper/debugger function that prints total cost and number of terms for each group of cost terms */
  void printCosts();

 private:

  /** \brief Callback for TDCP msgs */
  void _tdcpCallback(cpo_interfaces::msg::TDCP::SharedPtr msg);

  void resetProblem();

  /** \brief Subscriber for TDCP msgs */
  rclcpp::Subscription<cpo_interfaces::msg::TDCP>::SharedPtr subscription_;

  /** \brief Publisher of odometry transforms (relative vehicle frame poses) */
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovariance>::SharedPtr publisher_;

  /** \brief The fixed sensor-vehicle transform. Allows us to do estimation in the vehicle frame */
  steam::se3::FixedTransformEvaluator::ConstPtr tf_gps_vehicle_;

  std::shared_ptr<steam::SolverBase> solver_;

  /** \brief The steam problem. */
  std::shared_ptr<steam::OptimizationProblem> problem_;

  /** \brief Cost terms associated with TDCP observations. */
  steam::ParallelizedCostTermCollection::Ptr tdcp_cost_terms_;

  /** \brief Cost terms associated with nonholonomic prior. */
  steam::ParallelizedCostTermCollection::Ptr nonholonomic_cost_terms_;      // todo: may need some other terms from gpso

  /** \brief Cost terms associated with white-noise-on-acceleration motion prior */
  steam::ParallelizedCostTermCollection::Ptr smoothing_cost_terms_;

  /** \brief Cost term for prior on C_ag roll */
  steam::WeightedLeastSqCostTerm<1, 6>::Ptr roll_cost_term_;

  /** \brief Loss function associated with TDCP costs */
  steam::LossFunctionBase::Ptr tdcp_loss_function_;

  /** \brief Loss function associated with nonholonomic costs */
  steam::LossFunctionBase::Ptr nonholonomic_loss_function_;

  /** \brief Loss function associated with roll prior */
  steam::LossFunctionBase::Ptr roll_loss_function_;

  /** \brief The steam trajectory, allows smoothing factors, velocity priors and pose extrapolation */
  std::shared_ptr<steam::se3::SteamTrajInterface> trajectory_;

  Eigen::Matrix<double, 6, 6> smoothing_factor_information_;    // todo: may want to put this stuff in Config struct

  Eigen::Matrix<double, 1, 1> tdcp_cov_;

  Eigen::Matrix<double, 4, 4> nonholonomic_cov_;

  Eigen::Matrix<double, 1, 1> roll_cov_;

  /** \brief Our estimate of C_ag, stored to initialize the next optimization problem */
  lgmath::so3::Rotation approx_rotation_;

};
