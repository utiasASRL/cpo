#pragma once

#include <Eigen/Core>
#include <steam.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>

#include <cpo_interfaces/msg/tdcp.hpp>

#include <deque>

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
  void _tdcpCallback(cpo_interfaces::msg::TDCP::SharedPtr msg_in);

  void resetProblem();

  /** \brief Store TDCP msg in our queue. Also checks times for dropped msgs */
  void addMsgToWindow(const cpo_interfaces::msg::TDCP::SharedPtr& msg);

  /** \brief Helper to convert lgmath Transform to ROS2 msg */
  static geometry_msgs::msg::PoseWithCovariance toPoseMsg(const lgmath::se3::Transformation& T,
                                                   const Eigen::Matrix<double, 6, 6>& cov);

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
  steam::ParallelizedCostTermCollection::Ptr nonholonomic_cost_terms_;

  /** \brief Cost terms associated with white-noise-on-acceleration motion prior */
  steam::ParallelizedCostTermCollection::Ptr smoothing_cost_terms_;

  /** \brief Cost term for prior on T_0g to help resolve roll, lock position */
  steam::WeightedLeastSqCostTerm<6, 6>::Ptr pose_prior_cost_;

  /** \brief Loss function associated with TDCP costs */
  steam::LossFunctionBase::Ptr tdcp_loss_function_;

  /** \brief Loss function associated with nonholonomic costs */
  steam::LossFunctionBase::Ptr nonholonomic_loss_function_;

  /** \brief Loss function associated with roll prior */
  steam::LossFunctionBase::Ptr pp_loss_function_;

  /** \brief The steam trajectory, allows smoothing factors, velocity priors and pose extrapolation */
  std::shared_ptr<steam::se3::SteamTrajInterface> trajectory_;

  Eigen::Matrix<double, 6, 6> smoothing_factor_information_;    // todo: may want to put this stuff in Config struct

  Eigen::Matrix<double, 1, 1> tdcp_cov_;

  Eigen::Matrix<double, 4, 4> nonholonomic_cov_;

  Eigen::Matrix<double, 6, 6> pose_prior_cov_;

  /** \brief Our estimate of T_ag, stored to initialize the next optimization problem */
  lgmath::se3::Transformation init_pose_;     // todo: not sure if we still need

  /** \brief Store a window of TDCP messages. We use deque over queue to get access */
  std::deque<std::pair<cpo_interfaces::msg::TDCP, lgmath::se3::Transformation>> msgs_;

  /** \brief Size of the optimization window in msgs */
  uint window_size_;

};
