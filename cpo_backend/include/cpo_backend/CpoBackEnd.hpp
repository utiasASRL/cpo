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

  /** \brief Helper/debugger function that prints total cost and number of terms for each group of cost terms
   * \param final  True if these are the final costs, if false assumed to be initial costs
   * */
  void printCosts(bool final);

  /** \brief Helper function */
  static Eigen::Vector3d toEigenVec3d(const geometry_msgs::msg::Vector3 &ros_vec);

 private:

  /** \brief Callback for TDCP msgs */
  void _tdcpCallback(cpo_interfaces::msg::TDCP::SharedPtr msg_in);

  /** \brief Timed callback to query trajectory if available and publish pose estimate */
  void _timedCallback();

  /** \brief Resets loss functions, cost terms, etc. at top of TDCP callback */
  void initializeProblem();

  /** \brief Clears window and starts over after major error occurs */
  void resetEstimator();

  /** \brief Store TDCP msg in our queue. Also checks times for dropped msgs */
  void addMsgToWindow(const cpo_interfaces::msg::TDCP::SharedPtr& msg);

  /** \brief Saves latest pose information to a CSV file for later analysis */
  void saveToFile(double t_n,
                  double t_n1,
                  const lgmath::se3::Transformation &T_ng,
                  const lgmath::se3::Transformation &T_n_n1) const;

  /** \brief Takes in Transforms and calls ROS publishers */
  void publishPoses(const lgmath::se3::TransformationWithCovariance &T_ng,
                    const lgmath::se3::TransformationWithCovariance &T_n_n1);

  /** \brief Helper to convert lgmath Transform to ROS2 msg */
  static geometry_msgs::msg::PoseWithCovariance toPoseMsg(lgmath::se3::TransformationWithCovariance T);

  /** \brief Subscriber for TDCP msgs */
  rclcpp::Subscription<cpo_interfaces::msg::TDCP>::SharedPtr subscription_;

  /** \brief Publisher of integrated odometry in ENU frame
   * \note Absolute accuracy of these poses will not be high after a long period of dead-reckoning but they are useful
   * for plotting
   * */
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovariance>::SharedPtr enu_publisher_;

  /** \brief Publisher of odometry transforms (relative vehicle frame poses) */
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovariance>::SharedPtr vehicle_publisher_;

  /** \brief Whether to publish transform estimates at fixed rate (true) or after each msg received (false) */
  bool fixed_rate_publish_;

  /** Timer to publish odometry estimates at a fixed rate */
  rclcpp::TimerBase::SharedPtr publish_timer_;

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
  steam::ParallelizedCostTermCollection::Ptr pose_prior_cost_;

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

  /** \brief File path for saving a CSV file of our estimates */
  std::string results_path_;

  /** \brief Whether to print extra information about the optimization */
  bool steam_verbose_;

  /** \brief Maximum number of solver iterations per optimization */
  uint steam_max_iterations_;

  /** \brief Our estimate of T_0g, stored to initialize the next optimization problem */
  lgmath::se3::Transformation init_pose_;

  /** \brief DEBUGGING Keeping track of T_0g separately to detect lgmath reproject issues */
  Eigen::Matrix4d init_pose_eig_;

  /** \brief Keep track of whether we have T_0g estimate or need to get one from code solution */
  bool init_pose_estimated_ = false;

  /** \brief Store a window of TDCP messages. We use deque over queue to get access */
  std::deque<std::pair<cpo_interfaces::msg::TDCP, lgmath::se3::Transformation>> msgs_;

  /** \brief velocities. todo: better data structure for this and above */
  std::deque<Eigen::Matrix<double, 6, 1>> vels_;

  /** \brief Size of the optimization window in msgs */
  uint window_size_;

  /** \brief If true, T_0g will be locked within a window once first estimated.
   *   Otherwise we'll apply a prior and re-estimate each loop.
   */
  bool lock_first_pose_;

  /** \brief Time period (in seconds) since last msg we will trust our odometry estimates */
  double traj_timeout_limit_;

  bool first_window_ = true;

  // debug
  double last_init_smoothing_cost_ = 0;

};
