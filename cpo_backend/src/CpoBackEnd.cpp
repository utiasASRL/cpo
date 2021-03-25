
#include <cpo_backend/CpoBackEnd.hpp>

CpoBackEnd::CpoBackEnd() : Node("cpo_back_end") {

  subscription_ = this->create_subscription<cpo_interfaces::msg::TDCP>(
      "tdcp", 10, std::bind(&CpoBackEnd::_tdcpCallback, this, std::placeholders::_1));

  publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovariance>("cpo_odometry", 10);

  // set up receiver-vehicle transform. todo: hard-coded for now but eventually make this configurable
  Eigen::Matrix4d T_gps_vehicle_eigen = Eigen::Matrix4d::Identity();
  T_gps_vehicle_eigen(0, 3) = -0.60;
  T_gps_vehicle_eigen(2, 3) = -0.52;
  auto T_gps_vehicle = lgmath::se3::TransformationWithCovariance(T_gps_vehicle_eigen);
  T_gps_vehicle.setZeroCovariance();
  tf_gps_vehicle_ = steam::se3::FixedTransformEvaluator::MakeShared(T_gps_vehicle);

  getParams();
}

void CpoBackEnd::_tdcpCallback(const cpo_interfaces::msg::TDCP::SharedPtr msg) {

  std::cout << "pass " << msg->t_a << std::endl;

  /// TdcpErrorEval stuff
  // todo: will have to save(?) set of ErrorEvals/costs/the problem as member of CpoBackEnd. Still need to figure out

  if ("some_condition") {
    /// set up steam problem
    /// optimize
    /// call publisher if successful
  }

}
void CpoBackEnd::getParams() {
  // todo: eventually want to setup as ROS2 params (this->declare_parameter<...) but for now will hard code

  double tdcp_cov = 0.001;
  double non_holo_y = 0.01;
  double non_holo_z = 0.01;
  double non_holo_roll = 0.0001;
  double non_holo_pitch = 0.01;
  double lin_acc_std_dev_x = 1.0;
  double lin_acc_std_dev_y = 0.1;
  double lin_acc_std_dev_z = 0.1;
  double ang_acc_std_dev_x = 0.001;
  double ang_acc_std_dev_y = 0.01;
  double ang_acc_std_dev_z = 0.1;

  tdcp_cov_ << tdcp_cov;

  Eigen::Array<double, 1, 4> non_holo_diag;
  non_holo_diag << non_holo_y, non_holo_z, non_holo_roll, non_holo_pitch;
  nonholonomic_cov_.setZero();
  nonholonomic_cov_.diagonal() = non_holo_diag;

  Eigen::Array<double, 1, 6> Qc_diag;
  Qc_diag << lin_acc_std_dev_x, lin_acc_std_dev_y, lin_acc_std_dev_z,
      ang_acc_std_dev_x, ang_acc_std_dev_y, ang_acc_std_dev_z;
  smoothing_factor_information_.setZero();
  smoothing_factor_information_.diagonal() = 1.0 / Qc_diag;
}
