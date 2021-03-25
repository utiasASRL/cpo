#include <TdcpErrorEval.hpp>
#include <RotationStateEvaluator.hpp>

#include <cpo_backend/CpoBackEnd.hpp>

using TransformStateVar = steam::se3::TransformStateVar;
using TransformStateEvaluator = steam::se3::TransformStateEvaluator;
using SteamTrajVar = steam::se3::SteamTrajVar;
using VectorSpaceStateVar = steam::VectorSpaceStateVar;
using PositionEvaluator = steam::se3::PositionEvaluator;
using RotationEvaluator = steam::so3::RotationEvaluator;

using RotationStateVar = steam::LieGroupStateVar<lgmath::so3::Rotation, 3>;

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
    resetProblem();

    /// set up steam problem
    // for now just using one pair of poses

    Eigen::Matrix<double, 6, 1> plausible_vel;       // temporary way to initialize velocity state variable
    plausible_vel << -0.9, 0.0, 0.0, 0.0, 0.0, 0.0;

    // setup state variables using initial condition
    std::vector<SteamTrajVar> traj_states_ic;
    std::vector<TransformStateVar::Ptr> statevars;
    // todo: eventually want to for loop over window
    {
      TransformStateVar::Ptr temp_statevar_a(new TransformStateVar());
      TransformStateEvaluator::Ptr temp_pose_a = TransformStateEvaluator::MakeShared(temp_statevar_a);
      VectorSpaceStateVar::Ptr temp_velocity_a = VectorSpaceStateVar::Ptr(new VectorSpaceStateVar(plausible_vel));
      temp_statevar_a->setLock(true);   // lock the first pose (but not the first velocity)
      SteamTrajVar temp_a(steam::Time((int64_t)msg->t_a), temp_pose_a, temp_velocity_a);
      statevars.push_back(temp_statevar_a);
      traj_states_ic.push_back(temp_a);

      TransformStateVar::Ptr temp_statevar_b(new TransformStateVar());
      TransformStateEvaluator::Ptr temp_pose_b = TransformStateEvaluator::MakeShared(temp_statevar_b);
      VectorSpaceStateVar::Ptr temp_velocity_b = VectorSpaceStateVar::Ptr(new VectorSpaceStateVar(plausible_vel));
      SteamTrajVar temp_b(steam::Time((int64_t)msg->t_b), temp_pose_b, temp_velocity_b);
      statevars.push_back(temp_statevar_b);
      traj_states_ic.push_back(temp_b);
    }
    PositionEvaluator::ConstPtr r_ba_ina
        (new PositionEvaluator(TransformStateEvaluator::MakeShared(statevars.back())));   // todo: will eventually want several of these

    // todo: set up C_ag state and RotationEvaluator
    RotationStateVar::Ptr C_ag_statevar(new RotationStateVar());
    RotationEvaluator::ConstPtr C_ag(new RotationEvaluator(RotationStateEvaluator::MakeShared(C_ag_statevar)));  // (?) todo



    // using constant covariance here for now
    steam::BaseNoiseModel<1>::Ptr tdcp_noise_model(new steam::StaticNoiseModel<1>(tdcp_cov_));

    // iterate through satellite pairs in msg and add TDCP costs
    for (const auto &pair : msg->pairs) {
      Eigen::Vector3d r_1a_ing_ata{pair.r_1a_a.x, pair.r_1a_a.y, pair.r_1a_a.z};
      Eigen::Vector3d r_1a_ing_atb{pair.r_1a_b.x, pair.r_1a_b.y, pair.r_1a_b.z};
      Eigen::Vector3d r_2a_ing_ata{pair.r_2a_a.x, pair.r_2a_a.y, pair.r_2a_a.z};
      Eigen::Vector3d r_2a_ing_atb{pair.r_2a_b.x, pair.r_2a_b.y, pair.r_2a_b.z};

      steam::TdcpErrorEval::Ptr tdcp_error(new steam::TdcpErrorEval(pair.phi_measured,
                                                                    r_ba_ina,
                                                                    C_ag,
                                                                    r_1a_ing_ata,
                                                                    r_1a_ing_atb,
                                                                    r_2a_ing_ata,
                                                                    r_2a_ing_atb));
      auto tdcp_factor = steam::WeightedLeastSqCostTerm<1, 6>::Ptr(new steam::WeightedLeastSqCostTerm<1, 6>(
          tdcp_error,
          tdcp_noise_model,
          tdcp_loss_function_));
      tdcp_cost_terms_->add(tdcp_factor);
    }

    // todo: nonholonomic cost(s)


    problem_->addCostTerm(tdcp_cost_terms_);
    problem_->addCostTerm(nonholonomic_cost_terms_);
    problem_->addCostTerm(smoothing_cost_terms_);

    // setup solver and optimize
    steam::DoglegGaussNewtonSolver::Params params;
    params.verbose = true;      // todo: make configurable
    params.maxIterations = 5;
    solver_.reset(new steam::DoglegGaussNewtonSolver(problem_.get(), params));
    solver_->optimize();

    // get optimized transform and publish
    lgmath::se3::Transformation pose = statevars.back()->getValue();
    auto gn_solver = std::dynamic_pointer_cast<steam::GaussNewtonSolverBase>(solver_);
    Eigen::Matrix<double, 6, 6> pose_covariance = gn_solver->queryCovariance(statevars.back()->getKey());
//    geometry_msgs::msg::PoseWithCovariance pose_msg = toPoseMsg(pose, pose_covariance);
//    publisher_->publish(pose_msg);
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

void CpoBackEnd::resetProblem() {
  // setup loss functions
  tdcp_loss_function_.reset(new steam::DcsLossFunc(2.0));
  nonholonomic_loss_function_.reset(new steam::L2LossFunc());

  // setup cost terms
  tdcp_cost_terms_.reset(new steam::ParallelizedCostTermCollection());
  nonholonomic_cost_terms_.reset(new steam::ParallelizedCostTermCollection());
  smoothing_cost_terms_.reset(new steam::ParallelizedCostTermCollection());

  // set up the steam problem
  problem_.reset(new steam::OptimizationProblem());
}
