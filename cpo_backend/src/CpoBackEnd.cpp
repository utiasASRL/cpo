#include <TdcpErrorEval.hpp>
#include <RotationStateEvaluator.hpp>
#include <UnicycleErrorEval.hpp>
#include <RollErrorEval.hpp>

#include <cpo_backend/CpoBackEnd.hpp>

using TransformStateVar = steam::se3::TransformStateVar;
using TransformStateEvaluator = steam::se3::TransformStateEvaluator;
using SteamTrajVar = steam::se3::SteamTrajVar;
using VectorSpaceStateVar = steam::VectorSpaceStateVar;
using PositionEvaluator = steam::se3::PositionEvaluator;
using RotationEvaluator = steam::so3::RotationEvaluator;
using RotationStateEvaluator = steam::so3::RotationStateEvaluator;

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

  /// TdcpErrorEval stuff
  // todo: will have to save(?) set of ErrorEvals/costs/the problem as member of CpoBackEnd. Still need to figure out

  uint n = msg->pairs.size();
  std::cout << "Found " << n << " sat pairs." << std::endl;

  addMsgToWindow(msg);

  // don't attempt optimization if we don't have a full window
  if (msgs_.size() < window_size_)
    return;

  if (n >= 4) {
    resetProblem();

    /// set up steam problem
    // for now just using one pair of poses

    Eigen::Matrix<double, 6, 1> plausible_vel;       // temporary way to initialize velocity state variable
    plausible_vel << -0.9, 0.0, 0.0, 0.0, 0.0, 0.0;

    Eigen::Matrix<double, 6, 1> plaus_Tba_vec;       // temporary way to initialize pose state variable
    plaus_Tba_vec << -1.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    // setup state variables using initial condition
    std::vector<SteamTrajVar> traj_states;
    std::vector<TransformStateVar::Ptr> statevars;
    // todo: eventually want to for loop over window
    {
      TransformStateVar::Ptr temp_statevar_a(new TransformStateVar(lgmath::se3::Transformation()));
      TransformStateEvaluator::Ptr temp_pose_a = TransformStateEvaluator::MakeShared(temp_statevar_a);
      VectorSpaceStateVar::Ptr temp_velocity_a = VectorSpaceStateVar::Ptr(new VectorSpaceStateVar(plausible_vel));
      temp_statevar_a->setLock(true);   // lock the first pose (but not the first velocity)
      SteamTrajVar temp_a(steam::Time((int64_t)msg->t_a), temp_pose_a, temp_velocity_a);
      statevars.push_back(temp_statevar_a);
      traj_states.push_back(temp_a);

      TransformStateVar::Ptr temp_statevar_b(new TransformStateVar(lgmath::se3::Transformation(plaus_Tba_vec)));
      TransformStateEvaluator::Ptr temp_pose_b = TransformStateEvaluator::MakeShared(temp_statevar_b);
      VectorSpaceStateVar::Ptr temp_velocity_b = VectorSpaceStateVar::Ptr(new VectorSpaceStateVar(plausible_vel));
      SteamTrajVar temp_b(steam::Time((int64_t)msg->t_b), temp_pose_b, temp_velocity_b);
      statevars.push_back(temp_statevar_b);
      traj_states.push_back(temp_b);
    }
    PositionEvaluator::ConstPtr r_ba_ina
        (new PositionEvaluator(TransformStateEvaluator::MakeShared(statevars.back())));   // todo: will eventually want several of these

    // set up C_ag state and RotationEvaluator
    RotationStateVar::Ptr C_ag_statevar(new RotationStateVar(approx_rotation_));
    RotationEvaluator::ConstPtr C_ag = RotationStateEvaluator::MakeShared(C_ag_statevar);

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

    // add weak prior on initial pose to deal with roll uncertainty
    steam::BaseNoiseModel<1>::Ptr roll_noise_model(new steam::StaticNoiseModel<1>(roll_cov_));
    RotationStateEvaluator::Ptr tmp_rot = RotationStateEvaluator::MakeShared(C_ag_statevar);
    steam::RollErrorEval::Ptr roll_error_func(new steam::RollErrorEval(tmp_rot));
    roll_cost_term_.reset(new steam::WeightedLeastSqCostTerm<1, 6>(roll_error_func, roll_noise_model, roll_loss_function_));

    steam::BaseNoiseModel<4>::Ptr nonholonomic_noise_model(new steam::StaticNoiseModel<4>(nonholonomic_cov_));
    steam::se3::SteamTrajInterface traj(smoothing_factor_information_, true);

    // loop through velocity state variables
    for (const auto &traj_state : traj_states) {
      if (!traj_state.getVelocity()->isLocked()) {
        // add nonholonomic costs
        steam::UnicycleErrorEval::Ptr non_holo_error_func(new steam::UnicycleErrorEval(traj_state.getVelocity()));
        auto non_holonomic_factor = steam::WeightedLeastSqCostTerm<4, 6>::Ptr(new steam::WeightedLeastSqCostTerm<4, 6>(
            non_holo_error_func,
            nonholonomic_noise_model,
            nonholonomic_loss_function_));
        nonholonomic_cost_terms_->add(non_holonomic_factor);

        // add smoothing costs
        steam::Time temp_time = traj_state.getTime();
        const steam::se3::TransformEvaluator::Ptr &temp_pose = traj_state.getPose();
        const steam::VectorSpaceStateVar::Ptr &temp_velocity = traj_state.getVelocity();
        traj.add(temp_time, temp_pose, temp_velocity);

        // also add velocity state to problem
        problem_->addStateVariable(traj_state.getVelocity());
      }
    }
    traj.appendPriorCostTerms(smoothing_cost_terms_);

    problem_->addCostTerm(tdcp_cost_terms_);
    problem_->addCostTerm(nonholonomic_cost_terms_);
    problem_->addCostTerm(smoothing_cost_terms_);
    problem_->addCostTerm(roll_cost_term_);

    problem_->addStateVariable(C_ag_statevar);
    for (const auto &state : statevars) {
      problem_->addStateVariable(state);
    }

    // print initial costs (for debugging/development)
    printCosts();

    // setup solver and optimize
    steam::DoglegGaussNewtonSolver::Params params;
    params.verbose = true;      // todo: make configurable
    params.maxIterations = 5;
    solver_.reset(new steam::DoglegGaussNewtonSolver(problem_.get(), params));
    solver_->optimize();

    // print final costs (for debugging/development)
    printCosts();

    // get optimized transform and publish
    lgmath::se3::Transformation pose = statevars.back()->getValue();
    auto gn_solver = std::dynamic_pointer_cast<steam::GaussNewtonSolverBase>(solver_);
    Eigen::Matrix<double, 6, 6> pose_covariance = gn_solver->queryCovariance(statevars.back()->getKey());
//    geometry_msgs::msg::PoseWithCovariance pose_msg = toPoseMsg(pose, pose_covariance);
//    publisher_->publish(pose_msg);

    std::cout << "r_ba_ina: " << pose.r_ba_ina().transpose() << std::endl;
    std::cout << "T_ba vec: " << pose.vec().transpose() << std::endl;

    // update our orientation estimate
    approx_rotation_ = C_ag_statevar->getValue();

    std::cout << "approx_rotation_ vec: " << approx_rotation_.vec().transpose() << std::endl;

  }

}
void CpoBackEnd::getParams() {
  // todo: eventually want to setup as ROS2 params (this->declare_parameter<...) but for now will hard code

  double tdcp_cov = 0.01;
  double non_holo_y = 0.1;
  double non_holo_z = 0.1;
  double non_holo_roll = 0.1;
  double non_holo_pitch = 0.1;
  double lin_acc_std_dev_x = 1.0;
  double lin_acc_std_dev_y = 0.1;
  double lin_acc_std_dev_z = 0.1;
  double ang_acc_std_dev_x = 0.1;
  double ang_acc_std_dev_y = 0.1;
  double ang_acc_std_dev_z = 0.1;
  double roll_cov = 0.01;
  uint window_size = 10;

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

  roll_cov_ << roll_cov;

  window_size_ = window_size;
}

void CpoBackEnd::resetProblem() {
  // setup loss functions
  tdcp_loss_function_.reset(new steam::DcsLossFunc(2.0));   // todo: try different loss functions
  nonholonomic_loss_function_.reset(new steam::L2LossFunc());
  roll_loss_function_.reset(new steam::L2LossFunc());

  // setup cost terms
  tdcp_cost_terms_.reset(new steam::ParallelizedCostTermCollection());
  nonholonomic_cost_terms_.reset(new steam::ParallelizedCostTermCollection());
  smoothing_cost_terms_.reset(new steam::ParallelizedCostTermCollection());

  // set up the steam problem
  problem_.reset(new steam::OptimizationProblem());
}

void CpoBackEnd::printCosts() {
  std::cout << " === Costs === " << std::endl;
  std::cout << "Carrier Phase:       " << tdcp_cost_terms_->cost() << "        Terms:  "
            << tdcp_cost_terms_->numCostTerms() << std::endl;
  std::cout << "Nonholonomic:        " << nonholonomic_cost_terms_->cost() << "        Terms:  "
            << nonholonomic_cost_terms_->numCostTerms() << std::endl;
  std::cout << "Smoothing:           " << smoothing_cost_terms_->cost() << "        Terms:  "
            << smoothing_cost_terms_->numCostTerms() << std::endl;
  std::cout << "Roll Prior:          " << roll_cost_term_->cost() << "        Terms:  1" << std::endl;
}

void CpoBackEnd::addMsgToWindow(const cpo_interfaces::msg::TDCP::SharedPtr &msg) {

  if (!msgs_.empty() && msg->t_a != msgs_.back().t_b) {
    // times don't align, so we've likely missed a msg. To be safe we will clear it for now
    std::cout << "Warning: mismatched times. Clearing msgs_. Current t_a: " << msg->t_a << ". Previous t_b: "
              << msgs_.back().t_b << std::endl;
    std::queue<cpo_interfaces::msg::TDCP>().swap(msgs_);
  }

  // add latest message
  msgs_.push(*msg);

  // if we have a full queue, discard the oldest msg
  while (msgs_.size() > window_size_) {
    msgs_.pop();
  }
}
