#include <TdcpErrorEval.hpp>
#include <UnicycleErrorEval.hpp>

#include <cpo_utilities.hpp>
#include <CpoBackEnd.hpp>
#include <fstream>
#include <chrono>
#include <filesystem>
#include <Eigen/Core>

using Transformation = lgmath::se3::Transformation;
using TransformationWithCovariance = lgmath::se3::TransformationWithCovariance;
using TransformStateVar = steam::se3::TransformStateVar;
using TransformStateEvaluator = steam::se3::TransformStateEvaluator;
using TransformEvaluator = steam::se3::TransformEvaluator;
using SteamTrajVar = steam::se3::SteamTrajVar;
using SteamTrajInterface = steam::se3::SteamTrajInterface;
using VectorSpaceStateVar = steam::VectorSpaceStateVar;
using PositionEvaluator = steam::se3::PositionEvaluator;

CpoBackEnd::CpoBackEnd() : Node("cpo_back_end") {

  subscription_ = this->create_subscription<cpo_interfaces::msg::TDCP>("tdcp",
                                                                       10,
                                                                       std::bind(
                                                                           &CpoBackEnd::_tdcpCallback,
                                                                           this,
                                                                           std::placeholders::_1));
  enu_publisher_ =
      this->create_publisher<geometry_msgs::msg::PoseWithCovariance>("cpo_enu",
                                                                     10);

  this->declare_parameter("fixed_rate_publish", false);
  fixed_rate_publish_ = this->get_parameter("fixed_rate_publish").as_bool();

  if (fixed_rate_publish_) {
    this->declare_parameter("publish_frequency", 5.0);
    double freq = this->get_parameter("publish_frequency").as_double();
    double period = 1000 / freq;

    this->declare_parameter("publish_delay", 5.0);
    publish_delay_ = this->get_parameter("publish_delay").as_double();

    publish_timer_ = rclcpp::create_timer(this,
                                          this->get_clock(),
                                          std::chrono::milliseconds((long) period),
                                          std::bind(&CpoBackEnd::_timedCallback,
                                                    this));
  } else {
    publish_timer_ = nullptr;
  }

  query_traj_srv_ = this->create_service<QueryTrajectory>("query_trajectory",
                                                          std::bind(&CpoBackEnd::_queryCallback,
                                                                    this,
                                                                    std::placeholders::_1,
                                                                    std::placeholders::_2));

  // set up receiver-vehicle transform
  this->declare_parameter("r_veh_gps_inv",
                          std::vector<double>{-0.60, 0.00, -0.52});
  auto r = this->get_parameter("r_veh_gps_inv").as_double_array();

  Eigen::Matrix4d T_gps_vehicle_eigen = Eigen::Matrix4d::Identity();
  T_gps_vehicle_eigen(0, 3) = r[0];
  T_gps_vehicle_eigen(1, 3) = r[1];
  T_gps_vehicle_eigen(2, 3) = r[2];
  auto T_gps_vehicle = TransformationWithCovariance(T_gps_vehicle_eigen);
  T_gps_vehicle.setZeroCovariance();
  tf_gps_vehicle_ =
      steam::se3::FixedTransformEvaluator::MakeShared(T_gps_vehicle);

  getParams();
}

void CpoBackEnd::_tdcpCallback(const cpo_interfaces::msg::TDCP::SharedPtr msg_in) {

  uint n = msg_in->pairs.size();
  std::cout << "Received " << n << " satellite pairs." << std::endl;

  addMsgToWindow(msg_in);

  // don't attempt optimization if we don't have a full window
  if (edges_.size() < window_size_)
    return;

  if (n >= 4) {
    initializeProblem();

    // set up steam problem

    // setup state variables using initial condition
    std::vector<SteamTrajVar> traj_states;
    std::vector<TransformStateVar::Ptr> statevars;

    // keep track of states composed with frame transform for use in TDCP terms
    std::vector<TransformEvaluator::ConstPtr> receiver_poses;
    std::vector<TransformEvaluator::ConstPtr> enu_poses;

    // set up T_0g state
    TransformStateVar::Ptr T_0g_statevar(new TransformStateVar(init_pose_));
    TransformEvaluator::ConstPtr
        T_0g = TransformStateEvaluator::MakeShared(T_0g_statevar);

    { // first pose in window gets locked (but not the first velocity)
      TransformStateVar::Ptr
          temp_statevar_a(new TransformStateVar(Transformation()));
      TransformStateEvaluator::Ptr
          temp_pose_a = TransformStateEvaluator::MakeShared(temp_statevar_a);
      VectorSpaceStateVar::Ptr temp_velocity_a =
          VectorSpaceStateVar::Ptr(new VectorSpaceStateVar(edges_.front().v_a));
      temp_statevar_a->setLock(true);
      SteamTrajVar temp_a(steam::Time((int64_t)
      edges_.front().msg.t_a), temp_pose_a, temp_velocity_a);
      statevars.push_back(temp_statevar_a);
      traj_states.push_back(temp_a);

      TransformEvaluator::Ptr
          rec_pose_a = steam::se3::compose(tf_gps_vehicle_, temp_pose_a);
      receiver_poses.emplace_back(rec_pose_a);
      enu_poses.emplace_back(steam::se3::compose(rec_pose_a, T_0g));
    }

    Transformation prev_T_k0_est = Transformation();
    // loop over window to add states for other poses
    for (auto &edge : edges_) {
      // k has been incremented so update T_k0
      Transformation T_k0_est(edge.T_ba * prev_T_k0_est);
      // prevents rotation matrix from drifting away from being orthogonal
      T_k0_est.reproject(true);

      TransformStateVar::Ptr temp_statevar(new TransformStateVar(T_k0_est));
      TransformStateEvaluator::Ptr
          temp_pose = TransformStateEvaluator::MakeShared(temp_statevar);
      VectorSpaceStateVar::Ptr temp_velocity =
          VectorSpaceStateVar::Ptr(new VectorSpaceStateVar(edge.v_b));
      SteamTrajVar temp(steam::Time((int64_t)
      edge.msg.t_b), temp_pose, temp_velocity);
      statevars.push_back(temp_statevar);
      traj_states.push_back(temp);

      // T_s0 = T_sv * T_v0
      TransformEvaluator::Ptr
          rec_pose = steam::se3::compose(tf_gps_vehicle_, temp_pose);
      receiver_poses.emplace_back(rec_pose); // T_k0
      // T_kg = T_k0 * T_0g
      enu_poses.emplace_back(steam::se3::compose(rec_pose, T_0g));

      prev_T_k0_est = T_k0_est;
    }

    // add TDCP terms
    for (uint k = 0; k < edges_.size(); ++k) {
      TransformEvaluator::ConstPtr T_k1k = steam::se3::compose(
          receiver_poses[k + 1],
          steam::se3::inverse(receiver_poses[k]));
      PositionEvaluator::ConstPtr r_ba_ina(new PositionEvaluator(T_k1k));

      // using constant covariance here for now
      steam::BaseNoiseModel<1>::Ptr
          tdcp_noise_model(new steam::StaticNoiseModel<1>(tdcp_cov_));

      // iterate through satellite pairs in msg and add TDCP costs
      for (const auto &pair : edges_[k].msg.pairs) {
        Eigen::Vector3d
            r_1a_ing_ata{pair.r_1a_a.x, pair.r_1a_a.y, pair.r_1a_a.z};
        Eigen::Vector3d
            r_1a_ing_atb{pair.r_1a_b.x, pair.r_1a_b.y, pair.r_1a_b.z};
        Eigen::Vector3d
            r_2a_ing_ata{pair.r_2a_a.x, pair.r_2a_a.y, pair.r_2a_a.z};
        Eigen::Vector3d
            r_2a_ing_atb{pair.r_2a_b.x, pair.r_2a_b.y, pair.r_2a_b.z};

        steam::TdcpErrorEval::Ptr tdcp_error(new steam::TdcpErrorEval(
            pair.phi_measured,
            r_ba_ina,
            enu_poses[k],   // T_kg
            r_1a_ing_ata,
            r_1a_ing_atb,
            r_2a_ing_ata,
            r_2a_ing_atb));
        auto tdcp_factor = steam::WeightedLeastSqCostTerm<1, 6>::Ptr(
            new steam::WeightedLeastSqCostTerm<1, 6>(
                tdcp_error,
                tdcp_noise_model,
                tdcp_loss_function_));
        tdcp_cost_terms_->add(tdcp_factor);
      }
    }

    // lock T_0g instead when we already have a good estimate of it
    if (init_pose_estimated_ && lock_first_pose_) {
      T_0g_statevar->setLock(true);
    } else {
      // add prior on initial pose to deal with roll uncertainty and constrain r^0g_g to zero
      steam::BaseNoiseModel<6>::Ptr
          sharedNoiseModel(new steam::StaticNoiseModel<6>(pose_prior_cov_));
      steam::TransformErrorEval::Ptr prior_error_func(
          new steam::TransformErrorEval(init_pose_, T_0g));
      auto pose_prior_factor_ = steam::WeightedLeastSqCostTerm<6, 6>::Ptr(
          new steam::WeightedLeastSqCostTerm<6, 6>(
              prior_error_func,
              sharedNoiseModel,
              pp_loss_function_));
      pose_prior_cost_->add(pose_prior_factor_);
    }

    steam::BaseNoiseModel<4>::Ptr nonholonomic_noise_model
        (new steam::StaticNoiseModel<4>(nonholonomic_cov_));
    trajectory_.reset(new SteamTrajInterface(smoothing_factor_information_,
                                             true));

    // loop through velocity state variables
    for (const auto &traj_state : traj_states) {
      if (!traj_state.getVelocity()->isLocked()) {
        // add nonholonomic costs
        steam::UnicycleErrorEval::Ptr non_holo_error_func(
            new steam::UnicycleErrorEval(traj_state.getVelocity()));
        auto non_holonomic_factor = steam::WeightedLeastSqCostTerm<4, 6>::Ptr(
            new steam::WeightedLeastSqCostTerm<4, 6>(
                non_holo_error_func,
                nonholonomic_noise_model,
                nonholonomic_loss_function_));
        nonholonomic_cost_terms_->add(non_holonomic_factor);

        // add smoothing costs
        steam::Time temp_time = traj_state.getTime();
        const TransformEvaluator::Ptr &temp_pose = traj_state.getPose();
        const steam::VectorSpaceStateVar::Ptr
            &temp_velocity = traj_state.getVelocity();
        trajectory_->add(temp_time, temp_pose, temp_velocity);

        // also add velocity state to problem
        problem_->addStateVariable(traj_state.getVelocity());
      }
    }
    trajectory_->appendPriorCostTerms(smoothing_cost_terms_);

    problem_->addCostTerm(tdcp_cost_terms_);
    problem_->addCostTerm(nonholonomic_cost_terms_);
    problem_->addCostTerm(smoothing_cost_terms_);
    problem_->addCostTerm(pose_prior_cost_);

    problem_->addStateVariable(T_0g_statevar);
    for (const auto &state : statevars) {
      problem_->addStateVariable(state);
    }

    // print initial costs (for debugging/development)
    printCosts(false);

    // setup solver and optimize
    steam::DoglegGaussNewtonSolver::Params params;
    params.verbose = steam_verbose_;
    params.maxIterations = steam_max_iterations_;
    params.absoluteCostChangeThreshold = 1e-2;
    solver_.reset(new steam::DoglegGaussNewtonSolver(problem_.get(), params));

    try {
      solver_->optimize();
    } catch (steam::unsuccessful_step &e) {
      // did any successful steps occur?
      if (solver_->getCurrIteration() <= 1) {
        // no: something is very wrong; we should start over. Should not occur frequently
        std::cout
            << "Steam has failed to optimize the problem! This is an ERROR."
            << std::endl;
        return;
      } else {
        // yes: just a marginal problem, let's use what we got
        std::cout
            << "Steam has failed due to an unsuccessful step. This should be OK if it happens infrequently."
            << std::endl;
      }
    } catch (steam::decomp_failure &e) {
      // Should not occur frequently
      std::cout
          << "Steam has encountered an LL^T decomposition error while optimizing! This is an ERROR."
          << std::endl;
      return;
    }

    // print final costs (for debugging/development)
    printCosts(true);

    // update with optimized transforms
    for (uint i = 0; i < edges_.size(); ++i) {
      edges_[i].T_ba = statevars[i + 1]->getValue()
          * statevars[i]->getValue().inverse();  // T_21 = T_20 * inv(T_10)
      edges_[i].T_ba.reproject(true);
      edges_[i].v_a = traj_states.at(i).getVelocity()->getValue();
      edges_[i].v_b = traj_states.at(i + 1).getVelocity()->getValue();
    }

    // update our orientation estimate
    if (!T_0g_statevar->isLocked()) {
      init_pose_ = T_0g_statevar->getValue();
    }

    init_pose_.reproject(true);

    if (!fixed_rate_publish_) {
      double t_n = (double) edges_.back().msg.t_b * 1e-9;
      double t_0 = (double) edges_.front().msg.t_a * 1e-9;
      Transformation T_ng = statevars.back()->getValue() * init_pose_;
      publishPose(init_pose_);
      saveToFile(init_pose_, t_0);

      std::cout << "Last time was: " << std::setprecision(12) << t_n;
      std::cout << "    Time zero was: " << t_0 << std::setprecision(6)
                << std::endl;
    }

    init_pose_estimated_ = true;
    first_window_ = false;
  }
}

void CpoBackEnd::_timedCallback() {
  if (first_window_ || !init_pose_estimated_ || trajectory_ == nullptr) {
    std::cout
        << "Not publishing because don't currently have a valid pose estimate."
        << std::endl;
    return;
  }

  // grab times and extrapolate poses
  double t_last_msg = (double) edges_.back().msg.t_b * 1e-9;
  double t_n = get_clock()->now().seconds();      // current time
  double t_k = t_n - publish_delay_;              // time to save an estimate at

  if (t_n - t_last_msg > traj_timeout_limit_) {
    std::cout << "Latest carrier phase measurement is " << t_n - t_last_msg
              << " seconds old. "
              << "Will not publish because trajectory is no longer valid."
              << std::endl;
    return;
  }
  if (t_k > t_last_msg && publish_delay_ > 0) {
    // if here we have set non-zero value to delay publishing so we don't have
    // to extrapolate but there's a big enough observation gap we would have to
    std::cout
        << "Gap in TDCP observations. Choosing not to extrapolate trajectory. "
        << std::endl;
    return;
  }

  TransformationWithCovariance T_0g = init_pose_;
  T_0g.setZeroCovariance();
  TransformationWithCovariance
      T_k0 = trajectory_->getInterpPoseEval(t_k)->evaluate();
  T_k0.setZeroCovariance();
  TransformationWithCovariance T_kg = T_k0 * T_0g;

  // publish
  publishPose(T_kg);
  saveToFile(T_kg, t_k);
}

void CpoBackEnd::_queryCallback(const std::shared_ptr<QueryTrajectory::Request> request,
                                std::shared_ptr<QueryTrajectory::Response> response) {
  if (first_window_ || !init_pose_estimated_ || trajectory_ == nullptr
      || solver_ == nullptr) {
    response->success = false;
    response->message = "CPO does not currently have a valid pose estimate.";
    return;
  }

  // get times of current window
  uint64_t t_start = edges_.front().msg.t_a;
  uint64_t t_end = edges_.back().msg.t_b;

  std::cout << std::setprecision(19) << "QC win t_a " << t_start
            << std::endl;    // DEBUGGING
  std::cout << "QC win t_b " << t_end << std::endl;    // DEBUGGING
  std::cout << "QC req t_1 " << request->t_1 << std::endl;    // DEBUGGING
  std::cout << "QC req t_2 " << request->t_2 << std::setprecision(6)
            << std::endl;    // DEBUGGING

  // error checking
  if (request->t_1 > request->t_2) {
    response->success = false;
    response->message = "t_1 > t_2 but expected t_2 to be after t_1.";

    std::cout << "response->message: " << response->message
              << std::endl;  // debug
    return;
  } else if (request->t_1 < t_start) {
    response->success = false;
    response->message =
        "Requested time before CPO window. Trajectory history has not been implemented.";

    std::cout << "response->message: " << response->message
              << std::endl;  // debug
    return;
  } else if (request->t_2 > t_end + (long) (traj_timeout_limit_ * 1e9)) {
    response->success = false;
    response->message = "t_2 too long after last GPS msg to trust trajectory.";

    std::cout << "response->message: " << response->message
              << std::endl;  // debug
    return;
  }

  // requested times are reasonable so let's try to estimate a transform
  lgmath::se3::TransformationWithCovariance T_10 =
      trajectory_->getInterpPoseEval(steam::Time((int64_t) request->t_1))->evaluate();
  lgmath::se3::TransformationWithCovariance T_20 =
      trajectory_->getInterpPoseEval(steam::Time((int64_t) request->t_2))->evaluate();
  lgmath::se3::TransformationWithCovariance T_21 = T_20 / T_10;

  auto gn_solver =
      std::dynamic_pointer_cast<steam::GaussNewtonSolverBase>(solver_);
  trajectory_->setSolver(gn_solver);

  Eigen::Matrix<double, 6, 6> Cov_21 = trajectory_->getRelativePoseCovariance(steam::Time((int64_t) request->t_1), steam::Time((int64_t) request->t_2));
  T_21.setCovariance(Cov_21);
  response->tf_2_1 = toPoseMsg(T_21);

  response->success = true;
  response->message = "Warning: query_trajectory not fully tested! Don't trust covariance";
}

void CpoBackEnd::publishPose(const TransformationWithCovariance &T_0g) {
  geometry_msgs::msg::PoseWithCovariance enu_pose_msg = toPoseMsg(T_0g);
  enu_publisher_->publish(enu_pose_msg);
  std::cout << "Message published! " << std::endl;
}

void CpoBackEnd::saveToFile(const Transformation &T_kg, double t_k) const {
  // append latest estimate to file
  std::ofstream outstream;
  outstream.open(results_path_, std::ofstream::out | std::ofstream::app);

  const Eigen::Vector3d r_kg_g = T_kg.r_ba_ina();     // vehicle position
  Eigen::Vector3d r_sg_g = (tf_gps_vehicle_->evaluate() * T_kg).r_ba_ina();
  Eigen::Matrix4d T_sg = (tf_gps_vehicle_->evaluate() * T_kg).matrix();

  // save times and global position for easy plotting
  outstream << std::setprecision(12) << t_k << ", " << 0 << ", ";
  outstream << r_sg_g(0) << ", " << r_sg_g(1) << ", " << r_sg_g(2)
            << ", ";   // receiver position in ENU frame
  outstream << r_kg_g(0) << ", " << r_kg_g(1) << ", " << r_kg_g(2)
            << ", ";   // vehicle position in ENU frame

  // save full transformations of receiver frame as well (** column major)
  auto T_sg_flat = std::vector<double>(T_sg.data(), T_sg.data() + 16);
  for (auto entry : T_sg_flat) outstream << entry << ",";

  outstream << std::endl;
  outstream.close();
}

void CpoBackEnd::getParams() {
  this->declare_parameter("tdcp_cov", 0.1);
  this->declare_parameter("non_holo_y", 0.1);
  this->declare_parameter("non_holo_z", 0.1);
  this->declare_parameter("non_holo_roll", 0.1);
  this->declare_parameter("non_holo_pitch", 0.1);
  this->declare_parameter("lin_acc_std_dev_x", 1.0);
  this->declare_parameter("lin_acc_std_dev_y", 0.1);
  this->declare_parameter("lin_acc_std_dev_z", 0.1);
  this->declare_parameter("ang_acc_std_dev_x", 0.1);
  this->declare_parameter("ang_acc_std_dev_y", 0.1);
  this->declare_parameter("ang_acc_std_dev_z", 0.1);
  this->declare_parameter("roll_cov_x", 0.001);
  this->declare_parameter("roll_cov_y", 0.001);
  this->declare_parameter("roll_cov_z", 0.001);
  this->declare_parameter("roll_cov_ang1", 0.01);
  this->declare_parameter("roll_cov_ang2", 0.01);
  this->declare_parameter("roll_cov_ang3", 1.0);
  this->declare_parameter("window_size", 10);
  this->declare_parameter("lock_first_pose", true);
  this->declare_parameter("results_path", "~/cpo.csv");
  this->declare_parameter("solver_verbose", false);
  this->declare_parameter("solver_max_iterations", 5);
  this->declare_parameter("traj_timeout_limit", 5.0);

  tdcp_cov_ << this->get_parameter("tdcp_cov").as_double();

  Eigen::Array<double, 1, 4> non_holo_diag;
  non_holo_diag << this->get_parameter("non_holo_y").as_double(),
      this->get_parameter("non_holo_z").as_double(),
      this->get_parameter("non_holo_roll").as_double(),
      this->get_parameter("non_holo_pitch").as_double();
  nonholonomic_cov_.setZero();
  nonholonomic_cov_.diagonal() = non_holo_diag;

  Eigen::Array<double, 1, 6> Qc_diag;
  Qc_diag << this->get_parameter("lin_acc_std_dev_x").as_double(),
      this->get_parameter("lin_acc_std_dev_y").as_double(),
      this->get_parameter("lin_acc_std_dev_z").as_double(),
      this->get_parameter("ang_acc_std_dev_x").as_double(),
      this->get_parameter("ang_acc_std_dev_y").as_double(),
      this->get_parameter("ang_acc_std_dev_z").as_double();
  smoothing_factor_information_.setZero();
  smoothing_factor_information_.diagonal() = 1.0 / Qc_diag;

  pose_prior_cov_ = Eigen::Matrix<double, 6, 6>::Identity();
  pose_prior_cov_(0, 0) = this->get_parameter("roll_cov_x").as_double();
  pose_prior_cov_(1, 1) = this->get_parameter("roll_cov_y").as_double();
  pose_prior_cov_(2, 2) = this->get_parameter("roll_cov_z").as_double();
  pose_prior_cov_(3, 3) = this->get_parameter("roll_cov_ang1").as_double();
  pose_prior_cov_(4, 4) = this->get_parameter("roll_cov_ang2").as_double();
  pose_prior_cov_(5, 5) = this->get_parameter("roll_cov_ang3").as_double();

  window_size_ = this->get_parameter("window_size").as_int();
  lock_first_pose_ = this->get_parameter("lock_first_pose").as_bool();
  results_path_ = expandUser(this->get_parameter("results_path").as_string());
  steam_verbose_ = this->get_parameter("solver_verbose").as_bool();
  steam_max_iterations_ = this->get_parameter("solver_max_iterations").as_int();
  traj_timeout_limit_ = this->get_parameter("traj_timeout_limit").as_double();
}

void CpoBackEnd::initializeProblem() {
  // setup loss functions
  tdcp_loss_function_.reset(new steam::CauchyLossFunc(2.0));
  nonholonomic_loss_function_.reset(new steam::L2LossFunc());
  pp_loss_function_.reset(new steam::L2LossFunc());

  // setup cost terms
  tdcp_cost_terms_.reset(new steam::ParallelizedCostTermCollection());
  nonholonomic_cost_terms_.reset(new steam::ParallelizedCostTermCollection());
  smoothing_cost_terms_.reset(new steam::ParallelizedCostTermCollection());
  pose_prior_cost_.reset(new steam::ParallelizedCostTermCollection());

  // set up the steam problem
  problem_.reset(new steam::OptimizationProblem());

  if (!init_pose_estimated_) {
    // estimate initial T_0g from code solutions
    Eigen::Vector3d r_k0_ing = toEigenVec3d(edges_.back().msg.enu_pos)
        - toEigenVec3d(edges_.front().msg.enu_pos);
    double theta = atan2(r_k0_ing.y(), r_k0_ing.x());

    Eigen::Matrix<double, 6, 1> init_pose_vec;
    init_pose_vec << toEigenVec3d(edges_.front().msg.enu_pos), 0, 0, -1 * theta;

    init_pose_ = Transformation(init_pose_vec);

    if (first_window_) {
      // save ENU origin to results file
      std::ofstream outstream;
      outstream.open(results_path_);
      Eigen::Vector3d enu_origin = toEigenVec3d(edges_.back().msg.enu_origin);
      outstream << std::setprecision(9) << enu_origin[0] << "," << enu_origin[1]
                << "," << enu_origin[2] << std::endl;
      outstream.close();
      if (!std::filesystem::exists(results_path_)) {
        std::cout << "Warning: Results path " << results_path_
                  << " does not exist." << std::endl;
      }
    } else {
      std::cout << "Warning: new initial pose was set midway through run."
                << "Estimates on either side of this time should not be compared."
                << std::endl;
    }
  }
}

void CpoBackEnd::resetEstimator() {
  std::cout << "Clearing window and resetting the estimator." << std::endl;
  edges_.clear();
  trajectory_ = nullptr;
  init_pose_estimated_ = false;
  std::cout << "WARNING: RESET ESTIMATOR !" << std::endl;
}

void CpoBackEnd::printCosts(bool final) {
  std::string stage_str = final ? "Final" : "Initial";
  std::cout << " === " << stage_str << " Costs === " << std::endl;
  std::cout << "Carrier Phase:       " << tdcp_cost_terms_->cost()
            << "        Terms:  " << tdcp_cost_terms_->numCostTerms()
            << std::endl;
  std::cout << "Nonholonomic:        " << nonholonomic_cost_terms_->cost()
            << "        Terms:  " << nonholonomic_cost_terms_->numCostTerms()
            << std::endl;
  std::cout << "Smoothing:           " << smoothing_cost_terms_->cost()
            << "        Terms:  " << smoothing_cost_terms_->numCostTerms()
            << std::endl;
  std::cout << "Pose Prior:          " << pose_prior_cost_->cost()
            << "        Terms:  " << pose_prior_cost_->numCostTerms()
            << std::endl;
}

void CpoBackEnd::addMsgToWindow(const cpo_interfaces::msg::TDCP::SharedPtr &msg) {

  if (!edges_.empty() && msg->t_a != edges_.back().msg.t_b) {
    // times don't align, so we've likely missed a msg. To be safe we'll clear
    std::cout << "Warning: mismatched times. Current t_a: " << msg->t_a
              << ". Previous t_b: " << edges_.back().msg.t_b
              << " Resetting estimator." << std::endl;
    resetEstimator();
  }

  // add latest message, setting initial values for states
  Transformation new_T_estimate;
  Eigen::Matrix<double, 6, 1> new_vel_a;
  Eigen::Matrix<double, 6, 1> new_vel_b;
  if (trajectory_ != nullptr) {
    Transformation T_b0 =
        trajectory_->getInterpPoseEval(steam::Time((int64_t) msg->t_b))->evaluate();
    Transformation T_a0 =
        trajectory_->getInterpPoseEval(steam::Time((int64_t) msg->t_a))->evaluate();
    new_T_estimate = T_b0 * T_a0.inverse();
    new_vel_a = trajectory_->getVelocity(steam::Time((int64_t) msg->t_a));
    new_vel_b = trajectory_->getVelocity(steam::Time((int64_t) msg->t_b));
  } else {
    // if we don't have a trajectory to extrapolate, try generic initial guesses
    double dist_since_last = edges_.empty() ? 0.0 : 1.0;
    Eigen::Vector3d r_ba_est{dist_since_last, 0.0, 0.0};
    new_T_estimate = Transformation(Eigen::Matrix3d::Identity(), r_ba_est);
    new_vel_a << -1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    new_vel_b << -1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  }

  edges_.emplace_back(CpoEdge{*msg, new_T_estimate, new_vel_a, new_vel_b});

  // if we have a full queue, discard the oldest msg
  while (edges_.size() > window_size_) {
    // incrementing indices so need to update our T_0g estimate
    init_pose_ = edges_.front().T_ba * init_pose_;
    init_pose_.reproject(true);

    edges_.pop_front();
  }
}

geometry_msgs::msg::PoseWithCovariance CpoBackEnd::toPoseMsg(
    TransformationWithCovariance T) {
  Eigen::Quaterniond q(T.C_ba());
  Eigen::Vector3d r_ab_inb = T.r_ab_inb();

  geometry_msgs::msg::PoseWithCovariance msg;
  msg.pose.position.set__x(r_ab_inb[0]);
  msg.pose.position.set__y(r_ab_inb[1]);
  msg.pose.position.set__z(r_ab_inb[2]);
  msg.pose.orientation.set__x(q.x());
  msg.pose.orientation.set__y(q.y());
  msg.pose.orientation.set__z(q.z());
  msg.pose.orientation.set__w(q.w());

  if (!T.covarianceSet())
    T.setZeroCovariance();

  Eigen::Matrix<double, 6, 6> T_cov = T.cov();
  std::array<double, 36> temp{};
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      temp[6*i + j] = T_cov(i, j);
    }
  }
  msg.set__covariance(temp);

  return msg;
}

Eigen::Vector3d CpoBackEnd::toEigenVec3d(const geometry_msgs::msg::Vector3 &ros_vec) {
  return Eigen::Vector3d{ros_vec.x, ros_vec.y, ros_vec.z};
}
