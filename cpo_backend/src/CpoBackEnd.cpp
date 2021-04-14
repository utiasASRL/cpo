#include <TdcpErrorEval.hpp>
#include <UnicycleErrorEval.hpp>

#include <cpo_backend/CpoBackEnd.hpp>
#include <fstream>
#include <chrono>

using TransformStateVar = steam::se3::TransformStateVar;
using TransformStateEvaluator = steam::se3::TransformStateEvaluator;
using TransformEvaluator = steam::se3::TransformEvaluator;
using SteamTrajVar = steam::se3::SteamTrajVar;
using SteamTrajInterface = steam::se3::SteamTrajInterface;
using VectorSpaceStateVar = steam::VectorSpaceStateVar;
using PositionEvaluator = steam::se3::PositionEvaluator;

CpoBackEnd::CpoBackEnd() : Node("cpo_back_end") {

  subscription_ = this->create_subscription<cpo_interfaces::msg::TDCP>(
      "tdcp", 10, std::bind(&CpoBackEnd::_tdcpCallback, this, std::placeholders::_1));

  vehicle_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovariance>("cpo_odometry", 10);
  enu_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovariance>("cpo_enu", 10);

  this->declare_parameter("fixed_rate_publish", true);
  fixed_rate_publish_ = this->get_parameter("fixed_rate_publish").as_bool();

  if (fixed_rate_publish_) {
    this->declare_parameter("publish_frequency", 5.0);
    double freq = this->get_parameter("publish_frequency").as_double();
    double period = 1000 / freq;
    publish_timer_ =
        this->create_wall_timer(std::chrono::milliseconds((long) period), std::bind(&CpoBackEnd::_timedCallback, this));
  } else {
    publish_timer_ = nullptr;
  }

  // set up receiver-vehicle transform
  this->declare_parameter("r_veh_gps_inv", std::vector<double>{-0.60, 0.00, -0.52});
  auto r = this->get_parameter("r_veh_gps_inv").as_double_array();

  Eigen::Matrix4d T_gps_vehicle_eigen = Eigen::Matrix4d::Identity();
  T_gps_vehicle_eigen(0, 3) = r[0];
  T_gps_vehicle_eigen(1, 3) = r[1];
  T_gps_vehicle_eigen(2, 3) = r[2];
  auto T_gps_vehicle = lgmath::se3::TransformationWithCovariance(T_gps_vehicle_eigen);
  T_gps_vehicle.setZeroCovariance();
  tf_gps_vehicle_ = steam::se3::FixedTransformEvaluator::MakeShared(T_gps_vehicle);

  getParams();
}

void CpoBackEnd::_tdcpCallback(const cpo_interfaces::msg::TDCP::SharedPtr msg_in) {

  uint n = msg_in->pairs.size();
  std::cout << "Received " << n << " satellite pairs." << std::endl;

  addMsgToWindow(msg_in);

  // don't attempt optimization if we don't have a full window
  if (msgs_.size() < window_size_)
    return;

  if (n >= 4) {
    initializeProblem();

    // set up steam problem

    Eigen::Matrix<double, 6, 1> plausible_vel;       // temporary way to initialize velocity state variable
    plausible_vel << -0.9, 0.0, 0.0, 0.0, 0.0, 0.0;

    // setup state variables using initial condition
    std::vector<SteamTrajVar> traj_states;
    std::vector<TransformStateVar::Ptr> statevars;

    // keep track of states composed with frame transform for use in TDCP terms
    std::vector<TransformEvaluator::ConstPtr> receiver_poses;
    std::vector<TransformEvaluator::ConstPtr> enu_poses;

    // set up T_0g state
    TransformStateVar::Ptr T_0g_statevar(new TransformStateVar(init_pose_));
    TransformEvaluator::ConstPtr T_0g = TransformStateEvaluator::MakeShared(T_0g_statevar);

    { // first pose in window gets locked
      TransformStateVar::Ptr temp_statevar_a(new TransformStateVar(lgmath::se3::Transformation()));
      TransformStateEvaluator::Ptr temp_pose_a = TransformStateEvaluator::MakeShared(temp_statevar_a);
      VectorSpaceStateVar::Ptr temp_velocity_a = VectorSpaceStateVar::Ptr(new VectorSpaceStateVar(plausible_vel));
      temp_statevar_a->setLock(true);   // lock the first pose (but not the first velocity)
      SteamTrajVar temp_a(steam::Time((int64_t)msgs_.front().first.t_a), temp_pose_a, temp_velocity_a);
      statevars.push_back(temp_statevar_a);
      traj_states.push_back(temp_a);

      TransformEvaluator::Ptr rec_pose_a = steam::se3::compose(tf_gps_vehicle_, temp_pose_a);
      receiver_poses.emplace_back(rec_pose_a);
      enu_poses.emplace_back(steam::se3::compose(rec_pose_a, T_0g));
    }

    lgmath::se3::Transformation T_k0_est;
    // loop over window to add states for other poses
    for (auto &msg : msgs_) {
      T_k0_est = msg.second * T_k0_est;     // k has been incremented so update T_k0

      TransformStateVar::Ptr temp_statevar(new TransformStateVar(T_k0_est));
      TransformStateEvaluator::Ptr temp_pose = TransformStateEvaluator::MakeShared(temp_statevar);
      VectorSpaceStateVar::Ptr temp_velocity = VectorSpaceStateVar::Ptr(new VectorSpaceStateVar(plausible_vel));
      SteamTrajVar temp(steam::Time((int64_t)msg.first.t_b), temp_pose, temp_velocity);
      statevars.push_back(temp_statevar);
      traj_states.push_back(temp);

      // T_s0 = T_sv * T_v0
      TransformEvaluator::Ptr rec_pose = steam::se3::compose(tf_gps_vehicle_, temp_pose);
      receiver_poses.emplace_back(rec_pose);                                      // T_k0
      enu_poses.emplace_back(steam::se3::compose(rec_pose, T_0g));   // T_kg = T_k0 * T_0g
    }

    // add TDCP terms
    for (uint k = 0; k < msgs_.size(); ++k) {
      TransformEvaluator::ConstPtr
          T_k1k = steam::se3::compose(receiver_poses[k + 1], steam::se3::inverse(receiver_poses[k]));
      PositionEvaluator::ConstPtr r_ba_ina(new PositionEvaluator(T_k1k));

      // using constant covariance here for now
      steam::BaseNoiseModel<1>::Ptr tdcp_noise_model(new steam::StaticNoiseModel<1>(tdcp_cov_));

      // iterate through satellite pairs in msg and add TDCP costs
      for (const auto &pair : msgs_[k].first.pairs) {
        Eigen::Vector3d r_1a_ing_ata{pair.r_1a_a.x, pair.r_1a_a.y, pair.r_1a_a.z};
        Eigen::Vector3d r_1a_ing_atb{pair.r_1a_b.x, pair.r_1a_b.y, pair.r_1a_b.z};
        Eigen::Vector3d r_2a_ing_ata{pair.r_2a_a.x, pair.r_2a_a.y, pair.r_2a_a.z};
        Eigen::Vector3d r_2a_ing_atb{pair.r_2a_b.x, pair.r_2a_b.y, pair.r_2a_b.z};

        steam::TdcpErrorEval::Ptr tdcp_error(new steam::TdcpErrorEval(pair.phi_measured,
                                                                      r_ba_ina,
                                                                      enu_poses[k],   // T_kg
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
    }

    // todo: try locking T_0g instead when we already have a good estimate of it
    // add prior on initial pose to deal with roll uncertainty and constrain r^0g_g to zero
    steam::BaseNoiseModel<6>::Ptr
        sharedNoiseModel(new steam::StaticNoiseModel<6>(pose_prior_cov_));
    steam::TransformErrorEval::Ptr prior_error_func(new steam::TransformErrorEval(init_pose_, T_0g));
    pose_prior_cost_ = steam::WeightedLeastSqCostTerm<6, 6>::Ptr(new steam::WeightedLeastSqCostTerm<6, 6>(
        prior_error_func,
        sharedNoiseModel,
        pp_loss_function_));

    steam::BaseNoiseModel<4>::Ptr nonholonomic_noise_model(new steam::StaticNoiseModel<4>(nonholonomic_cov_));
    trajectory_ = std::make_shared<SteamTrajInterface>(SteamTrajInterface(smoothing_factor_information_, true));

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
        const TransformEvaluator::Ptr &temp_pose = traj_state.getPose();
        const steam::VectorSpaceStateVar::Ptr &temp_velocity = traj_state.getVelocity();
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
        std::cout << "Steam has failed to optimise the problem! This is an ERROR." << std::endl;
        resetEstimator();
        return;
      } else {
        // yes: just a marginal problem, let's use what we got
        std::cout << "Steam has failed due to an unsuccessful step. This should be OK if it happens infrequently."
                  << std::endl;
      }
    } catch (steam::decomp_failure &e) {
      // Should not occur frequently
      std::cout << "Steam has encountered an LL^T decomposition error while optimizing! This is an ERROR." << std::endl;
      resetEstimator();
      return;
    }

    // print final costs (for debugging/development)
    printCosts(true);

    // update with optimized transforms
    for (uint i = 0; i < msgs_.size(); ++i) {
      msgs_[i].second = statevars[i + 1]->getValue() * statevars[i]->getValue().inverse();  // T_21 = T_20 * inv(T_10)
    }

    // update our orientation estimate
    init_pose_ = T_0g_statevar->getValue();

    if (!fixed_rate_publish_) {
      double t_n = (double) msgs_.back().first.t_b * 1e-9;
      double t_n1 = (double) msgs_.back().first.t_a * 1e-9;
      lgmath::se3::Transformation T_ng = statevars.back()->getValue() * init_pose_;
      const auto &T_n_n1 = msgs_.back().second;
      publishPoses(T_ng, T_n_n1);
      saveToFile(t_n, t_n1, T_ng, T_n_n1);
    }

    init_pose_estimated_ = true;
    first_window_ = false;
  }
}

void CpoBackEnd::_timedCallback() {
  if (first_window_ || !init_pose_estimated_ || trajectory_ == nullptr) {
    std::cout << "Not publishing because don't currently have a valid pose estimate." << std::endl;
    return;
  }

  // grab times and extrapolate poses
  double t_last_msg = (double) msgs_.back().first.t_b * 1e-9;
  double t_n = get_clock()->now().seconds();
  double t_n1 = t_n - 1.0;

  if (t_n - t_last_msg > traj_timeout_limit_){
    std::cout << "Latest carrier phase measurement is " << t_n - t_last_msg << " seconds old. "
    << "Will not publish because trajectory is no longer valid." << std::endl;
    return;
  }

  lgmath::se3::TransformationWithCovariance T_0g = init_pose_;
  lgmath::se3::TransformationWithCovariance T_n0 = trajectory_->getInterpPoseEval(t_n)->evaluate();
  lgmath::se3::TransformationWithCovariance T_n10 = trajectory_->getInterpPoseEval(t_n1)->evaluate();

  lgmath::se3::TransformationWithCovariance T_ng = T_n0 * T_0g;                // todo: get correct cov
  lgmath::se3::TransformationWithCovariance T_n_n1 = T_n0 * T_n10.inverse();

  // publish
  publishPoses(T_ng, T_n_n1);
  saveToFile(t_n, t_n1, T_ng, T_n_n1);
}

void CpoBackEnd::publishPoses(const lgmath::se3::TransformationWithCovariance &T_ng,
                              const lgmath::se3::TransformationWithCovariance &T_n_n1) {
  geometry_msgs::msg::PoseWithCovariance relative_pose_msg = toPoseMsg(T_n_n1);
  vehicle_publisher_->publish(relative_pose_msg);
  geometry_msgs::msg::PoseWithCovariance enu_pose_msg = toPoseMsg(T_ng);
  enu_publisher_->publish(enu_pose_msg);
  std::cout << "Message published! " << std::endl;
}

void CpoBackEnd::saveToFile(double t_n,
                            double t_n1,
                            const lgmath::se3::Transformation &T_ng,
                            const lgmath::se3::Transformation &T_n_n1) const {
  // append latest estimate to file
  std::ofstream outstream;
  outstream.open(results_path_, std::ofstream::out | std::ofstream::app);

  const auto r_ng_g = T_ng.r_ba_ina();

  // also get receiver position estimate for comparing to ground truth
  auto r_sg_g = (tf_gps_vehicle_->evaluate() * T_ng).r_ba_ina();

  // save times and global position for easy plotting
  outstream << std::setprecision(12) << t_n << ", " << t_n1 << ", ";
  outstream << r_sg_g[0] << ", " << r_sg_g[1] << ", " << r_sg_g[2] << ", ";
  outstream << r_ng_g[0] << ", " << r_ng_g[1] << ", " << r_ng_g[2] << ", ";

  // save full transformations as well. Transpose needed to print in row-major order
  auto temp = T_ng.matrix().transpose();
  auto T_ng_flat = std::vector<double>(temp.data(), temp.data() + 16);
  for (auto entry : T_ng_flat) outstream << entry << ",";

  temp = T_n_n1.matrix().transpose();
  auto T_n_n1_flat = std::vector<double>(temp.data(), temp.data() + 16);
  for (auto entry : T_n_n1_flat) outstream << entry << ",";

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
  this->declare_parameter("results_path");
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
  results_path_ = this->get_parameter("results_path").as_string();
  steam_verbose_ = this->get_parameter("solver_verbose").as_bool();
  steam_max_iterations_ = this->get_parameter("solver_max_iterations").as_int();
  traj_timeout_limit_ = this->get_parameter("traj_timeout_limit").as_double();
}

void CpoBackEnd::initializeProblem() {
  // setup loss functions
  tdcp_loss_function_.reset(new steam::DcsLossFunc(2.0));
  nonholonomic_loss_function_.reset(new steam::L2LossFunc());
  pp_loss_function_.reset(new steam::L2LossFunc());

  // setup cost terms
  tdcp_cost_terms_.reset(new steam::ParallelizedCostTermCollection());
  nonholonomic_cost_terms_.reset(new steam::ParallelizedCostTermCollection());
  smoothing_cost_terms_.reset(new steam::ParallelizedCostTermCollection());

  // set up the steam problem
  problem_.reset(new steam::OptimizationProblem());

  if (!init_pose_estimated_) {
    // estimate initial T_0g from code solutions
    Eigen::Vector3d r_k0_ing = toEigenVec3d(msgs_.back().first.enu_pos) - toEigenVec3d(msgs_.front().first.enu_pos);
    double theta = atan2(r_k0_ing.y(), r_k0_ing.x());

    Eigen::Matrix<double, 6, 1> init_pose_vec;
    init_pose_vec << toEigenVec3d(msgs_.front().first.enu_pos), 0, 0, -1 * theta;

    init_pose_ = lgmath::se3::Transformation(init_pose_vec);

    if (first_window_) {
      // save ENU origin to results file
      std::ofstream outstream;
      outstream.open(results_path_);
      Eigen::Vector3d enu_origin = toEigenVec3d(msgs_.back().first.enu_origin);
      outstream << std::setprecision(9) << enu_origin[0] << "," << enu_origin[1] << "," << enu_origin[2] << std::endl;
      outstream.close();
    } else {
      std::cout << "Warning: new initial pose was set midway through run."
                << "Estimates on either side of this time should not be compared." << std::endl;
    }
  }
}

void CpoBackEnd::resetEstimator() {
  std::cout << "Clearing window and resetting the estimator." << std::endl;
  msgs_.clear();
  trajectory_ = nullptr;
  init_pose_estimated_ = false;
}

void CpoBackEnd::printCosts(bool final) {
  std::string stage_str = final ? "Final" : "Initial";
  std::cout << " === " << stage_str << " Costs === " << std::endl;
  std::cout << "Carrier Phase:       " << tdcp_cost_terms_->cost() << "        Terms:  "
            << tdcp_cost_terms_->numCostTerms() << std::endl;
  std::cout << "Nonholonomic:        " << nonholonomic_cost_terms_->cost() << "        Terms:  "
            << nonholonomic_cost_terms_->numCostTerms() << std::endl;
  std::cout << "Smoothing:           " << smoothing_cost_terms_->cost() << "        Terms:  "
            << smoothing_cost_terms_->numCostTerms() << std::endl;
  std::cout << "Pose Prior:          " << pose_prior_cost_->cost() << "        Terms:  1" << std::endl;
}

void CpoBackEnd::addMsgToWindow(const cpo_interfaces::msg::TDCP::SharedPtr &msg) {

  if (!msgs_.empty() && msg->t_a != msgs_.back().first.t_b) {
    // times don't align, so we've likely missed a msg. To be safe we will clear it for now
    std::cout << "Warning: mismatched times. Clearing msgs_. Current t_a: " << msg->t_a << ". Previous t_b: "
              << msgs_.back().first.t_b << std::endl;
    std::deque<std::pair<cpo_interfaces::msg::TDCP, lgmath::se3::Transformation>>().swap(msgs_);
  }

  // add latest message
  double dist_since_last = msgs_.empty() ? 0 : (toEigenVec3d(msg->enu_pos) - toEigenVec3d(msgs_.back().first.enu_pos)).norm();

  if (dist_since_last > 1.2) dist_since_last = 0.9;   // hacky error checking. todo: something else

  Eigen::Vector3d r_ba_est{dist_since_last, 0.0, 0.0};
  lgmath::se3::Transformation new_T_estimate = lgmath::se3::Transformation(Eigen::Matrix3d::Identity(), r_ba_est);

  msgs_.emplace_back(*msg, new_T_estimate);

  // if we have a full queue, discard the oldest msg
  while (msgs_.size() > window_size_) {
    init_pose_ = msgs_.front().second * init_pose_; // incrementing indices so need to update our T_0g estimate
    msgs_.pop_front();
  }
}

geometry_msgs::msg::PoseWithCovariance CpoBackEnd::toPoseMsg(lgmath::se3::TransformationWithCovariance T) {
  Eigen::Quaterniond q(T.C_ba());
  Eigen::Vector3d r_ba_ina = T.r_ba_ina();

  geometry_msgs::msg::PoseWithCovariance msg;
  msg.pose.position.set__x(r_ba_ina[0]);
  msg.pose.position.set__y(r_ba_ina[1]);
  msg.pose.position.set__z(r_ba_ina[2]);
  msg.pose.orientation.set__x(q.x());
  msg.pose.orientation.set__y(q.y());
  msg.pose.orientation.set__z(q.z());
  msg.pose.orientation.set__w(q.w());

  if (!T.covarianceSet())
    T.setZeroCovariance();

  std::array<double, 36> temp{};
  Eigen::Matrix<double, 6, 6>::Map(temp.data()) = T.cov();
  msg.set__covariance(temp);

  return msg;
}

Eigen::Vector3d CpoBackEnd::toEigenVec3d(const geometry_msgs::msg::Vector3 &ros_vec) {
  return Eigen::Vector3d{ros_vec.x, ros_vec.y, ros_vec.z};
}
