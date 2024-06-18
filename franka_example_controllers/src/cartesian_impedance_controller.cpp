// Copyright (c) 2021 Franka Emika GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "franka_example_controllers/cartesian_impedance_controller.hpp"
#include "franka_example_controllers/pseudo_inversion.hpp"

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <Eigen/Eigen>

namespace franka_example_controllers {

controller_interface::InterfaceConfiguration
CartesianImpedanceController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

//claim all state interfaces
controller_interface::InterfaceConfiguration
CartesianImpedanceController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; ++i) {
    state_interfaces_config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
    state_interfaces_config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }
  for (const auto& franka_robot_model_name : franka_robot_model_->get_state_interface_names()) {
    state_interfaces_config.names.push_back(franka_robot_model_name);
  }
  return state_interfaces_config;
}

controller_interface::return_type CartesianImpedanceController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& period) {

  //keeps track of time
  init_time_ = init_time_ + period;
  updateJointStates();
  
  std::array<double, 7> coriolis_array = franka_robot_model_->getCoriolis();
  //jacobian wrt base frame
  //FIXME: This hardcodes control for the point between the franka gripper fingers
  std::array<double, 42> jacobian_array = 
      franka_robot_model_->getZeroJacobian(franka::Frame::kEndEffector);
  std::array<double, 16> pose_array = franka_robot_model_->getPose(franka::Frame::kEndEffector);
  // convert to Eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(franka_robot_model_->getTauJ_d().data());
  //Eigen::Map<Eigen::Matrix<double, 7, 1>> q(q_.data());
  //Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(dq_.data());

  dq_filtered_ = filter_params_ * dq_ + (1-filter_params_)*dq_filtered_;

  Eigen::Affine3d transform(Eigen::Matrix4d::Map(pose_array.data()));
  Eigen::Vector3d position(transform.translation());
  //NOTE .linear() returns the rotation part for an affine transform
  Eigen::Quaterniond orientation(transform.linear());

  // compute error to desired pose
  // position error
  Eigen::Matrix<double, 6, 1> error;
  error.head(3) << position - position_d_;

  // orientation error, make sure the quaternion didn't flip sign
  if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();
  }
  // "difference" quaternion
  Eigen::Quaterniond error_quaternion(orientation * orientation_d_.inverse());
  // convert to axis angle
  Eigen::AngleAxisd error_quaternion_angle_axis(error_quaternion);
  // compute "orientation error"
  error.tail(3) << error_quaternion_angle_axis.axis() * error_quaternion_angle_axis.angle();

  // compute control
  // allocate variables
  Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7);

  // pseudoinverse for nullspace handling
  // kinematic pseuoinverse
  Eigen::MatrixXd jacobian_transpose_pinv;
  pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

  // Cartesian PD control with damping ratio = 1
  tau_task << jacobian.transpose() *
                  (-cartesian_stiffness_ * error - cartesian_damping_ * (jacobian * dq_filtered_));
  // nullspace PD control with damping ratio = 1
  tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                    jacobian.transpose() * jacobian_transpose_pinv) *
                       (nullspace_stiffness_ * (q_d_nullspace_ - q_) -
                        (2.0 * sqrt(nullspace_stiffness_)) * dq_filtered_);
  // Desired torque
  tau_d << tau_task + tau_nullspace + coriolis;
  // Saturate torque rate to avoid discontinuities
  tau_d << saturateTorqueRate(tau_d, tau_d_prev_);
  //tau_d << saturateTorqueRate(tau_d, tau_J_d);
  tau_d_prev_ = tau_d;
  for (size_t i = 0; i < 7; ++i) {
    command_interfaces_[i].set_value(tau_d(i));
  }

  //std::cerr<<"task={"<<tau_task.transpose()<<"}, tau_d={"<<tau_d.transpose()<<"}\n";
  //check-update for new desired stiffness and equilibrium pose
  auto input_s{stiffness_buffer_.readFromRT()};
  if(input_s != nullptr) {
    if((*input_s) != nullptr) {
      k_gains_<<(*input_s)->linear.x,(*input_s)->linear.y,(*input_s)->linear.z,
                (*input_s)->angular.x,(*input_s)->angular.y,(*input_s)->angular.z;
      cartesian_stiffness_target_ = k_gains_.array().matrix().asDiagonal();
      d_gains_ = 2.0*k_gains_.cwiseSqrt();
      cartesian_damping_target_ = d_gains_.array().matrix().asDiagonal();
    }
  } 
  auto input_p{pose_buffer_.readFromRT()};
  if(input_p != nullptr) {
    if((*input_p) != nullptr) {
      position_d_target_ << (*input_p)->position.x, 
                         (*input_p)->position.y, (*input_p)->position.z;
      Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);
      orientation_d_target_.coeffs() << (*input_p)->orientation.x, 
                                        (*input_p)->orientation.y,
                                        (*input_p)->orientation.z, 
                                        (*input_p)->orientation.w;
      if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0) {
        orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
      }
    }
  } 

  // update parameters changed online either through dynamic reconfigure or through the interactive
  // target by filtering
  cartesian_stiffness_ =
      filter_params_ * cartesian_stiffness_target_ + (1.0 - filter_params_) * cartesian_stiffness_;
  cartesian_damping_ =
      filter_params_ * cartesian_damping_target_ + (1.0 - filter_params_) * cartesian_damping_;
  nullspace_stiffness_ =
      filter_params_ * nullspace_stiffness_target_ + (1.0 - filter_params_) * nullspace_stiffness_;
  position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
  orientation_d_ = orientation_d_target_;
  /*
  Eigen::AngleAxisd aa_orientation_d(orientation_d_);
  Eigen::AngleAxisd aa_orientation_d_target(orientation_d_target_);
  aa_orientation_d.axis() = filter_params_ * aa_orientation_d_target.axis() +
                            (1.0 - filter_params_) * aa_orientation_d.axis();
  aa_orientation_d.angle() = filter_params_ * aa_orientation_d_target.angle() +
                             (1.0 - filter_params_) * aa_orientation_d.angle();
  orientation_d_ = Eigen::Quaterniond(aa_orientation_d);
  */

  if (error_realtime_publisher_ && error_realtime_publisher_->trylock()) {
    error_realtime_publisher_->msg_.linear.x = error(0);
    error_realtime_publisher_->msg_.linear.y = error(1);
    error_realtime_publisher_->msg_.linear.z = error(2);
    ////
    error_realtime_publisher_->msg_.linear.y = coriolis(6);
    error_realtime_publisher_->msg_.linear.z = tau_nullspace(6);
    error_realtime_publisher_->msg_.angular.x = tau_d(6);
    error_realtime_publisher_->msg_.angular.y = tau_J_d(6);
    error_realtime_publisher_->msg_.angular.z = tau_task(6);
    //error_realtime_publisher_->msg_.angular.x = error(3);
    //error_realtime_publisher_->msg_.angular.y = error(4);
    //error_realtime_publisher_->msg_.angular.z = error(5);
    error_realtime_publisher_->unlockAndPublish();
  }
/*
  std::cerr<<"e="<<error.transpose()<<"\ntau="<<tau_d.transpose()<<std::endl;
  std::cerr<<"tau_null="<<tau_nullspace.transpose()<<"\ntau_task="<<tau_task.transpose()<<std::endl;
  std::cerr<<"tau_d_prev="<<tau_d_prev_.transpose()<<"\n coriolis="<<coriolis.transpose()<<std::endl;
  std::cerr<<"p="<<position_d_.transpose()<<" o="<<orientation_d_<<" \n s="<<cartesian_stiffness_.transpose()<<std::endl;
  */
  return controller_interface::return_type::OK;
}

//helper function
Eigen::Matrix<double, 7, 1> CartesianImpedanceController::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] =
        tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  }
  return tau_d_saturated;
}

CallbackReturn CartesianImpedanceController::on_init() {
  try {
    auto_declare<std::string>("arm_id", "panda");
    auto_declare<std::vector<double>>("k_gains", {});
    auto_declare<std::vector<double>>("d_gains", {});
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianImpedanceController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  RCLCPP_DEBUG(get_node()->get_logger(), "configuring rm %s", arm_id_.c_str());

  franka_robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
      franka_semantic_components::FrankaRobotModel(arm_id_ + "/" + k_robot_model_interface_name,
                                                   arm_id_ + "/" + k_robot_state_interface_name));

  RCLCPP_DEBUG(get_node()->get_logger(), "configured model pointer successfully");
  
  auto k_gains = get_node()->get_parameter("k_gains").as_double_array();
  auto d_gains = get_node()->get_parameter("d_gains").as_double_array();
  if (k_gains.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "k_gains parameter not set");
    return CallbackReturn::FAILURE;
  }
  if (k_gains.size() != static_cast<uint>(num_cdof)) {
    RCLCPP_FATAL(get_node()->get_logger(), "k_gains should be of size %d but is of size %ld",
                 num_joints, k_gains.size());
    return CallbackReturn::FAILURE;
  }
  if (d_gains.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "d_gains parameter not set");
    return CallbackReturn::FAILURE;
  }
  if (d_gains.size() != static_cast<uint>(num_cdof)) {
    RCLCPP_FATAL(get_node()->get_logger(), "d_gains should be of size %d but is of size %ld",
                 num_joints, d_gains.size());
    return CallbackReturn::FAILURE;
  }
  for (int i = 0; i < num_cdof; ++i) {
    d_gains_(i) = d_gains.at(i);
    k_gains_(i) = k_gains.at(i);
  }
  dq_filtered_.setZero();

  std::cerr<<"Cartesian impedance controller configuring\n";
  std::cerr<<"Desired k_gains: "<<k_gains_.transpose()<<"\n\t d_gains:"
           <<d_gains_.transpose()<<std::endl;
  //set initial and desired stiffness/damping
  cartesian_stiffness_target_ = k_gains_.array().matrix().asDiagonal();
  cartesian_damping_target_ = d_gains_.array().matrix().asDiagonal();
  cartesian_stiffness_.setZero();
  cartesian_damping_.setZero();

  std::cerr<<"Stiffness matrix: "<<cartesian_stiffness_target_<<std::endl;

  //registering subscribers
  stiffness_sub_ = get_node()->create_subscription<Twist>(
    "~/desired_stiffness",
    rclcpp::QoS(1),
    [this](const Twist::SharedPtr message) {
      stiffness_buffer_.writeFromNonRT(message);
    });
  pose_sub_ = get_node()->create_subscription<Pose>(
    "~/equilibrium_pose",
    rclcpp::QoS(1),
    [this](const Pose::SharedPtr message) {
      pose_buffer_.writeFromNonRT(message);
    });

  error_publisher_ = get_node()->create_publisher<Twist>("~/error", rclcpp::QoS(1));
  error_realtime_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<Twist> >(error_publisher_);


  return CallbackReturn::SUCCESS;
}

void CartesianImpedanceController::updateJointStates() {
  //std::cerr<<"state interfaces are "<<state_interfaces_.size()<<" long\n";
  for (auto i = 0; i < num_joints; ++i) {
    const auto& position_interface = state_interfaces_.at(2 * i);
    const auto& velocity_interface = state_interfaces_.at(2 * i + 1);

    assert(position_interface.get_interface_name() == "position");
    assert(velocity_interface.get_interface_name() == "velocity");

    q_(i) = position_interface.get_value();
    dq_(i) = velocity_interface.get_value();
  }
}

CallbackReturn CartesianImpedanceController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);
  
  //set initial target pose to current pose
  updateJointStates();
  std::cerr<<"Current joint angles q="<<q_<<std::endl;

  //FIXME: This hardcodes control for the point between the franka gripper fingers
  std::array<double, 16> pose_array = franka_robot_model_->getPose(franka::Frame::kEndEffector);
  tau_d_prev_.setZero();// Vector7d(franka_robot_model_->getTauJ_d().data());
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(pose_array.data()));

  std::cerr<<"Initial equilibrium pose = "<<initial_transform.matrix()<<std::endl;

  // set equilibrium point to current state
  position_d_ = initial_transform.translation();
  orientation_d_ = Eigen::Quaterniond(initial_transform.linear());
  position_d_target_ = initial_transform.translation();
  orientation_d_target_ = Eigen::Quaterniond(initial_transform.linear());

  std::cerr<<"Position "<<position_d_.transpose()<<" orientation "<<orientation_d_<<std::endl;
  // set nullspace equilibrium configuration to initial q
  q_d_nullspace_ = q_;

  initial_q_ = q_;
  start_time_ = this->get_node()->now();
  init_time_ = rclcpp::Duration(0, 0);

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianImpedanceController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_robot_model_->release_interfaces();
  if (error_realtime_publisher_) {
    error_realtime_publisher_->stop();
  }

  return CallbackReturn::SUCCESS;
}


CallbackReturn CartesianImpedanceController::on_error(
  const rclcpp_lifecycle::State& /*previous_state*/){
    RCLCPP_ERROR(this->get_node()->get_logger(), "error encountered!");
    franka_robot_model_->release_interfaces();
    return CallbackReturn::ERROR;
  }

}  // namespace franka_example_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianImpedanceController,
                       controller_interface::ControllerInterface)
