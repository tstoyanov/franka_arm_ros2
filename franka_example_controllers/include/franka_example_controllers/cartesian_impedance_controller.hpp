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

#pragma once

#include <string>

#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <franka_semantic_components/franka_robot_model.hpp>
#include "realtime_tools/realtime_buffer.h"
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "realtime_tools/realtime_publisher.h"


using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_example_controllers {

/**
 * Provides a controller that maintains an equilibrium Cartesian pose. 
 * The pose and stiffness can be set via topic.
 */
class CartesianImpedanceController : public controller_interface::ControllerInterface {
 public:
  using Vector7d = Eigen::Matrix<double, 7, 1>;
  using Vector6d = Eigen::Matrix<double, 6, 1>;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override;
 private:
  using Twist = geometry_msgs::msg::Twist;
  using Pose = geometry_msgs::msg::Pose;

  std::string arm_id_;
  std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_robot_model_;

  const std::string k_robot_state_interface_name{"robot_state"};
  const std::string k_robot_model_interface_name{"robot_model"};
  
  const int num_joints = 7;
  const int num_cdof = 6;
  Vector7d q_;
  Vector7d initial_q_;
  Vector7d dq_;
  Vector7d dq_filtered_;
  Vector7d tau_d_prev_;

  Vector6d k_gains_;
  Vector6d d_gains_;
  rclcpp::Time start_time_;
  rclcpp::Duration init_time_ = rclcpp::Duration(0, 0);
  
  //impedance controller params
  double filter_params_{0.01};
  double nullspace_stiffness_{0.5};
  double nullspace_stiffness_target_{0.5};
  const double delta_tau_max_{1.0};
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_;
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_target_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_target_;
  Eigen::Matrix<double, 7, 1> q_d_nullspace_;
  Eigen::Vector3d position_d_;
  Eigen::Quaterniond orientation_d_;
  Eigen::Vector3d position_d_target_;
  Eigen::Quaterniond orientation_d_target_;

  void updateJointStates();

  //subscribers
  rclcpp::Subscription<Twist>::SharedPtr stiffness_sub_;
  rclcpp::Subscription<Pose>::SharedPtr pose_sub_;

  //realtime buffers
  realtime_tools::RealtimeBuffer<Twist::SharedPtr> stiffness_buffer_;
  realtime_tools::RealtimeBuffer<Pose::SharedPtr> pose_buffer_;

  rclcpp::Publisher<Twist>::SharedPtr error_publisher_;
  realtime_tools::RealtimePublisherSharedPtr<Twist> error_realtime_publisher_;



  // Saturation
  Eigen::Matrix<double, 7, 1> saturateTorqueRate(
      const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
      const Eigen::Matrix<double, 7, 1>& tau_J_d);  // NOLINT (readability-identifier-naming)

};

}  // namespace franka_example_controllers
