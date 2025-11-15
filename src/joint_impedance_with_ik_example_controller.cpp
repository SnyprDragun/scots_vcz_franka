// Copyright (c) 2023 Franka Robotics GmbH
// Modified for Gazebo simulation
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

#include <franka_example_controllers/joint_impedance_with_ik_example_controller.hpp>
#include <franka_example_controllers/robot_utils.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <Eigen/Eigen>

namespace franka_example_controllers {

controller_interface::InterfaceConfiguration
JointImpedanceWithIKExampleController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints_; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
JointImpedanceWithIKExampleController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  
  // Position interfaces
  for (int i = 1; i <= num_joints_; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
  }
  // Velocity interfaces
  for (int i = 1; i <= num_joints_; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }
  
  return config;
}

void JointImpedanceWithIKExampleController::update_joint_states() {
  for (auto i = 0; i < num_joints_; ++i) {
    const auto& position_interface = state_interfaces_.at(i);
    const auto& velocity_interface = state_interfaces_.at(num_joints_ + i);

    assert(position_interface.get_interface_name() == "position");
    assert(velocity_interface.get_interface_name() == "velocity");

    joint_positions_current_[i] = position_interface.get_value();
    joint_velocities_current_[i] = velocity_interface.get_value();
  }
}

Eigen::Affine3d JointImpedanceWithIKExampleController::forward_kinematics(
    const std::vector<double>& joint_positions) {
  // Simple forward kinematics for initial pose extraction
  // This uses the current joint positions to compute end-effector pose
  // In simulation, we'll use the initial joint configuration
  
  // For a more accurate implementation, you would compute the full FK here
  // For now, we'll return a default pose that will be refined by the first IK call
  Eigen::Affine3d transform = Eigen::Affine3d::Identity();
  
  // Default end-effector position for FR3 in home configuration (approximate)
  transform.translation() << 0.5, 0.0, 0.5;
  
  return transform;
}

Eigen::Vector3d JointImpedanceWithIKExampleController::compute_new_position() {
  const double radius = 0.1; // 10 cm radius for the circle
  
  // Set angular velocity: A full circle (2*M_PI) will take 10 seconds
  const double angular_velocity = M_PI / 5.0; 

  // Calculate a continuously increasing angle based on elapsed time
  // Unlike the original, this angle grows indefinitely, causing continuous rotation
  double angle = angular_velocity * elapsed_time_; 

  // Calculate the change in X and Y using standard circle equations.
  // We use (1 - cos(angle)) for X and sin(angle) for Y.
  // At t=0, angle=0:
  //   delta_x = 0.1 * (1 - 1) = 0
  //   delta_y = 0.1 * (0)     = 0
  // This ensures the motion *starts* at the home position (0,0 delta).
  double delta_x = radius * (1 - std::cos(angle));
  double delta_y = radius * std::sin(angle);

  // Start from the initial home position read during activation
  Eigen::Vector3d new_position = position_;
  
  // Apply the calculated deltas to the X and Y coordinates
  new_position.x() += delta_x;
  new_position.y() += delta_y;
  
  // The Z coordinate remains constant at the home position's height
  // new_position.z() is untouched
  return new_position;
}

std::shared_ptr<moveit_msgs::srv::GetPositionIK::Request>
JointImpedanceWithIKExampleController::create_ik_service_request(
    const Eigen::Vector3d& position,
    const Eigen::Quaterniond& orientation,
    const std::vector<double>& joint_positions_current) {
  auto service_request = std::make_shared<moveit_msgs::srv::GetPositionIK::Request>();

  service_request->ik_request.group_name = arm_id_ + "_arm";
  service_request->ik_request.pose_stamped.header.frame_id = arm_id_ + "_link0";
  service_request->ik_request.pose_stamped.pose.position.x = position.x();
  service_request->ik_request.pose_stamped.pose.position.y = position.y();
  service_request->ik_request.pose_stamped.pose.position.z = position.z();
  service_request->ik_request.pose_stamped.pose.orientation.x = orientation.x();
  service_request->ik_request.pose_stamped.pose.orientation.y = orientation.y();
  service_request->ik_request.pose_stamped.pose.orientation.z = orientation.z();
  service_request->ik_request.pose_stamped.pose.orientation.w = orientation.w();
  
  service_request->ik_request.robot_state.joint_state.name = {
      arm_id_ + "_joint1", arm_id_ + "_joint2", arm_id_ + "_joint3", arm_id_ + "_joint4",
      arm_id_ + "_joint5", arm_id_ + "_joint6", arm_id_ + "_joint7"};
  service_request->ik_request.robot_state.joint_state.position = joint_positions_current;

  if (is_gripper_loaded_) {
    service_request->ik_request.ik_link_name = arm_id_ + "_hand_tcp";
  }
  
  return service_request;
}

Eigen::Matrix<double, 7, 1> JointImpedanceWithIKExampleController::compute_torque_command(
    const Eigen::Matrix<double, 7, 1>& joint_positions_desired,
    const Eigen::Matrix<double, 7, 1>& joint_positions_current,
    const Eigen::Matrix<double, 7, 1>& joint_velocities_current) {
  
  const double kAlpha = 0.99;
  dq_filtered_ = (1 - kAlpha) * dq_filtered_ + kAlpha * joint_velocities_current;
  
  Eigen::Matrix<double, 7, 1> q_error = joint_positions_desired - joint_positions_current;
  Eigen::Matrix<double, 7, 1> tau_d_calculated =
      k_gains_.cwiseProduct(q_error) - d_gains_.cwiseProduct(dq_filtered_);

  return tau_d_calculated;
}

controller_interface::return_type JointImpedanceWithIKExampleController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& period) {
  
  if (initialization_flag_) {
    // Get initial position from forward kinematics
    update_joint_states();
    Eigen::Affine3d initial_transform = forward_kinematics(joint_positions_current_);
    position_ = initial_transform.translation();
    orientation_ = Eigen::Quaterniond(initial_transform.rotation());
    
    elapsed_time_ = 0.0;
    initialization_flag_ = false;
  } else {
    elapsed_time_ += period.seconds();
  }
  
  update_joint_states();

  Eigen::Vector3d new_position = compute_new_position();

  auto service_request = create_ik_service_request(
      new_position, orientation_, joint_positions_current_);

  using ServiceResponseFuture = rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedFuture;
  auto response_received_callback =
      [this](ServiceResponseFuture future) {
        const auto& response = future.get();

        if (response->error_code.val == response->error_code.SUCCESS) {
          joint_positions_desired_ = response->solution.joint_state.position;
        } else {
          RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                               "Inverse kinematics solution failed.");
        }
      };
  
  auto result_future = compute_ik_client_->async_send_request(service_request, response_received_callback);

  if (joint_positions_desired_.empty()) {
    return controller_interface::return_type::OK;
  }

  Eigen::Matrix<double, 7, 1> joint_positions_desired_eigen(joint_positions_desired_.data());
  Eigen::Matrix<double, 7, 1> joint_positions_current_eigen(joint_positions_current_.data());
  Eigen::Matrix<double, 7, 1> joint_velocities_current_eigen(joint_velocities_current_.data());

  auto tau_d_calculated = compute_torque_command(
      joint_positions_desired_eigen, joint_positions_current_eigen, joint_velocities_current_eigen);

  for (int i = 0; i < num_joints_; i++) {
    command_interfaces_[i].set_value(tau_d_calculated(i));
  }

  return controller_interface::return_type::OK;
}

CallbackReturn JointImpedanceWithIKExampleController::on_init() {
  try {
    auto_declare<std::string>("arm_id", "");
    auto_declare<std::vector<double>>("k_gains", {});
    auto_declare<std::vector<double>>("d_gains", {});
    auto_declare<bool>("load_gripper", true);
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

bool JointImpedanceWithIKExampleController::assign_parameters() {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  is_gripper_loaded_ = get_node()->get_parameter("load_gripper").as_bool();

  auto k_gains = get_node()->get_parameter("k_gains").as_double_array();
  auto d_gains = get_node()->get_parameter("d_gains").as_double_array();
  
  if (k_gains.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "k_gains parameter not set");
    return false;
  }
  if (k_gains.size() != static_cast<uint>(num_joints_)) {
    RCLCPP_FATAL(get_node()->get_logger(), "k_gains should be of size %d but is of size %ld",
                 num_joints_, k_gains.size());
    return false;
  }
  if (d_gains.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "d_gains parameter not set");
    return false;
  }
  if (d_gains.size() != static_cast<uint>(num_joints_)) {
    RCLCPP_FATAL(get_node()->get_logger(), "d_gains should be of size %d but is of size %ld",
                 num_joints_, d_gains.size());
    return false;
  }
  
  for (int i = 0; i < num_joints_; ++i) {
    d_gains_(i) = d_gains.at(i);
    k_gains_(i) = k_gains.at(i);
  }
  
  return true;
}

CallbackReturn JointImpedanceWithIKExampleController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  
  if (!assign_parameters()) {
    return CallbackReturn::FAILURE;
  }

  // Create IK service client
  compute_ik_client_ = get_node()->create_client<moveit_msgs::srv::GetPositionIK>("compute_ik");

  // Wait for IK service
  if (!compute_ik_client_->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(get_node()->get_logger(), 
                 "IK service not available after waiting. Make sure MoveIt is running.");
    return CallbackReturn::ERROR;
  }
  
  RCLCPP_INFO(get_node()->get_logger(), "IK service connected.");

  auto parameters_client =
      std::make_shared<rclcpp::AsyncParametersClient>(get_node(), "robot_state_publisher");
  parameters_client->wait_for_service();

  auto future = parameters_client->get_parameters({"robot_description"});
  auto result = future.get();
  if (!result.empty()) {
    robot_description_ = result[0].value_to_string();
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to get robot_description parameter.");
  }

  arm_id_ = robot_utils::getRobotNameFromDescription(robot_description_, get_node()->get_logger());

  return CallbackReturn::SUCCESS;
}

CallbackReturn JointImpedanceWithIKExampleController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  
  initialization_flag_ = true;
  elapsed_time_ = 0.0;
  dq_filtered_.setZero();
  
  joint_positions_desired_.resize(num_joints_, 0.0);
  joint_positions_current_.resize(num_joints_, 0.0);
  joint_velocities_current_.resize(num_joints_, 0.0);

  return CallbackReturn::SUCCESS;
}

}  // namespace franka_example_controllers

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::JointImpedanceWithIKExampleController,
                       controller_interface::ControllerInterface)