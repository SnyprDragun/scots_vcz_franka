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
#include <fstream>
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
  static bool trajectory_loaded = false;
  static std::vector<Eigen::Vector3d> position_steps;
  static size_t position_index = 0;
  static const std::string position_filename = "/home/focaslab/SCOTS/SCOTS_ros2_v2/SCOTS/examples/franka/fr3_hw_final/fr3_hw_ex_2_final.csv";

  if (!trajectory_loaded) {
    std::ifstream position_file(position_filename);
    if (!position_file.is_open()) {
      std::cerr << "Error: Could not open trajectory position file: " << position_filename << std::endl;
      trajectory_loaded = true;
      return position_;
    }

    std::string position_line;
    if (std::getline(position_file, position_line)) {
    }

    while (std::getline(position_file, position_line)) {
      std::stringstream ss(position_line);
      std::string cell;
      std::vector<double> coords;

      while (std::getline(ss, cell, ',')) {
        try {
          coords.push_back(std::stod(cell));
        } catch (const std::exception& e) {
          std::cerr << "Warning: Malformed data in CSV line. Skipping." << std::endl;
          coords.clear();
          break;
        }
      }

      if (coords.size() == 3) {
          position_steps.emplace_back(coords[0], coords[1], coords[2]);
      }
    }

    if (position_steps.empty()) {
        std::cerr << "Error: Trajectory file was empty or contained no valid data." << std::endl;
    } else {
        std::cout << "Trajectory loaded successfully with " << position_steps.size() << " steps." << std::endl;
    }

    trajectory_loaded = true;
  }

  if (!position_steps.empty()) {
    Eigen::Vector3d new_position_command = position_steps[position_index];
    position_index++;
    if (position_index >= position_steps.size()) {
        position_index = 0;
    }

    return new_position_command;
  }

  // const double radius = 0.1; // 10 cm radius for the circle
  
  // // Set angular velocity: A full circle (2*M_PI) will take 10 seconds
  // const double angle = M_PI / 4 * (1 - std::cos(M_PI / 5 * elapsed_time_)); // Ramps up and down over 10 seconds 

  // // Calculate a continuously increasing angle based on elapsed time
  // // Unlike the original, this angle grows indefinitely, causing continuous rotation
  // // double angle = angular_velocity * elapsed_time_; 

  // // Calculate the change in X and Y using standard circle equations.
  // // We use (1 - cos(angle)) for X and sin(angle) for Y.
  // // At t=0, angle=0:
  // //   delta_x = 0.1 * (1 - 1) = 0
  // //   delta_y = 0.1 * (0)     = 0
  // // This ensures the motion *starts* at the home position (0,0 delta).
  // double delta_x = radius * std::sin(angle);
  // double delta_z = radius * (std::cos(angle) - 1);

  // // Start from the initial home position read during activation
  // Eigen::Vector3d new_position = position_;
  
  // // Apply the calculated deltas to the X and z coordinates
  // new_position.x() -= delta_x;
  // new_position.y() -= delta_z;
  
  // // The Z coordinate remains constant at the home position's height
  // // new_position.z() is untouched
  // return new_position;
}

Eigen::Vector3d JointImpedanceWithIKExampleController::compute_new_orientation() {
  static bool trajectory_loaded = false;
  static std::vector<Eigen::Vector3d> orientation_steps;
  static size_t orientation_index = 0;
  static const std::string orientation_filename = "/home/focaslab/SCOTS/SCOTS_ros2_v2/SCOTS/examples/franka/fr3_hw_final/simulated_orientation_20000.csv";
  orientation = {0.0, 0.0, 0.0};

  if (!trajectory_loaded) {
    std::ifstream orientation_file(orientation_filename);
    if (!orientation_file.is_open()) {
      std::cerr << "Error: Could not open trajectory orientation file: " << orientation_filename << std::endl;
      trajectory_loaded = true;
      return orientation;
    }

    std::string orientation_line;
    if (std::getline(orientation_file, orientation_line)) {
    }

    while (std::getline(orientation_file, orientation_line)) {
      std::stringstream ss(orientation_line);
      std::string cell;
      std::vector<double> coords;

      while (std::getline(ss, cell, ',')) {
        try {
          coords.push_back(std::stod(cell));
        } catch (const std::exception& e) {
          std::cerr << "Warning: Malformed data in CSV line. Skipping." << std::endl;
          coords.clear();
          break;
        }
      }

      if (coords.size() == 3) {
          orientation_steps.emplace_back(coords[0], coords[1], coords[2]);
      }
    }

    if (orientation_steps.empty()) {
        std::cerr << "Error: Trajectory file was empty or contained no valid data." << std::endl;
    } else {
        std::cout << "Trajectory loaded successfully with " << orientation_steps.size() << " steps." << std::endl;
    }

    trajectory_loaded = true;
  }

  if (!orientation_steps.empty()) {
    Eigen::Vector3d new_orientation_command = orientation_steps[orientation_index];
    orientation_index++;
    if (orientation_index >= orientation_steps.size()) {
        orientation_index = 0;
    }

    return orientation;
    // return new_orientation_command;
  } 
}

Eigen::Quaterniond JointImpedanceWithIKExampleController::convert_rpy_to_quaternion(const Eigen::Vector3d& rpy_angles) {
    // Eigen's AngleAxis is a robust way to create rotations.
    // We concatenate three simple rotations (Z-Y-X sequence is common for RPY)
    // The order of multiplication matters: R = R_z * R_y * R_x
    
    Eigen::AngleAxisd roll_angle(rpy_angles(0), Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitch_angle(rpy_angles(1), Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw_angle(rpy_angles(2), Eigen::Vector3d::UnitZ());

    // Construct the quaternion: Q = Q_z * Q_y * Q_x (extrinsic)
    // Or Q = Q_x * Q_y * Q_z (intrinsic, which is more common in robotics)
    // The code below uses the intrinsic ZYX (Yaw-Pitch-Roll) convention,
    // which is often denoted as R = R_z * R_y * R_x in matrix form.
    
    // Note: Eigen also allows direct conversion using: 
    // Eigen::Matrix3d R = (yaw_angle * pitch_angle * roll_angle).toRotationMatrix();
    // return Eigen::Quaterniond(R);
    
    Eigen::Quaterniond q = yaw_angle * pitch_angle * roll_angle;
    
    // Ensure the quaternion is normalized before returning
    return q.normalized();
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
  service_request->ik_request.pose_stamped.pose.orientation.x = 1;
  service_request->ik_request.pose_stamped.pose.orientation.y = 0;
  service_request->ik_request.pose_stamped.pose.orientation.z = 0;
  service_request->ik_request.pose_stamped.pose.orientation.w = 0;
  
  service_request->ik_request.robot_state.joint_state.name = {
    arm_id_ + "_joint1", arm_id_ + "_joint2", arm_id_ + "_joint3", arm_id_ + "_joint4",
    arm_id_ + "_joint5", arm_id_ + "_joint6", arm_id_ + "_joint7"};
  service_request->ik_request.robot_state.joint_state.position = joint_positions_current;

  if (is_gripper_loaded_) {
    service_request->ik_request.ik_link_name = arm_id_ + "_hand_tcp";
  }

  RCLCPP_INFO(get_node()->get_logger(), "Orientation (%.3f, %.3f, %.3f, %.3f)", 
    service_request->ik_request.pose_stamped.pose.orientation.x, 
    service_request->ik_request.pose_stamped.pose.orientation.y, 
    service_request->ik_request.pose_stamped.pose.orientation.z, 
    service_request->ik_request.pose_stamped.pose.orientation.w);
  RCLCPP_INFO(get_node()->get_logger(), "Position (%.3f, %.3f, %.3f)", position.x(), position.y(), position.z());
  
  return service_request;
}

Eigen::Matrix<double, 7, 1> JointImpedanceWithIKExampleController::compute_torque_command(
    const Eigen::Matrix<double, 7, 1>& joint_positions_desired,
    const Eigen::Matrix<double, 7, 1>& joint_positions_current,
    const Eigen::Matrix<double, 7, 1>& joint_velocities_current) {

  const double kAlpha = 0.99;
  dq_filtered_ = (1 - kAlpha) * dq_filtered_ + kAlpha * joint_velocities_current;
  
  // Eigen::Matrix<double, 7, 1> q_error = joint_positions_desired - joint_positions_current;
  // Eigen::Matrix<double, 7, 1> tau_d_calculated = k_gains_.cwiseProduct(q_error) - d_gains_.cwiseProduct(dq_filtered_);

  Vector7d taub = {10, 10 ,10 ,10, 5 ,5, 2}; // Joint Torque Bounds for external force
  Vector7d vb = {2, 2, 2, 2, 2.5, 1.5, 1}; // Joint Velocity Bounds

  Vector7d psi1 = {0, 0, 0, 0, 0, 0, 0};
  Vector7d psi2 = {0, 0, 0, 0, 0, 0, 0};

  double sigma = 0.1;
  double rho = 1;

  // double e = 0.0;
  for (int i = 0; i < 7; ++i) {
    double e = (joint_positions_current[i] - joint_positions_desired[i])/sigma;
    // if (i == 6) { 
    //   double e = (joint_positions_current[i])/sigma;
    // }
    psi1[i] = pow((tanh(e)),1);
    psi2[i] = tanh((dq_filtered_[i] + vb[i]*psi1[i])/rho);
  }

  // Calculate the desired torques
  // Vector7d tau_d_calculated = k_gains_.cwiseProduct(q_desired - q_) + d_gains_.cwiseProduct(-dq_filtered_);
  Vector7d tau_d_calculated = -taub.cwiseProduct(psi2);

  return tau_d_calculated;
}

controller_interface::return_type JointImpedanceWithIKExampleController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& period) {
  
  if (initialization_flag_) {
    // Get initial position from forward kinematics
    update_joint_states();
    // Eigen::Affine3d initial_transform = forward_kinematics(joint_positions_current_);
    // position_ = initial_transform.translation();
    // orientation_ = Eigen::Quaterniond(initial_transform.rotation());
    
    elapsed_time_ = 0.0;
    initialization_flag_ = false;
  } else {
    elapsed_time_ += period.seconds();
  }
  
  update_joint_states();

  Eigen::Vector3d new_position = compute_new_position();
  Eigen::Vector3d new_orientation = compute_new_orientation();
  Eigen::Quaterniond orientation_ = convert_rpy_to_quaternion(new_orientation);

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