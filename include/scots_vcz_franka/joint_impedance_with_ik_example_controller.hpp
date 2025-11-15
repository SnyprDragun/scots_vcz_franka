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

#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <franka_example_controllers/robot_utils.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <rclcpp/rclcpp.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_example_controllers {

/**
 * Joint impedance controller with IK for Gazebo simulation.
 * Gets desired pose and uses inverse kinematics from MoveIt.
 * IK returns the desired joint positions from the desired pose.
 * Desired joint positions are fed to the impedance control law together with the current
 * joint velocities to calculate the desired joint torques.
 */
class JointImpedanceWithIKExampleController : public controller_interface::ControllerInterface {
 public:
  using Vector7d = Eigen::Matrix<double, 7, 1>;
  
  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  /**
   * @brief Updates joint states from state interfaces
   */
  void update_joint_states();

  /**
   * @brief Computes forward kinematics to get end-effector pose
   * 
   * @param joint_positions Current joint positions
   * @return Eigen::Affine3d End-effector transformation
   */
  Eigen::Affine3d forward_kinematics(const std::vector<double>& joint_positions);

  /**
   * @brief Calculates the new pose based on the initial pose.
   *
   * @return Eigen::Vector3d calculated sinusoidal period for the x,z position of the pose.
   */
  Eigen::Vector3d compute_new_position();

  /**
   * @brief Creates the IK service request for IK service from MoveIt.
   *
   * @return std::shared_ptr<moveit_msgs::srv::GetPositionIK::Request> request service message
   */
  std::shared_ptr<moveit_msgs::srv::GetPositionIK::Request> create_ik_service_request(
      const Eigen::Vector3d& new_position,
      const Eigen::Quaterniond& new_orientation,
      const std::vector<double>& joint_positions_current);

  /**
   * @brief Computes the torque commands based on impedance control law
   *
   * @return Eigen::Matrix<double, 7, 1> torque for each joint of the robot
   */
  Eigen::Matrix<double, 7, 1> compute_torque_command(const Eigen::Matrix<double, 7, 1>& joint_positions_desired,
                                  const Eigen::Matrix<double, 7, 1>& joint_positions_current,
                                  const Eigen::Matrix<double, 7, 1>& joint_velocities_current);

  /**
   * @brief Assigns the Kp, Kd and arm_id parameters
   *
   * @return true when parameters are present, false when parameters are not available
   */
  bool assign_parameters();

  // ROS2 service client for IK
  rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedPtr compute_ik_client_;

  // End-effector pose
  Eigen::Quaterniond orientation_;
  Eigen::Vector3d position_;

  // Controller parameters
  std::string arm_id_;
  bool is_gripper_loaded_ = true;
  std::string robot_description_;
  bool initialization_flag_{true};

  // Time tracking
  double elapsed_time_{0.0};

  // Control gains
  Eigen::Matrix<double, 7, 1> dq_filtered_;
  Eigen::Matrix<double, 7, 1> k_gains_;
  Eigen::Matrix<double, 7, 1> d_gains_;
  int num_joints_{7};

  // Joint states
  std::vector<double> joint_positions_desired_;
  std::vector<double> joint_positions_current_;
  std::vector<double> joint_velocities_current_;
};

}  // namespace franka_example_controllers