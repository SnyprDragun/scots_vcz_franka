#include <franka_example_controllers/scots_vcz_example_controller.hpp>
#include <cmath>
#include <algorithm>

namespace franka_example_controllers {

  controller_interface::InterfaceConfiguration ScotsVCZExampleController::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (int i = 1; i <= num_joints_; ++i) {
      config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
    }
    return config;
  }

  controller_interface::InterfaceConfiguration ScotsVCZExampleController::state_interface_configuration() const {
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

  void ScotsVCZExampleController::update_joint_states() {
    for (auto i = 0; i < num_joints_; ++i) {
      const auto& position_interface = state_interfaces_.at(i);
      const auto& velocity_interface = state_interfaces_.at(num_joints_ + i);

      assert(position_interface.get_interface_name() == "position");
      assert(velocity_interface.get_interface_name() == "velocity");

      joint_positions_current_[i] = position_interface.get_value();
      joint_velocities_current_[i] = velocity_interface.get_value();
    }
  }

  bool ScotsVCZExampleController::load_scots_controller(const std::string& csv_file_path) {
    std::ifstream file(csv_file_path);
    if (!file.is_open()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to open SCOTS controller file: %s", csv_file_path.c_str());
      return false;
    }

    scots_controller_.clear();
    std::string line;
    int line_count = 0;
    
    // Skip header if present
    std::getline(file, line);
    
    while (std::getline(file, line)) {
      line_count++;
      std::stringstream ss(line);
      std::string token;
      std::vector<double> values;
      
      // Parse comma-separated values
      while (std::getline(ss, token, ',')) {
        try {
          values.push_back(std::stod(token));
        } catch (const std::exception& e) {
          RCLCPP_WARN(get_node()->get_logger(), "Failed to parse line %d: %s", line_count, line.c_str());
          continue;
        }
      }
      
      // Expect 6 values: x, y, z, vx, vy, vz
      if (values.size() == 6) {
        scots_controller_.emplace_back(values[0], values[1], values[2], 
                                       values[3], values[4], values[5]);
      } else {
        RCLCPP_WARN(get_node()->get_logger(), "Line %d has incorrect number of values: %zu (expected 6)", 
                    line_count, values.size());
      }
    }
    
    file.close();
    
    RCLCPP_INFO(get_node()->get_logger(), "Loaded %zu states from SCOTS controller", scots_controller_.size());
    
    if (scots_controller_.empty()) {
      RCLCPP_ERROR(get_node()->get_logger(), "SCOTS controller is empty!");
      return false;
    }
    
    return true;
  }

  bool ScotsVCZExampleController::find_scots_velocity(const Eigen::Vector3d& current_position, 
                                                      Eigen::Vector3d& velocity) {
    if (scots_controller_.empty()) {
      return false;
    }
    
    // Find the closest state in the SCOTS controller
    double min_distance = std::numeric_limits<double>::max();
    int closest_idx = -1;
    
    for (size_t i = 0; i < scots_controller_.size(); ++i) {
      const auto& state = scots_controller_[i];
      double dx = state.x - current_position.x();
      double dy = state.y - current_position.y();
      double dz = state.z - current_position.z();
      double distance = std::sqrt(dx*dx + dy*dy + dz*dz);
      
      if (distance < min_distance) {
        min_distance = distance;
        closest_idx = i;
      }
    }
    
    // Check if the closest state is within tolerance
    if (min_distance > position_tolerance_) {
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                          "No matching state found. Current: [%.3f, %.3f, %.3f], Closest distance: %.4f m",
                          current_position.x(), current_position.y(), current_position.z(), min_distance);
      return false;
    }
    
    // Found a matching state
    const auto& matched_state = scots_controller_[closest_idx];
    velocity << matched_state.vx, matched_state.vy, matched_state.vz;
    
    RCLCPP_DEBUG(get_node()->get_logger(), 
                 "Matched state [%.3f, %.3f, %.3f] -> velocity [%.3f, %.3f, %.3f]",
                 matched_state.x, matched_state.y, matched_state.z,
                 matched_state.vx, matched_state.vy, matched_state.vz);
    
    return true;
  }

Eigen::Affine3d ScotsVCZExampleController::forward_kinematics(const std::vector<double>& joint_positions) {
    // ⚠️ NOTE: This is a conceptual implementation using standard DH parameters.
    // In a real ROS 2 environment, you should use the KDL or libfranka model.
    
    // Check for correct number of joints
    if (joint_positions.size() != num_joints_) {
        RCLCPP_ERROR(get_node()->get_logger(), "Joint positions vector size mismatch for FK.");
        return Eigen::Affine3d::Identity();
    }

    // Standard DH Parameters for the Franka Panda/FR3 (in meters and radians)
    // Link 1 (joint 1) to Link 7 (joint 7)
    // DH Parameters: [alpha (rad), a (m), d (m)]
    // We use the common 7-link parameters (ignoring the hand/gripper for simplicity)
    const std::vector<double> alpha = {
        0.0, -M_PI_2, M_PI_2, M_PI_2, -M_PI_2, M_PI_2, M_PI_2
    };
    const std::vector<double> a = {
        0.0, 0.0, 0.0, 0.0825, -0.0825, 0.0, 0.088
    };
    const std::vector<double> d = {
        0.333, 0.0, 0.316, 0.0, 0.384, 0.0, 0.0
    };

    // Base transformation (World to Frame 0)
    Eigen::Affine3d T_total = Eigen::Affine3d::Identity();

    // Iterate through all 7 joints
    for (int i = 0; i < num_joints_; ++i) {
        double current_q = joint_positions[i];
        double current_alpha = alpha[i];
        double current_a = a[i];
        double current_d = d[i];

        // Homogeneous Transformation Matrix (T_i-1_to_i) using Modified DH
        // [ cos(q)  -sin(q)sin(alpha)  sin(q)cos(alpha)  a*cos(q) ]
        // [ sin(q)   cos(q)cos(alpha) -cos(q)sin(alpha)  a*sin(q) ]
        // [ 0        sin(alpha)        cos(alpha)       d        ]
        // [ 0        0                 0                1        ]
        
        Eigen::Matrix4d Ti;
        
        // Define common terms
        double cq = std::cos(current_q);
        double sq = std::sin(current_q);
        double ca = std::cos(current_alpha);
        double sa = std::sin(current_alpha);

        // Populate the transformation matrix Ti
        Ti << cq, -sq * ca,  sq * sa, current_a * cq,
              sq,  cq * ca, -cq * sa, current_a * sq,
              0.0,    sa,       ca,      current_d,
              0.0,   0.0,      0.0,        1.0;

        // Concatenate the transformations: T_total = T_total * Ti
        Eigen::Matrix4d T_matrix = T_total.matrix() * Ti;
        T_total = Eigen::Affine3d(T_matrix);
    }
    
    // T_total now holds the full 4x4 transformation matrix from the base frame 
    // to the end-effector frame (Frame 7).
    
    // Optional: Add the transformation for the tool/flange to the end-effector tip
    // If you need the *exact* TCP, you might need an extra constant transformation T_7_EE.
    // T_total = T_total * T_7_EE; 

    return T_total;
}

  Eigen::MatrixXd ScotsVCZExampleController::compute_jacobian(const std::vector<double>& joint_positions) {
    // Compute geometric Jacobian using numerical differentiation
    // In production, use analytical Jacobian or robot model
    
    Eigen::MatrixXd jacobian(3, 7);  // 3 DOF (position only) x 7 joints
    
    const double delta = 1e-6;  // Small perturbation for numerical differentiation
    
    Eigen::Affine3d T0 = forward_kinematics(joint_positions);
    Eigen::Vector3d p0 = T0.translation();
    
    for (int i = 0; i < num_joints_; ++i) {
      std::vector<double> q_perturbed = joint_positions;
      q_perturbed[i] += delta;
      
      Eigen::Affine3d T_perturbed = forward_kinematics(q_perturbed);
      Eigen::Vector3d p_perturbed = T_perturbed.translation();
      
      // Numerical derivative: dP/dq_i
      Eigen::Vector3d dp = (p_perturbed - p0) / delta;
      jacobian.col(i) = dp;
    }
    
    return jacobian;
  }

  double ScotsVCZExampleController::sigmoid(double x) {
    // Sigmoid function: ψ(x) = 1 / (1 + e^(-x))
    // Using tanh for numerical stability: tanh(x/2)/2 + 0.5
    return std::tanh(x / 2.0) / 2.0 + 0.5;
  }

  bool ScotsVCZExampleController::is_goal_reached(const Eigen::Vector3d& position) {
    return (position.x() >= goal_region_[0] && position.x() <= goal_region_[1] &&
            position.y() >= goal_region_[2] && position.y() <= goal_region_[3] &&
            position.z() >= goal_region_[4] && position.z() <= goal_region_[5]);
  }

  bool ScotsVCZExampleController::is_position_reachable(const Eigen::Vector3d& position) {
    return (position.x() >= workspace_limits_[0] && position.x() <= workspace_limits_[1] &&
            position.y() >= workspace_limits_[2] && position.y() <= workspace_limits_[3] &&
            position.z() >= workspace_limits_[4] && position.z() <= workspace_limits_[5]);
  }

  ScotsVCZExampleController::Vector7d ScotsVCZExampleController::compute_scots_torque(
      const Eigen::Vector3d& task_velocity,
      const std::vector<double>& joint_positions,
      const std::vector<double>& joint_velocities) {
    
    // Get Jacobian
    Eigen::MatrixXd J = compute_jacobian(joint_positions);
    
    // Compute pseudo-inverse of Jacobian (for redundant manipulator)
    Eigen::MatrixXd J_pinv = J.completeOrthogonalDecomposition().pseudoInverse();
    
    // Convert to Eigen vectors
    Vector7d omega;  // Current joint velocities
    for (int i = 0; i < num_joints_; ++i) {
      omega(i) = joint_velocities[i];
    }
    
    // Compute required joint velocities: J^(-1) * u
    Vector7d required_joint_vel = J_pinv * task_velocity;
    
    // Compute velocity error: ω - J^(-1)u
    Vector7d vel_error = omega - required_joint_vel;
    
    // Apply control law: τ_i = -τ̄_i * ψ((ω - J^(-1)u) / ρ)
    Vector7d tau_command;
    bool torque_limit_exceeded = false;
    
    for (int i = 0; i < num_joints_; ++i) {
      double normalized_error = vel_error(i) / rho_;
      double psi = sigmoid(normalized_error);
      tau_command(i) = -max_joint_torques_(i) * psi;
      
      // Check if commanded torque exceeds limits
      if (std::abs(tau_command(i)) > max_joint_torques_(i) * 0.95) {
        torque_limit_exceeded = true;
      }
    }
    
    if (torque_limit_exceeded) {
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
                          "Calculated torques approaching limits! Joint velocities may be too high.");
    }
    
    // Log for debugging
    RCLCPP_DEBUG(get_node()->get_logger(), 
                 "Task vel: [%.3f, %.3f, %.3f] -> Joint vel error: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                 task_velocity.x(), task_velocity.y(), task_velocity.z(),
                 vel_error(0), vel_error(1), vel_error(2), vel_error(3), 
                 vel_error(4), vel_error(5), vel_error(6));
    
    return tau_command;
  }

  controller_interface::return_type ScotsVCZExampleController::update(
      const rclcpp::Time& /*time*/, 
      const rclcpp::Duration& period) {
    
    update_counter_++;
    
    // Update joint states
    update_joint_states();
    
    // Compute current end-effector position
    Eigen::Affine3d current_transform = forward_kinematics(joint_positions_current_);
    current_ee_position_ = current_transform.translation();
    
    // Check if goal reached
    if (is_goal_reached(current_ee_position_)) {
      if (!goal_reached_) {
        RCLCPP_INFO(get_node()->get_logger(), 
                    "Goal region reached! Position: [%.3f, %.3f, %.3f]",
                    current_ee_position_.x(), current_ee_position_.y(), current_ee_position_.z());
        goal_reached_ = true;
      }
      
      // Send zero torques when goal is reached
      for (int i = 0; i < num_joints_; i++) {
        command_interfaces_[i].set_value(0.0);
      }
      return controller_interface::return_type::OK;
    }
    
    // Check if position is reachable (within workspace)
    if (!is_position_reachable(current_ee_position_)) {
      RCLCPP_ERROR(get_node()->get_logger(), 
                   "End-effector position [%.3f, %.3f, %.3f] is outside workspace limits! Stopping controller.",
                   current_ee_position_.x(), current_ee_position_.y(), current_ee_position_.z());
      
      // Send zero torques
      for (int i = 0; i < num_joints_; i++) {
        command_interfaces_[i].set_value(0.0);
      }
      return controller_interface::return_type::ERROR;
    }
    
    // Look up velocity from SCOTS controller
    Eigen::Vector3d scots_velocity;
    bool velocity_found = find_scots_velocity(current_ee_position_, scots_velocity);
    
    if (!velocity_found) {
      // Use last valid velocity if lookup fails
      scots_velocity = last_valid_velocity_;
      velocity_lookup_failures_++;
      
      if (velocity_lookup_failures_ > MAX_LOOKUP_FAILURES) {
        RCLCPP_ERROR(get_node()->get_logger(), 
                     "Failed to find matching velocity for %d consecutive updates. Stopping controller.",
                     MAX_LOOKUP_FAILURES);
        
        // Send zero torques
        for (int i = 0; i < num_joints_; i++) {
          command_interfaces_[i].set_value(0.0);
        }
        return controller_interface::return_type::ERROR;
      }
      
      RCLCPP_DEBUG(get_node()->get_logger(), "Using last valid velocity [%.3f, %.3f, %.3f]",
                   scots_velocity.x(), scots_velocity.y(), scots_velocity.z());
    } else {
      // Valid velocity found, reset failure counter and store
      last_valid_velocity_ = scots_velocity;
      velocity_lookup_failures_ = 0;
    }
    
    // Compute torque command using SCOTS control law
    Vector7d tau_command = compute_scots_torque(scots_velocity, joint_positions_current_, 
                                                joint_velocities_current_);
    
    // Check for excessive torques
    bool excessive_torque = false;
    for (int i = 0; i < num_joints_; ++i) {
      if (std::abs(tau_command(i)) > max_joint_torques_(i)) {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "Calculated torque for joint %d (%.3f Nm) exceeds limit (%.3f Nm)!",
                     i + 1, tau_command(i), max_joint_torques_(i));
        excessive_torque = true;
      }
    }
    
    if (excessive_torque) {
      // Clip torques to limits
      for (int i = 0; i < num_joints_; ++i) {
        tau_command(i) = std::clamp(tau_command(i), -max_joint_torques_(i), max_joint_torques_(i));
      }
      RCLCPP_WARN(get_node()->get_logger(), "Torques clipped to joint limits.");
    }
    
    // Apply torque commands
    for (int i = 0; i < num_joints_; i++) {
      command_interfaces_[i].set_value(tau_command(i));
    }
    
    // Periodic status logging
    if (update_counter_ % 1000 == 0) {
      RCLCPP_INFO(get_node()->get_logger(), 
                  "Status - EE pos: [%.3f, %.3f, %.3f], Velocity: [%.3f, %.3f, %.3f]",
                  current_ee_position_.x(), current_ee_position_.y(), current_ee_position_.z(),
                  scots_velocity.x(), scots_velocity.y(), scots_velocity.z());
    }
    
    return controller_interface::return_type::OK;
  }

  CallbackReturn ScotsVCZExampleController::on_init() {
    try {
      auto_declare<std::string>("arm_id", "fr3");
      auto_declare<bool>("load_gripper", false);
      auto_declare<std::string>("scots_controller_path", "");
      auto_declare<double>("rho", 1.0);
      auto_declare<double>("position_tolerance", 0.01);
      auto_declare<std::vector<double>>("goal_region", std::vector<double>{0.5, 0.6, 0.5, 0.6, 0.5, 0.6});
      auto_declare<std::vector<double>>("workspace_limits", std::vector<double>{0.3, 0.8, -0.4, 0.4, 0.1, 0.9});
    } catch (const std::exception& e) {
      fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
      return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
  }

  bool ScotsVCZExampleController::assign_parameters() {
    arm_id_ = get_node()->get_parameter("arm_id").as_string();
    is_gripper_loaded_ = get_node()->get_parameter("load_gripper").as_bool();
    scots_controller_path_ = get_node()->get_parameter("scots_controller_path").as_string();
    rho_ = get_node()->get_parameter("rho").as_double();
    position_tolerance_ = get_node()->get_parameter("position_tolerance").as_double();
    goal_region_ = get_node()->get_parameter("goal_region").as_double_array();
    workspace_limits_ = get_node()->get_parameter("workspace_limits").as_double_array();
    
    // Validate parameters
    if (scots_controller_path_.empty()) {
      RCLCPP_FATAL(get_node()->get_logger(), "scots_controller_path parameter not set!");
      return false;
    }
    
    if (goal_region_.size() != 6) {
      RCLCPP_FATAL(get_node()->get_logger(), "goal_region must have 6 values [x_min, x_max, y_min, y_max, z_min, z_max]");
      return false;
    }
    
    if (workspace_limits_.size() != 6) {
      RCLCPP_FATAL(get_node()->get_logger(), "workspace_limits must have 6 values");
      return false;
    }
    
    // Set max joint torques for FR3 (in Nm)
    max_joint_torques_ << 87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0;
    
    RCLCPP_INFO(get_node()->get_logger(), "Parameters loaded successfully");
    RCLCPP_INFO(get_node()->get_logger(), "SCOTS controller path: %s", scots_controller_path_.c_str());
    RCLCPP_INFO(get_node()->get_logger(), "Goal region: [%.2f-%.2f, %.2f-%.2f, %.2f-%.2f]",
                goal_region_[0], goal_region_[1], goal_region_[2], 
                goal_region_[3], goal_region_[4], goal_region_[5]);
    
    return true;
  }

  CallbackReturn ScotsVCZExampleController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
    
    if (!assign_parameters()) {
      return CallbackReturn::FAILURE;
    }

    // Load SCOTS controller from CSV
    if (!load_scots_controller(scots_controller_path_)) {
      RCLCPP_FATAL(get_node()->get_logger(), "Failed to load SCOTS controller!");
      return CallbackReturn::FAILURE;
    }
    
    controller_loaded_ = true;

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

  CallbackReturn ScotsVCZExampleController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
    
    initialization_flag_ = true;
    goal_reached_ = false;
    update_counter_ = 0;
    velocity_lookup_failures_ = 0;
    
    joint_positions_current_.resize(num_joints_, 0.0);
    joint_velocities_current_.resize(num_joints_, 0.0);
    
    // Initialize last valid velocity to zero
    last_valid_velocity_.setZero();

    RCLCPP_INFO(get_node()->get_logger(), "SCOTS VCZ Controller activated!");
    
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn ScotsVCZExampleController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
    RCLCPP_INFO(get_node()->get_logger(), "SCOTS VCZ Controller deactivated");
    return CallbackReturn::SUCCESS;
  }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::ScotsVCZExampleController, 
                       controller_interface::ControllerInterface)