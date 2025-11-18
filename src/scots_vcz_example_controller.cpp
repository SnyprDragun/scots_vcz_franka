#include <franka_example_controllers/scots_vcz_example_controller.hpp>

namespace franka_example_controllers {

  controller_interface::InterfaceConfiguration ScotsVCZExampleController::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (int i = 1; i <= num_joints_; ++i) {
      config.names.push_back(arm_id_ + "_joint" + to_string(i) + "/effort");
    }
    return config;
  }

  controller_interface::InterfaceConfiguration ScotsVCZExampleController::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (int i = 1; i <= num_joints_; ++i) {
      config.names.push_back(arm_id_ + "_joint" + to_string(i) + "/position");
    }
    for (int i = 1; i <= num_joints_; ++i) {
      config.names.push_back(arm_id_ + "_joint" + to_string(i) + "/velocity");
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

  bool ScotsVCZExampleController::load_scots_controller(const string& csv_file_path) {
    ifstream file(csv_file_path);
    if (!file.is_open()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to open SCOTS controller file: %s", csv_file_path.c_str());
      return false;
    }

    scots_controller_.clear();
    string line;
    int line_count = 0;

    getline(file, line);
    while (getline(file, line)) {
      line_count++;
      stringstream ss(line);
      string token;
      vector<double> values;

      while (getline(ss, token, ',')) {
        try {
          values.push_back(stod(token));
        } catch (const exception& e) {
          RCLCPP_WARN(get_node()->get_logger(), "Failed to parse line %d: %s", line_count, line.c_str());
          continue;
        }
      }

      if (values.size() == 6) {
        scots_controller_.emplace_back(values[0], values[1], values[2], values[3], values[4], values[5]);
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

  Vector3d ScotsVCZExampleController::find_scots_velocity(const Vector3d& current_position) {
    if (scots_controller_.empty()) {
      return Vector3d().Zero();
    }
    Vector3d velocity;
    double min_distance = numeric_limits<double>::max();
    int closest_idx = -1;
    
    for (size_t i = 0; i < scots_controller_.size(); ++i) {
      const auto& state = scots_controller_[i];
      double dx = state.x - current_position.x();
      double dy = state.y - current_position.y();
      double dz = state.z - current_position.z();
      double distance = sqrt(dx*dx + dy*dy + dz*dz);
      
      if (distance < min_distance) {
        min_distance = distance;
        closest_idx = i;
      }
    }

    if (min_distance > position_tolerance_) {
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                          "No matching state found. Current: [%.3f, %.3f, %.3f], Closest distance: %.4f m",
                          current_position.x(), current_position.y(), current_position.z(), min_distance);
      return Vector3d().Zero();
    }

    const auto& matched_state = scots_controller_[closest_idx];
    Eigen::Vector3d matched_state_position;
    matched_state_position << matched_state.x, matched_state.y, matched_state.z;
    double lambda_ = 0.1;
    velocity = {matched_state.vx, matched_state.vy, matched_state.vz};
    // RCLCPP_INFO(get_node()->get_logger(), 
    //              "Matched state [%.3f, %.3f, %.3f] -> velocity [%.3f, %.3f, %.3f] and Current state [%.3f, %.3f, %.3f]",
    //              matched_state.x, matched_state.y, matched_state.z,
    //              matched_state.vx, matched_state.vy, matched_state.vz,
    //              current_position.x(), current_position.y(), current_position.z());
    RCLCPP_DEBUG(get_node()->get_logger(), 
                 "Matched state [%.3f, %.3f, %.3f] -> velocity [%.3f, %.3f, %.3f]",
                 matched_state.x, matched_state.y, matched_state.z,
                 matched_state.vx, matched_state.vy, matched_state.vz);

    velocity = (current_position - matched_state_position) / lambda_;
    velocity = velocity.array().unaryExpr([this](double x) { return this->sigmoid(x); }).array() * max_velocity_.array();
    velocity = velocity.cwiseMin(max_velocity_).cwiseMax(-max_velocity_);

    return velocity;
  }

  Affine3d ScotsVCZExampleController::forward_kinematics(const vector<double>& joint_positions) {
    if (joint_positions.size() != num_joints_) {
        RCLCPP_ERROR(get_node()->get_logger(), "Joint positions vector size mismatch for FK.");
        return Affine3d::Identity();
    }

    const vector<double> alpha = {0.0, -M_PI_2, M_PI_2, M_PI_2, -M_PI_2, M_PI_2, M_PI_2};
    const vector<double> a = {0.0, 0.0, 0.0, 0.0825, -0.0825, 0.0, 0.088};
    const vector<double> d = {0.333, 0.0, 0.316, 0.0, 0.384, 0.0, 0.0};

    Affine3d T_total = Affine3d::Identity();

    for (int i = 0; i < num_joints_; ++i) {
        double current_q = joint_positions[i];
        double current_alpha = alpha[i];
        double current_a = a[i];
        double current_d = d[i];

        Matrix4d Ti;
        
        double cq = cos(current_q);
        double sq = sin(current_q);
        double ca = cos(current_alpha);
        double sa = sin(current_alpha);

        Ti << cq, -sq * ca,  sq * sa, current_a * cq,
              sq,  cq * ca, -cq * sa, current_a * sq,
              0.0,    sa,       ca,      current_d,
              0.0,   0.0,      0.0,        1.0;

        Matrix4d T_matrix = T_total.matrix() * Ti;
        T_total = Affine3d(T_matrix);
    }
    
    // T_total now holds the full 4x4 transformation matrix from the base frame to the end-effector frame (Frame 7).
    
    // Optional: Add the transformation for the tool/flange to the end-effector tip
    // If you need the *exact* TCP, you might need an extra constant transformation T_7_EE.
    // T_total = T_total * T_7_EE; 

    return T_total;
}

  MatrixXd ScotsVCZExampleController::compute_jacobian(const vector<double>& joint_positions) {
    MatrixXd jacobian(3, 7);
    const double delta = 1e-6;
    Affine3d T0 = forward_kinematics(joint_positions);
    Vector3d p0 = T0.translation();
    
    for (int i = 0; i < num_joints_; ++i) {
      vector<double> q_perturbed = joint_positions;
      q_perturbed[i] += delta;
      
      Affine3d T_perturbed = forward_kinematics(q_perturbed);
      Vector3d p_perturbed = T_perturbed.translation();

      Vector3d dp = (p_perturbed - p0) / delta;
      jacobian.col(i) = dp;
    }

    return jacobian;
  }

  ScotsVCZExampleController::Vector7d ScotsVCZExampleController::calculate_dls_joint_velocity(
    const MatrixXd& jacobian,
    const Vector3d& cartesian_velocity) {

    const double DAMPING_LAMBDA = 0.05; 

    // Check matrix dimensions (J must be 3x7)
    if (jacobian.rows() != 3 || jacobian.cols() != 7) {
        RCLCPP_ERROR(get_node()->get_logger(), "Jacobian matrix size is not 3x7.");
        return Vector7d::Zero(7);
    }

    // 1. Compute J * J^T (a 3x3 matrix)
    Matrix3d JJT = jacobian * jacobian.transpose();

    // 2. Compute the DLS term: (J * J^T + lambda^2 * I)
    //    We useMatrix3d::Identity() for the 3x3 identity matrix.
    Matrix3d DLS_term = JJT + DAMPING_LAMBDA * DAMPING_LAMBDA *Matrix3d::Identity();

    // 3. Compute the DLS Pseudo-Inverse (J_DLS_pinv): J^T * (DLS_term)^-1
    //    We use .inverse() for the 3x3 matrix inversion.
    MatrixXd J_DLS_pinv = jacobian.transpose() * DLS_term.inverse();

    // 4. Solve for joint velocity (q_dot): q_dot = J_DLS_pinv * x_dot
    Vector7d q_dot = J_DLS_pinv * cartesian_velocity;

    return q_dot;
  }

  double ScotsVCZExampleController::sigmoid(double x) {
    return tanh(x / 2.0) / 2.0 + 0.5;
  }

  bool ScotsVCZExampleController::is_goal_reached(const Vector3d& position) {
    return (position.x() >= goal_region_[0] && position.x() <= goal_region_[1] &&
            position.y() >= goal_region_[2] && position.y() <= goal_region_[3] &&
            position.z() >= goal_region_[4] && position.z() <= goal_region_[5]);
  }

  bool ScotsVCZExampleController::is_position_reachable(const Vector3d& position) {
    return (position.x() >= workspace_limits_[0] && position.x() <= workspace_limits_[1] &&
            position.y() >= workspace_limits_[2] && position.y() <= workspace_limits_[3] &&
            position.z() >= workspace_limits_[4] && position.z() <= workspace_limits_[5]);
  }

  ScotsVCZExampleController::Vector7d ScotsVCZExampleController::compute_scots_torque(
      const Vector3d& task_velocity,
      const vector<double>& joint_positions,
      const vector<double>& joint_velocities) {

    MatrixXd J = compute_jacobian(joint_positions);
    MatrixXd J_pinv = J.completeOrthogonalDecomposition().pseudoInverse();

    Vector7d omega;
    for (int i = 0; i < num_joints_; ++i) {
      omega(i) = joint_velocities[i];
    }

    /* 
    Compute required joint velocities: J^(-1) * u
    Compute velocity error: ω - J^(-1)u
    Apply control law: τ_i = -τ̄_i * ψ((ω - J^(-1)u) / ρ)
    */

    // Vector7d required_joint_vel = calculate_dls_joint_velocity(J, task_velocity);
    Vector7d required_joint_vel = J_pinv * task_velocity;
    Vector7d vel_error = omega - required_joint_vel;
    Vector7d tau_command;
    bool torque_limit_exceeded = false;

    for (int i = 0; i < num_joints_; ++i) {
      double normalized_error = vel_error(i) / rho_;
      double psi = sigmoid(normalized_error);
      tau_command(i) = -max_joint_torques_(i) * psi;

      if (abs(tau_command(i)) > max_joint_torques_(i) * 0.95) {
        torque_limit_exceeded = true;
      }
    }

    if (torque_limit_exceeded) {
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000, "Calculated torques approaching limits! Joint velocities may be too high.");
    }

    RCLCPP_DEBUG(get_node()->get_logger(), 
                 "Task vel: [%.3f, %.3f, %.3f] -> Joint vel error: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                 task_velocity.x(), task_velocity.y(), task_velocity.z(), vel_error(0), vel_error(1), vel_error(2), 
                 vel_error(3), vel_error(4), vel_error(5), vel_error(6));

    if (update_counter_ % 100 == 0) {
      RCLCPP_INFO(get_node()->get_logger(), "\n\n\nSCOTS Task velocity: [%.3f, %.3f, %.3f],\nCurrent Joint Velocity: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f], \nCurrent Joint Position: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f],\nRequired Joint Velocity: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f],\nComputed Torque Command: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]", 
      task_velocity.x(), task_velocity.y(), task_velocity.z(),
      joint_velocities[0], joint_velocities[1], joint_velocities[2], joint_velocities[3], joint_velocities[4], joint_velocities[5], joint_velocities[6],
      joint_positions[0], joint_positions[1], joint_positions[2], joint_positions[3], joint_positions[4], joint_positions[5], joint_positions[6],
      required_joint_vel(0), required_joint_vel(1), required_joint_vel(2), required_joint_vel(3), required_joint_vel(4), required_joint_vel(5), required_joint_vel(6),
      tau_command(0), tau_command(1), tau_command(2), tau_command(3), tau_command(4), tau_command(5), tau_command(6));
    }

    return tau_command;
  }

  controller_interface::return_type ScotsVCZExampleController::update(
      const rclcpp::Time&, 
      const rclcpp::Duration& period) {
    
    update_counter_++;
    update_joint_states();
    Affine3d current_transform = forward_kinematics(joint_positions_current_);
    current_ee_position_ = current_transform.translation();

    if (is_goal_reached(current_ee_position_)) {
      if (!goal_reached_) {
        RCLCPP_INFO(get_node()->get_logger(), 
                    "Goal region reached! Position: [%.3f, %.3f, %.3f]",
                    current_ee_position_.x(), current_ee_position_.y(), current_ee_position_.z());
        goal_reached_ = true;
      }

      for (int i = 0; i < num_joints_; i++) {
        command_interfaces_[i].set_value(0.0);
      }
      return controller_interface::return_type::OK;
    }

    if (!is_position_reachable(current_ee_position_)) {
      // RCLCPP_ERROR(get_node()->get_logger(), "End-effector position [%.3f, %.3f, %.3f] is outside workspace limits! Stopping controller.", current_ee_position_.x(), current_ee_position_.y(), current_ee_position_.z());

      for (int i = 0; i < num_joints_; i++) {
        command_interfaces_[i].set_value(0.0);
      }
      // return controller_interface::return_type::ERROR;
    }

    Vector3d scots_velocity;
    bool velocity_found = false;
    scots_velocity = find_scots_velocity(current_ee_position_);

    if (scots_velocity == Vector3d()) {
      velocity_found = true;
      // RCLCPP_INFO(get_node()->get_logger(), "FOUND");
    }
    // RCLCPP_INFO(get_node()->get_logger(), "CHECK [%.3f, %.3f, %.3f]", scots_velocity.x(), scots_velocity.y(), scots_velocity.z());
    if (!velocity_found) {
      scots_velocity = last_valid_velocity_;
      velocity_lookup_failures_++;
      
      if (velocity_lookup_failures_ > MAX_LOOKUP_FAILURES) {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to find matching velocity for %d consecutive updates. Stopping controller.", MAX_LOOKUP_FAILURES);

        for (int i = 0; i < num_joints_; i++) {
          command_interfaces_[i].set_value(0.0);
        }
        // return controller_interface::return_type::ERROR;
      }
      // RCLCPP_INFO(get_node()->get_logger(), "Using last valid velocity [%.3f, %.3f, %.3f]", scots_velocity.x(), scots_velocity.y(), scots_velocity.z());
      RCLCPP_DEBUG(get_node()->get_logger(), "Using last valid velocity [%.3f, %.3f, %.3f]", scots_velocity.x(), scots_velocity.y(), scots_velocity.z());
    } else {
      last_valid_velocity_ = {0, 0, 0};
      velocity_lookup_failures_ = 0;
    }

    Vector7d tau_command = compute_scots_torque(scots_velocity, joint_positions_current_, joint_velocities_current_);

    bool excessive_torque = false;
    for (int i = 0; i < num_joints_; ++i) {
      if (abs(tau_command(i)) > max_joint_torques_(i)) {
        // RCLCPP_INFO(get_node()->get_logger(), "Calculated torque for joint %d (%.3f Nm) exceeds limit (%.3f Nm)!", i + 1, tau_command(i), max_joint_torques_(i));
        RCLCPP_ERROR(get_node()->get_logger(), "Calculated torque for joint %d (%.3f Nm) exceeds limit (%.3f Nm)!", i + 1, tau_command(i), max_joint_torques_(i));
        excessive_torque = true;
      }
    }

    if (excessive_torque) {
      for (int i = 0; i < num_joints_; ++i) {
        tau_command(i) = clamp(tau_command(i), -max_joint_torques_(i), max_joint_torques_(i));
      }
      RCLCPP_WARN(get_node()->get_logger(), "Torques clipped to joint limits.");
    }

    for (int i = 0; i < num_joints_; i++) {
      command_interfaces_[i].set_value(tau_command(i));
    }

    // RCLCPP_INFO(get_node()->get_logger(), "CMD INT [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]", 
    //   command_interfaces_[0].get_value(),
    //   command_interfaces_[1].get_value(),
    //   command_interfaces_[2].get_value(),
    //   command_interfaces_[3].get_value(),
    //   command_interfaces_[4].get_value(),
    //   command_interfaces_[5].get_value(),
    //   command_interfaces_[6].get_value());

    if (update_counter_ % 100 == 0) {
      RCLCPP_INFO(get_node()->get_logger(), "\nStatus - EE pos: [%.3f, %.3f, %.3f], Velocity: [%.3f, %.3f, %.3f]", current_ee_position_.x(), current_ee_position_.y(), current_ee_position_.z(), scots_velocity.x(), scots_velocity.y(), scots_velocity.z());
      RCLCPP_INFO(get_node()->get_logger(), "\nCMD TAU [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]", 
      tau_command(0),
      tau_command(1),
      tau_command(2),
      tau_command(3),
      tau_command(4),
      tau_command(5),
      tau_command(6));
    }

    return controller_interface::return_type::OK;
  }

  CallbackReturn ScotsVCZExampleController::on_init() {
    try {
      auto_declare<string>("arm_id", "fr3");
      auto_declare<bool>("load_gripper", false);
      auto_declare<string>("scots_controller_path", "");
      auto_declare<double>("rho", 1.0);
      auto_declare<double>("position_tolerance", 0.01);
      auto_declare<vector<double>>("goal_region", vector<double>{0.5, 0.6, 0.5, 0.6, 0.5, 0.6});
      auto_declare<vector<double>>("workspace_limits", vector<double>{-0.7, 0.7, -0.7, 0.7, -0.7, 0.7});
    } catch (const exception& e) {
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
    max_joint_torques_ << 10.0, 10.0, 10.0, 10.0, 5.0, 5.0, 2.0;
    max_velocity_ << 0.05, 0.05, 0.05;
    
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

    if (!load_scots_controller(scots_controller_path_)) {
      RCLCPP_FATAL(get_node()->get_logger(), "Failed to load SCOTS controller!");
      return CallbackReturn::FAILURE;
    }
    
    controller_loaded_ = true;

    auto parameters_client = make_shared<rclcpp::AsyncParametersClient>(get_node(), "robot_state_publisher");
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
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::ScotsVCZExampleController, controller_interface::ControllerInterface)
