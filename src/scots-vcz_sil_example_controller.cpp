#include <franka_example_controllers/scots_vcz_sil_example_controller.hpp>

namespace franka_example_controllers {

  controller_interface::InterfaceConfiguration ScotsVczSILExampleController::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (int i = 1; i <= num_joints_; ++i) {
      config.names.push_back(arm_id_ + "_joint" + to_string(i) + "/effort");
    }
    return config;
  }

  controller_interface::InterfaceConfiguration ScotsVczSILExampleController::state_interface_configuration() const {
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

  void ScotsVczSILExampleController::update_joint_states() {
    for (auto i = 0; i < num_joints_; ++i) {
      const auto& position_interface = state_interfaces_.at(i);
      const auto& velocity_interface = state_interfaces_.at(num_joints_ + i);

      assert(position_interface.get_interface_name() == "position");
      assert(velocity_interface.get_interface_name() == "velocity");

      joint_positions_current_[i] = position_interface.get_value();
      joint_velocities_current_[i] = velocity_interface.get_value();
    }
  }

  Vector3d ScotsVczSILExampleController::compute_new_position() {
    static bool trajectory_loaded = false;
    static vector<Vector3d> position_steps;
    static double segment_elapsed = 0.0;
    static constexpr double segment_duration = 0.01;
    double dt = 0.001;
    static const string position_filename = "~/ros2_ws/src/scots_vcz_franka/final_examples/example_1 files/example_1_final_trajectory.csv";
    // static const string position_filename = "~/ros2_ws/src/scots_vcz_franka/final_examples/example_1 files/example_2_final_trajectory.csv";

    if (!trajectory_loaded) {
      ifstream position_file(position_filename);
      if (!position_file.is_open()) {
        RCLCPP_ERROR(get_node()->get_logger(), "Error: Could not open trajectory position file: [%s]", position_filename.c_str());
        trajectory_loaded = true;
        return position_;
      }

      string position_line;
      if (getline(position_file, position_line)) {
      }

      while (getline(position_file, position_line)) {
        stringstream ss(position_line);
        string cell;
        vector<double> coords;

        while (getline(ss, cell, ',')) {
          try {
            coords.push_back(stod(cell));
          } catch (const exception& e) {
            RCLCPP_ERROR(get_node()->get_logger(), "Warning: Malformed data in CSV line. Skipping.");
            coords.clear();
            break;
          }
        }

        if (coords.size() == 3) {
            position_steps.emplace_back(coords[0], coords[1], coords[2]);
        }
      }

      if (position_steps.empty()) {
          RCLCPP_ERROR(get_node()->get_logger(), "Error: Trajectory file was empty or contained no valid data.");
      } else {
          RCLCPP_INFO(get_node()->get_logger(), "Trajectory loaded successfully with [%ld] steps.", position_steps.size());
      }

      trajectory_loaded = true;
    }

    if (position_steps.empty()) return position_;

    size_t next_index = (position_index + 1) % position_steps.size();

    const Vector3d& p0 = position_steps[position_index];
    const Vector3d& p1 = position_steps[next_index];

    segment_elapsed += dt;
    double fraction = segment_elapsed / segment_duration;
    fraction = clamp(fraction, 0.0, 1.0);
    Vector3d new_position_command = p0 + fraction * (p1 - p0);

    if (fraction >= 1.0) {
      segment_elapsed = 0.0;
      position_index = next_index;
    }

    return new_position_command;
  }

  shared_ptr<moveit_msgs::srv::GetPositionIK::Request> ScotsVczSILExampleController::create_ik_service_request(const Vector3d& position, const Quaterniond& orientation, const vector<double>& joint_positions_current) {
    auto service_request = make_shared<moveit_msgs::srv::GetPositionIK::Request>();

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

  Vector7d ScotsVczSILExampleController::compute_torque_command(const Vector7d& joint_positions_desired, const Vector7d& joint_positions_current, const Vector7d& joint_velocities_current) {

    const double kAlpha = 0.99;
    dq_filtered_ = (1 - kAlpha) * dq_filtered_ + kAlpha * joint_velocities_current;

    Vector7d taub = {10, 10 ,10 ,10, 5 ,5, 2};
    Vector7d vb = {2, 2, 2, 2, 2.5, 1.5, 1}; 

    Vector7d psi1 = {0, 0, 0, 0, 0, 0, 0};
    Vector7d psi2 = {0, 0, 0, 0, 0, 0, 0};

    double sigma = 0.1;
    double rho = 1;

    for (int i = 0; i < 7; ++i) {
      double e = (joint_positions_current[i] - joint_positions_desired[i])/sigma;

      psi1[i] = pow((tanh(e)),1);
      psi2[i] = tanh((dq_filtered_[i] + vb[i]*psi1[i])/rho);
    }

    Vector7d tau_d_calculated = -taub.cwiseProduct(psi2);

    return tau_d_calculated;
  }

  controller_interface::return_type ScotsVczSILExampleController::update(const rclcpp::Time& /*time*/, const rclcpp::Duration& period) {
    
    if (initialization_flag_) {
      update_joint_states();
      
      elapsed_time_ = 0.0;
      initialization_flag_ = false;
    } else {
      elapsed_time_ += period.seconds();
    }
    
    update_joint_states();

    Vector3d new_position = compute_new_position();
    Quaterniond new_orientation = {0, 1, 0, 0};

    auto service_request = create_ik_service_request(new_position, new_orientation, joint_positions_current_);

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

    Vector7d joint_positions_desired_eigen(joint_positions_desired_.data());
    Vector7d joint_positions_current_eigen(joint_positions_current_.data());
    Vector7d joint_velocities_current_eigen(joint_velocities_current_.data());

    auto tau_d_calculated = compute_torque_command(
        joint_positions_desired_eigen, joint_positions_current_eigen, joint_velocities_current_eigen);

    for (int i = 0; i < num_joints_; i++) {
      command_interfaces_[i].set_value(tau_d_calculated(i));
    }

    return controller_interface::return_type::OK;
  }

  CallbackReturn ScotsVczSILExampleController::on_init() {
    try {
      auto_declare<string>("arm_id", "");
      auto_declare<vector<double>>("k_gains", {});
      auto_declare<vector<double>>("d_gains", {});
      auto_declare<bool>("load_gripper", true);
    } catch (const exception& e) {
      fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
      return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
  }

  bool ScotsVczSILExampleController::assign_parameters() {
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

    return true;
  }

  CallbackReturn ScotsVczSILExampleController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
    
    if (!assign_parameters()) {
      return CallbackReturn::FAILURE;
    }

    compute_ik_client_ = get_node()->create_client<moveit_msgs::srv::GetPositionIK>("compute_ik");

    if (!compute_ik_client_->wait_for_service(chrono::seconds(5))) {
      RCLCPP_ERROR(get_node()->get_logger(), 
                  "IK service not available after waiting. Make sure MoveIt is running.");
      return CallbackReturn::ERROR;
    }
    
    RCLCPP_INFO(get_node()->get_logger(), "IK service connected.");

    auto parameters_client =
        make_shared<rclcpp::AsyncParametersClient>(get_node(), "robot_state_publisher");
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

  CallbackReturn ScotsVczSILExampleController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
    
    initialization_flag_ = true;
    elapsed_time_ = 0.0;
    dq_filtered_.setZero();
    
    joint_positions_desired_.resize(num_joints_, 0.0);
    joint_positions_current_.resize(num_joints_, 0.0);
    joint_velocities_current_.resize(num_joints_, 0.0);

    return CallbackReturn::SUCCESS;
  }

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::ScotsVczSILExampleController, controller_interface::ControllerInterface)
