#include <franka_example_controllers/scots_vcz_sil_example_controller.hpp>

namespace franka_example_controllers {

  controller_interface::InterfaceConfiguration ScotsVczHILExampleController::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (int i = 1; i <= num_joints_; ++i) {
      config.names.push_back(arm_id_ + "_joint" + to_string(i) + "/effort");
    }
    return config;
  }

  controller_interface::InterfaceConfiguration ScotsVczHILExampleController::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    config.names = franka_cartesian_pose_->get_state_interface_names();

    for (int i = 1; i <= num_joints_; ++i) {
      config.names.push_back(arm_id_ + "_joint" + to_string(i) + "/position");
    }
    for (int i = 1; i <= num_joints_; ++i) {
      config.names.push_back(arm_id_ + "_joint" + to_string(i) + "/velocity");
    }
    for (int i = 1; i <= num_joints_; ++i) {
      config.names.push_back(arm_id_ + "_joint" + to_string(i) + "/effort");
    }
    for (const auto& franka_robot_model_name : franka_robot_model_->get_state_interface_names()) {
      config.names.push_back(franka_robot_model_name);
    }

    return config;
  }

  void ScotsVczHILExampleController::update_joint_states() {
    for (auto i = 0; i < num_joints_; ++i) {
      const auto& position_interface = state_interfaces_.at(16 + i);
      const auto& velocity_interface = state_interfaces_.at(23 + i);
      const auto& effort_interface = state_interfaces_.at(30 + i);

      joint_positions_current_[i] = position_interface.get_value();
      joint_velocities_current_[i] = velocity_interface.get_value();
      joint_efforts_current_[i] = effort_interface.get_value();
    }
  }

  Vector3d ScotsVczHILExampleController::compute_new_position() {
    static bool trajectory_loaded = false;
    static vector<Vector3d> position_steps;
    static double segment_elapsed = 0.0;
    static constexpr double segment_duration = 0.01;
    double dt = 0.001;

    if (!trajectory_loaded) {
      ifstream file("~/ros2_ws/src/scots_vcz_franka/final_examples/example_1 files/example_1_final_trajectory_gripper.csv");
      ifstream file("~/ros2_ws/src/scots_vcz_franka/final_examples/example_2 files/example_2_final_trajectory_gripper.csv");
      
      string line;
      getline(file, line);

      while (getline(file, line)) {
        stringstream ss(line);
        string cell;
        vector<double> vals;
        while (getline(ss, cell, ',')) vals.push_back(stod(cell));
        if (vals.size() >= 4){
          position_steps.emplace_back(vals[0], vals[1], vals[2]);
          double toggle_double = vals[3];
          int toggle_int = static_cast<int>(toggle_double);
          gripper_toggle_states.push_back(toggle_int);
        } else {
          RCLCPP_WARN(get_node()->get_logger(), YELLOW "Skipping line with fewer than 4 values." RESET);
        }
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

    Vector3d new_position_command =
        p0 + fraction * (p1 - p0);

    if (fraction >= 1.0) {
      segment_elapsed = 0.0;
      position_index = next_index;
    }

    return new_position_command;
  }

  Quaterniond ScotsVczHILExampleController::convert_rpy_to_quaternion(const Vector3d& rpy_angles) {
      AngleAxisd roll_angle(rpy_angles(0), Vector3d::UnitX());
      AngleAxisd pitch_angle(rpy_angles(1), Vector3d::UnitY());
      AngleAxisd yaw_angle(rpy_angles(2), Vector3d::UnitZ());
      Quaterniond q = yaw_angle * pitch_angle * roll_angle;
      return q.normalized();
  }

  shared_ptr<moveit_msgs::srv::GetPositionIK::Request> ScotsVczHILExampleController::create_ik_service_request(const Vector3d& position, const Quaterniond& orientation, const vector<double>& joint_positions_current, const vector<double>& joint_velocities_current, const vector<double>& joint_efforts_current) {
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
    service_request->ik_request.robot_state.joint_state.velocity = joint_velocities_current;
    service_request->ik_request.robot_state.joint_state.effort = joint_efforts_current;

    if (is_gripper_loaded_) {
      service_request->ik_request.ik_link_name = arm_id_ + "_hand_tcp";
    }
    return service_request;
  }

  ScotsVczHILExampleController::Vector7d ScotsVczHILExampleController::compute_torque_command(const Vector7d& joint_positions_desired, const Vector7d& joint_positions_current, const Vector7d& joint_velocities_current) {

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

  controller_interface::return_type ScotsVczHILExampleController::update(const rclcpp::Time& /*time*/, const rclcpp::Duration& period) {
    if (initialization_flag_) {
      initial_robot_time_ = state_interfaces_.back().get_value();
      initialization_flag_ = false;
    } else {
      robot_time_ = state_interfaces_.back().get_value();
    }

    update_joint_states();
    start_time_ = this->get_node()->now();

    Vector3d new_position = compute_new_position();
    Quaterniond new_orientation = {0, 1, 0, 0};

    auto service_request = create_ik_service_request(new_position, new_orientation, joint_positions_current_, joint_velocities_current_, joint_efforts_current_);
    
    using ServiceResponseFuture = rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedFuture;
    auto response_received_callback =
        [&](ServiceResponseFuture future) {
          const auto& response = future.get();

          if (response->error_code.val == response->error_code.SUCCESS) {
            joint_positions_desired_ = response->solution.joint_state.position;
          } else {
            RCLCPP_INFO(get_node()->get_logger(), "Inverse kinematics solution failed.");
          }
        };
    auto result_future_ =
        compute_ik_client_->async_send_request(service_request, response_received_callback);

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

    toggleGripperState();

    return controller_interface::return_type::OK;
  }

  CallbackReturn ScotsVczHILExampleController::on_init() {
    try {
      auto_declare<string>("arm_id", "fr3");
    } catch (const exception& e) {
      fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
      return CallbackReturn::ERROR;
    }

    franka_cartesian_pose_ = make_unique<franka_semantic_components::FrankaCartesianPoseInterface>(franka_semantic_components::FrankaCartesianPoseInterface(k_elbow_activated_));

    return CallbackReturn::SUCCESS;
  }

  bool ScotsVczHILExampleController::assign_parameters() {
    arm_id_ = get_node()->get_parameter("arm_id").as_string();
    is_gripper_loaded_ = get_node()->get_parameter("load_gripper").as_bool();

    auto k_gains = get_node()->get_parameter("k_gains").as_double_array();
    auto d_gains = get_node()->get_parameter("d_gains").as_double_array();
    if (k_gains.empty()) {
      RCLCPP_FATAL(get_node()->get_logger(), "k_gains parameter not set");
      return false;
    }
    if (k_gains.size() != static_cast<uint>(num_joints_)) {
      RCLCPP_FATAL(get_node()->get_logger(), "k_gains should be of size %d but is of size %ld", num_joints_, k_gains.size());
      return false;
    }
    if (d_gains.empty()) {
      RCLCPP_FATAL(get_node()->get_logger(), "d_gains parameter not set");
      return false;
    }
    if (d_gains.size() != static_cast<uint>(num_joints_)) {
      RCLCPP_FATAL(get_node()->get_logger(), "d_gains should be of size %d but is of size %ld", num_joints_, d_gains.size());
      return false;
    }
    for (int i = 0; i < num_joints_; ++i) {
      d_gains_(i) = d_gains.at(i);
      k_gains_(i) = k_gains.at(i);
    }
    return true;
  }

  CallbackReturn ScotsVczHILExampleController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
    if (!assign_parameters()) {
      return CallbackReturn::FAILURE;
    }

    franka_robot_model_ = make_unique<franka_semantic_components::FrankaRobotModel>(franka_semantic_components::FrankaRobotModel(arm_id_ + "/" + k_robot_model_interface_name, arm_id_ + "/" + k_robot_state_interface_name));
    auto collision_client = get_node()->create_client<franka_msgs::srv::SetFullCollisionBehavior>("/service_server/set_full_collision_behavior");
    compute_ik_client_ = get_node()->create_client<moveit_msgs::srv::GetPositionIK>("/compute_ik");

    while (!compute_ik_client_->wait_for_service(1s) || !collision_client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(get_node()->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return CallbackReturn::ERROR;
      }
      RCLCPP_INFO(get_node()->get_logger(), "service not available, waiting again...");
    }

    auto request = DefaultRobotBehavior::getDefaultCollisionBehaviorRequest();
    auto future_result = collision_client->async_send_request(request);
    auto success = future_result.get();

    if (!success->success) {
      RCLCPP_FATAL(get_node()->get_logger(), "Failed to set default collision behavior.");
      return CallbackReturn::ERROR;
    } else {
      RCLCPP_INFO(get_node()->get_logger(), "Default collision behavior set.");
    }

    auto parameters_client = make_shared<rclcpp::AsyncParametersClient>(get_node(), "/robot_state_publisher");
    parameters_client->wait_for_service();

    auto future = parameters_client->get_parameters({"robot_description"});
    auto result = future.get();
    if (!result.empty()) {
      robot_description_ = result[0].value_to_string();
    } else {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to get robot_description parameter.");
    }

    arm_id_ = robot_utils::getRobotNameFromDescription(robot_description_, get_node()->get_logger());
    gripper_grasp_action_client_ = rclcpp_action::create_client<franka_msgs::action::Grasp>(get_node(), fmt::format("/{}_gripper/grasp", arm_id_));
    gripper_move_action_client_ = rclcpp_action::create_client<franka_msgs::action::Move>(get_node(), fmt::format("/{}_gripper/move", arm_id_));
    gripper_stop_client_ = get_node()->create_client<std_srvs::srv::Trigger>(fmt::format("/{}_gripper/stop", arm_id_));

    assignMoveGoalOptionsCallbacks();
    assignGraspGoalOptionsCallbacks();
    return nullptr != gripper_grasp_action_client_ && nullptr != gripper_move_action_client_ && nullptr != gripper_stop_client_ ? CallbackReturn::SUCCESS : CallbackReturn::ERROR;
  }

  CallbackReturn ScotsVczHILExampleController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
    initialization_flag_ = true;
    dq_filtered_.setZero();
    joint_positions_desired_.reserve(num_joints_);
    joint_positions_current_.reserve(num_joints_);
    joint_velocities_current_.reserve(num_joints_);
    joint_efforts_current_.reserve(num_joints_);

    franka_cartesian_pose_->assign_loaned_state_interfaces(state_interfaces_);
    franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);

    if (!gripper_move_action_client_->wait_for_action_server(chrono::seconds(5))) {
      RCLCPP_ERROR(get_node()->get_logger(), "Move Action server not available after waiting.");
      return CallbackReturn::ERROR;
    }
    if (!gripper_grasp_action_client_->wait_for_action_server(chrono::seconds(5))) {
      RCLCPP_ERROR(get_node()->get_logger(), "Grasp Action server not available after waiting.");
      return CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn ScotsVczHILExampleController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
    franka_cartesian_pose_->release_interfaces();
      if (gripper_stop_client_->service_is_ready()) {
      std_srvs::srv::Trigger::Request::SharedPtr request = make_shared<std_srvs::srv::Trigger::Request>();

      auto result = gripper_stop_client_->async_send_request(request);
      if (result.get() && result.get()->success) {
        RCLCPP_INFO(get_node()->get_logger(), "Gripper stopped successfully.");
      } else {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to stop gripper.");
      }
    } else {
      RCLCPP_ERROR(get_node()->get_logger(), "Gripper stop service is not available.");
    }
    return CallbackReturn::SUCCESS;
  }

  void ScotsVczHILExampleController::assignMoveGoalOptionsCallbacks() {
    move_goal_options_.goal_response_callback = [this](const shared_ptr<rclcpp_action::ClientGoalHandle<franka_msgs::action::Move>>& goal_handle) {
      if (!goal_handle) {
        RCLCPP_ERROR(get_node()->get_logger(), RED "Move Goal (i.e. open gripper) NOT accepted." RESET);
      } else {
        RCLCPP_INFO(get_node()->get_logger(), "Move Goal accepted");
      }
    };

    move_goal_options_.feedback_callback = [this](const shared_ptr<rclcpp_action::ClientGoalHandle<franka_msgs::action::Move>>&, const shared_ptr<const franka_msgs::action::Move_Feedback>& feedback) {
      RCLCPP_INFO(get_node()->get_logger(), "Move Goal current_width [%f].", feedback->current_width);
    };

    move_goal_options_.result_callback = [this](const rclcpp_action::ClientGoalHandle<franka_msgs::action::Move>::WrappedResult& result) {
      RCLCPP_INFO(get_node()->get_logger(), "Move Goal result %s.", (rclcpp_action::ResultCode::SUCCEEDED == result.code ? YELLOW "SUCCESS" RESET : RED "FAIL" RESET));
      if (rclcpp_action::ResultCode::SUCCEEDED == result.code) {}
    };
  }

  void ScotsVczHILExampleController::assignGraspGoalOptionsCallbacks() {
    grasp_goal_options_.goal_response_callback = [this](const shared_ptr<rclcpp_action::ClientGoalHandle<franka_msgs::action::Grasp>>& goal_handle) {
      if (!goal_handle) {
        RCLCPP_ERROR(get_node()->get_logger(), RED "Grasp Goal NOT accepted." RESET);
      } else {
        RCLCPP_INFO(get_node()->get_logger(), "Grasp Goal accepted.");
      }
    };

    grasp_goal_options_.feedback_callback = [this](const shared_ptr<rclcpp_action::ClientGoalHandle<franka_msgs::action::Grasp>>&, const shared_ptr<const franka_msgs::action::Grasp_Feedback>& feedback) {
      RCLCPP_INFO(get_node()->get_logger(), "Grasp Goal current_width: %f", feedback->current_width);
    };

    grasp_goal_options_.result_callback = [this](const rclcpp_action::ClientGoalHandle<franka_msgs::action::Grasp>::WrappedResult& result) {
      RCLCPP_INFO(get_node()->get_logger(), "Grasp Goal result %s.", (rclcpp_action::ResultCode::SUCCEEDED == result.code ? GREEN "SUCCESS" RESET : RED "FAIL" RESET));
    };
  }

  void ScotsVczHILExampleController::toggleGripperState() {
    /*
    * Simply toggle the existing gripper state between open and closed.
    */
      // if (idx >= gripper_toggle_states.size()) {
      //     RCLCPP_INFO(get_node()->get_logger(), "Finished all CSV gripper states.");
      //     return; 
      // }

      // RCLCPP_INFO(get_node()->get_logger(), "INDEX2: [%d]", idx);
      // if (gripper_toggle_states.size() >= position_index) {
      //   RCLCPP_INFO(get_node()->get_logger(), "POSITION INDEX: [%ld], GRIPPER TOGGLE STATE: [%d], CURRENT_GRIPPER_STATE: [%d]", position_index, gripper_toggle_states[position_index], current_gripper_state);
      
      //   if (gripper_toggle_states[position_index] == 0 && current_gripper_state == true){
      //     RCLCPP_INFO(get_node()->get_logger(), GREEN "WW-O CS-C" RESET);
      //     openGripper();
      //     current_gripper_state = false;
      //   }
      //   else if (gripper_toggle_states[position_index] == 1 && current_gripper_state == false){
      //     RCLCPP_INFO(get_node()->get_logger(), GREEN "WW-C CS-O" RESET);
      //     graspGripper();
      //     current_gripper_state = true;
      //   }
      //   else if (gripper_toggle_states[position_index] == 0 && current_gripper_state == false){
      //     RCLCPP_INFO(get_node()->get_logger(), YELLOW "WW-O CS-O" RESET);
      //     openGripper();
      //     current_gripper_state = false;
      //   }
      //   else if (gripper_toggle_states[position_index] == 1 && current_gripper_state == true){
      //     RCLCPP_INFO(get_node()->get_logger(), YELLOW "WW-C CS-C" RESET);
      //     graspGripper();
      //     current_gripper_state = true;
      //   }
      //   else{
      //     // graspGripper();
      //     // current_gripper_state = true;
      //      RCLCPP_INFO(get_node()->get_logger(), "INDEX: [%.d]", current_gripper_state);
      //   }
      // }
      // else
      //   RCLCPP_INFO(get_node()->get_logger(), "NOT POSSIBLE");

      // // graspGripper();

      idx++;
  }

  bool ScotsVczHILExampleController::openGripper() {
    RCLCPP_INFO(get_node()->get_logger(), "Opening the gripper - Submitting a Move Goal");

    franka_msgs::action::Move::Goal move_goal;
    move_goal.width = 0.08;
    move_goal.speed = 0.2;

    shared_future<shared_ptr<rclcpp_action::ClientGoalHandle<franka_msgs::action::Move>>> move_goal_handle = gripper_move_action_client_->async_send_goal(move_goal, move_goal_options_);
    bool ret = move_goal_handle.valid();
    if (ret) {
      RCLCPP_INFO(get_node()->get_logger(), "Submited a Move Goal");
    } else {
      RCLCPP_ERROR(get_node()->get_logger(), RED "Failed to submit a Move Goal" RESET);
    }
    return ret;
  }

  void ScotsVczHILExampleController::graspGripper() {
    RCLCPP_INFO(get_node()->get_logger(), "Closing the gripper - Submitting a Grasp Goal");

    franka_msgs::action::Grasp::Goal grasp_goal;
    grasp_goal.width = 0.015;
    grasp_goal.speed = 0.05;
    grasp_goal.force = 100.0;
    grasp_goal.epsilon.inner = 0.005;
    grasp_goal.epsilon.outer = 0.010;

    shared_future<shared_ptr<rclcpp_action::ClientGoalHandle<franka_msgs::action::Grasp>>> grasp_goal_handle = gripper_grasp_action_client_->async_send_goal(grasp_goal, grasp_goal_options_);

    bool ret = grasp_goal_handle.valid();
    if (ret) {
      RCLCPP_INFO(get_node()->get_logger(), "Submited a Grasp Goal");
    } else {
      RCLCPP_ERROR(get_node()->get_logger(), RED "Failed to submit a Grasp Goal" RESET);
    }
  }

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::ScotsVczHILExampleController, controller_interface::ControllerInterface)
