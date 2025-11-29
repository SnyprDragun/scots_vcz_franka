#pragma once
#include <cmath>
#include <string>
#include <chrono>
#include <fstream>
#include <cassert>
#include <exception>
#include <Eigen/Dense>
#include "fmt/format.h"
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "franka_msgs/action/move.hpp"
#include "franka_msgs/action/grasp.hpp"
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <franka_example_controllers/robot_utils.hpp>
#include <controller_interface/controller_interface.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "franka_semantic_components/franka_robot_model.hpp"
#include "franka_example_controllers/gripper_example_controller.hpp"
#include <franka_example_controllers/default_robot_behavior_utils.hpp>
#include "franka_semantic_components/franka_cartesian_pose_interface.hpp"

#define RESET "\033[0m"
#define RED "\033[1;31m"
#define GREEN "\033[1;32m"
#define YELLOW "\033[1;33m"

using namespace std;
using namespace Eigen;
using namespace chrono_literals;
using Vector7d = Matrix<double, 7, 1>;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_example_controllers {
    class ScotsVczHILExampleController : public controller_interface::ControllerInterface {
        public:
            [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration() const override;
            [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration() const override;
            controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;
            CallbackReturn on_init() override;
            CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
            CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
            CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
            int gripper_toggle_value{1};
            size_t position_index = 0;
            double value{0.00};
            vector<int> gripper_toggle_states; 
            int state_index = 0;
            int idx = 0;
            bool current_gripper_state = false;

        private:
            void toggleGripperState();
            bool openGripper();
            void graspGripper();
            void assignMoveGoalOptionsCallbacks();
            void assignGraspGoalOptionsCallbacks();
            void update_joint_states();
            Vector3d compute_new_position();
            Quaterniond convert_rpy_to_quaternion(const Vector3d& rpy_angles);
            shared_ptr<moveit_msgs::srv::GetPositionIK::Request> create_ik_service_request(const Vector3d& new_position, const Quaterniond& new_orientation, const vector<double>& joint_positions_desired, const vector<double>& joint_positions_current, const vector<double>& joint_efforts_current);
            Vector7d compute_torque_command(const Vector7d& joint_positions_desired, const Vector7d& joint_positions_current, const Vector7d& joint_velocities_current);
            bool assign_parameters();
            unique_ptr<franka_semantic_components::FrankaCartesianPoseInterface> franka_cartesian_pose_;
            Quaterniond orientation_;
            Vector3d position_;
            rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedPtr compute_ik_client_;
            const bool k_elbow_activated_{false};
            bool initialization_flag_{true};
            string arm_id_;
            bool is_gripper_loaded_ = true;
            string robot_description_;
            double elapsed_time_{0.0};
            double initial_robot_time_{0.0};
            double robot_time_{0.0};
            unique_ptr<franka_semantic_components::FrankaRobotModel> franka_robot_model_;
            const string k_robot_state_interface_name{"robot_state"};
            const string k_robot_model_interface_name{"robot_model"};
            Vector7d dq_filtered_;
            rclcpp::Time start_time_;
            Vector7d k_gains_;
            Vector7d d_gains_;
            int num_joints_{7};
            vector<double> joint_positions_desired_;
            vector<double> joint_positions_current_{0, 0, 0, 0, 0, 0, 0};
            vector<double> joint_velocities_current_{0, 0, 0, 0, 0, 0, 0};
            vector<double> joint_efforts_current_{0, 0, 0, 0, 0, 0, 0};
            shared_ptr<rclcpp_action::Client<franka_msgs::action::Grasp>> gripper_grasp_action_client_;
            shared_ptr<rclcpp_action::Client<franka_msgs::action::Move>> gripper_move_action_client_;
            shared_ptr<rclcpp::Client<std_srvs::srv::Trigger>> gripper_stop_client_;
            rclcpp_action::Client<franka_msgs::action::Move>::SendGoalOptions move_goal_options_;
            rclcpp_action::Client<franka_msgs::action::Grasp>::SendGoalOptions grasp_goal_options_;
    };
}
