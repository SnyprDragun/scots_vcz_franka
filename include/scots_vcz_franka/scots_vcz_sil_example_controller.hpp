#pragma once
#include <cmath>
#include <vector>
#include <string>
#include <fstream>
#include <cassert>
#include <exception>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <franka_example_controllers/robot_utils.hpp>
#include <controller_interface/controller_interface.hpp>

using namespace std;
using namespace Eigen;
using Vector7d = Matrix<double, 7, 1>;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_example_controllers {
    class ScotsVczSILExampleController : public controller_interface::ControllerInterface {
        public:
            [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration() const override;
            [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration() const override;
            controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;
            CallbackReturn on_init() override;
            CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
            CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
            size_t position_index = 0;

        private:
            void update_joint_states();
            Vector3d compute_new_position();
            shared_ptr<moveit_msgs::srv::GetPositionIK::Request> create_ik_service_request(const Vector3d& new_position, const Quaterniond& new_orientation, const vector<double>& joint_positions_current);
            Vector7d compute_torque_command(const Vector7d& joint_positions_desired, const Vector7d& joint_positions_current, const Vector7d& joint_velocities_current);
            bool assign_parameters();
            rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedPtr compute_ik_client_;
            Vector3d position_;
            string arm_id_;
            bool is_gripper_loaded_ = true;
            string robot_description_;
            bool initialization_flag_{true};
            double elapsed_time_{0.0};
            Vector7d dq_filtered_;
            int num_joints_{7};
            vector<double> joint_positions_desired_;
            vector<double> joint_positions_current_;
            vector<double> joint_velocities_current_;
    };
}
