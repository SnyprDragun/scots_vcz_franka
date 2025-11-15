#pragma once

#include <cmath>
#include <string>
#include <vector>
#include <map>
#include <cassert>
#include <exception>
#include <fstream>
#include <sstream>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <franka_example_controllers/robot_utils.hpp>
#include <controller_interface/controller_interface.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_example_controllers {

    // Structure to hold SCOTS controller state
    struct ScotsState {
        double x, y, z;
        double vx, vy, vz;
        
        ScotsState() : x(0), y(0), z(0), vx(0), vy(0), vz(0) {}
        ScotsState(double x_, double y_, double z_, double vx_, double vy_, double vz_)
            : x(x_), y(y_), z(z_), vx(vx_), vy(vy_), vz(vz_) {}
    };

    class ScotsVCZExampleController : public controller_interface::ControllerInterface {
        public:
            using Vector7d = Eigen::Matrix<double, 7, 1>;

            [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration() const override;
            [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration() const override;
            controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;
            CallbackReturn on_init() override;
            CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
            CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
            CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

        private:
            // Joint state update
            void update_joint_states();
            
            // CSV parsing for SCOTS controller
            bool load_scots_controller(const std::string& csv_file_path);
            
            // Find closest state in SCOTS controller
            bool find_scots_velocity(const Eigen::Vector3d& current_position, Eigen::Vector3d& velocity);
            
            // Compute Jacobian matrix
            Eigen::MatrixXd compute_jacobian(const std::vector<double>& joint_positions);
            
            // Forward kinematics
            Eigen::Affine3d forward_kinematics(const std::vector<double>& joint_positions);
            
            // Sigmoid function
            double sigmoid(double x);
            
            // Check if goal region reached
            bool is_goal_reached(const Eigen::Vector3d& position);
            
            // Check if position is reachable
            bool is_position_reachable(const Eigen::Vector3d& position);
            
            // Compute torque using SCOTS control law
            Vector7d compute_scots_torque(
                const Eigen::Vector3d& task_velocity,
                const std::vector<double>& joint_positions,
                const std::vector<double>& joint_velocities);
            
            // Parameter assignment
            bool assign_parameters();

            // ROS parameters
            std::string arm_id_;
            std::string robot_description_;
            std::string scots_controller_path_;
            bool is_gripper_loaded_ = false;
            
            // Control parameters
            double rho_{1.0};  // Funnel parameter
            double position_tolerance_{0.01};  // Tolerance for state matching in meters
            
            // Joint limits (for FR3)
            Vector7d max_joint_torques_;
            
            // SCOTS controller data
            std::vector<ScotsState> scots_controller_;
            Eigen::Vector3d last_valid_velocity_;
            bool controller_loaded_{false};
            
            // Goal region bounds [x_min, x_max, y_min, y_max, z_min, z_max]
            std::vector<double> goal_region_{0.5, 0.6, 0.5, 0.6, 0.5, 0.6};
            
            // Workspace limits (safety bounds)
            std::vector<double> workspace_limits_{-0.7, 0.7, -0.7, 0.7, 0.2, 0.7};
            
            // State variables
            bool initialization_flag_{true};
            bool goal_reached_{false};
            int num_joints_{7};
            
            std::vector<double> joint_positions_current_;
            std::vector<double> joint_velocities_current_;
            
            // Current end-effector pose
            Eigen::Vector3d current_ee_position_;
            
            // Controller state tracking
            int update_counter_{0};
            int velocity_lookup_failures_{0};
            static constexpr int MAX_LOOKUP_FAILURES = 100;  // Stop after 100 consecutive failures
    };
}