#pragma once

#include <map>
#include <cmath>
#include <string>
#include <vector>
#include <cassert>
#include <fstream>
#include <sstream>
#include <exception>
#include <algorithm>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <franka_example_controllers/robot_utils.hpp>
#include <controller_interface/controller_interface.hpp>

using namespace std;
using namespace Eigen;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_example_controllers {

    struct ScotsState {
        double x, y, z;
        double vx, vy, vz;
        ScotsState() : x(0), y(0), z(0), vx(0), vy(0), vz(0) {}
        ScotsState(double x_, double y_, double z_, double vx_, double vy_, double vz_) : x(x_), y(y_), z(z_), vx(vx_), vy(vy_), vz(vz_) {}
    };

    class ScotsVCZExampleController : public controller_interface::ControllerInterface {
        public:
            using Vector7d = Matrix<double, 7, 1>;

            [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration() const override;
            [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration() const override;
            controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;
            CallbackReturn on_init() override;
            CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
            CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
            CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

        private:
            void update_joint_states();
            bool load_scots_controller(const string& csv_file_path);
            Vector3d find_scots_velocity(const Vector3d& current_position);
            MatrixXd compute_jacobian(const vector<double>& joint_positions);
            Affine3d forward_kinematics(const vector<double>& joint_positions);
            double sigmoid(double x);
            bool is_goal_reached(const Vector3d& position);
            bool is_position_reachable(const Vector3d& position);
            Vector7d compute_scots_torque(const Vector3d& task_velocity, const vector<double>& joint_positions, const vector<double>& joint_velocities);
            bool assign_parameters();
            string arm_id_;
            string robot_description_;
            string scots_controller_path_;
            bool is_gripper_loaded_ = false;
            double rho_{1.0};
            double position_tolerance_{0.01};
            Vector7d max_joint_torques_;
            vector<ScotsState> scots_controller_;
            Vector3d last_valid_velocity_;
            bool controller_loaded_{false};
            vector<double> goal_region_{0.5, 0.6, 0.5, 0.6, 0.5, 0.6};
            vector<double> workspace_limits_{-0.7, 0.7, -0.7, 0.7, -0.7, 0.7};
            bool initialization_flag_{true};
            bool goal_reached_{false};
            int num_joints_{7};
            vector<double> joint_positions_current_;
            vector<double> joint_velocities_current_;
            Vector3d current_ee_position_;
            int update_counter_{0};
            int velocity_lookup_failures_{0};
            static constexpr int MAX_LOOKUP_FAILURES = 100;
    };
}
