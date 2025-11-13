#include <cmath>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

using namespace std;
using namespace rclcpp;
using placeholders::_1;

// Define the number of joints for the Franka Emika Panda
constexpr int NUM_JOINTS = 7;

/**
 * @brief ROS 2 Node implementing a dummy Proportional (P) torque controller for the Franka Panda.
 * * This node subscribes to joint position feedback and calculates a simple torque command
 * based on the error between the current position and a fixed desired setpoint.
 */
class DummyTorqueController : public Node
{
public:
    DummyTorqueController()
    : Node("dummy_torque_controller"),
      p_gain_(20.0), // Proportional gain (tune this in a real system)
      desired_positions_({0.0, -M_PI_4 / 2.0, 0.0, -3.0 * M_PI_4 / 2.0, 0.0, M_PI_2, M_PI_4 / 2.0}) // Example 'home' position
    {
        // 1. Initialize the Publisher for Torque Commands
        // NOTE: In a real franka_ros2 setup, you might need to use franka_msgs/msg/TorqueCommand
        // For simplicity and general compatibility, we use Float64MultiArray here.
        torque_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/franka_effort_controller/joint_commands", 10);

        // 2. Initialize the Subscriber for Joint Position Feedback
        // Topic typically provides joint positions from the Franka State Controller.
        state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/franka_state_controller/joint_states", 10,
            bind(&DummyTorqueController::jointStateCallback, this, _1));

        RCLCPP_INFO(this->get_logger(), "DummyTorqueController node initialized.");
        RCLCPP_INFO(this->get_logger(), "P Gain set to: %f", p_gain_);
    }

private:
    Subscription<sensor_msgs::msg::JointState>::SharedPtr state_subscriber_;
    Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torque_publisher_;

    // Controller parameters
    const double p_gain_;
    const vector<double> desired_positions_;

    /**
     * @brief Callback function for incoming joint state messages (position feedback).
     * * @param msg The received JointState message containing current joint positions.
     */
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // Check if the number of received positions matches the expected number of joints
        if (msg->position.size() != NUM_JOINTS) {
            RCLCPP_WARN(this->get_logger(), "Received %zu joint positions, expected %d. Skipping calculation.",
                        msg->position.size(), NUM_JOINTS);
            return;
        }

        // Create the message for torque output
        auto torque_msg = std::make_unique<std_msgs::msg::Float64MultiArray>();
        torque_msg->data.resize(NUM_JOINTS);

        // --- DUMMY P-CONTROLLER IMPLEMENTATION ---
        for (int i = 0; i < NUM_JOINTS; ++i) {
            // Get current position feedback
            double current_pos = msg->position[i];
            
            // Get desired setpoint
            double desired_pos = desired_positions_[i];

            // Calculate position error
            double error = desired_pos - current_pos;

            // Calculate torque using P-control: Torque = P_gain * Error
            double calculated_torque = p_gain_ * error;

            // Apply a simple saturation limit (e.g., +/- 10 Nm) for safety in a real system
            // This is a crucial step to prevent excessive torques
            const double MAX_TORQUE = 10.0;
            if (calculated_torque > MAX_TORQUE) {
                calculated_torque = MAX_TORQUE;
            } else if (calculated_torque < -MAX_TORQUE) {
                calculated_torque = -MAX_TORQUE;
            }

            // Assign the calculated torque to the output message
            torque_msg->data[i] = calculated_torque;
        }
        
        // Publish the calculated torque commands
        torque_publisher_->publish(move(torque_msg));
    }
};

/**
 * @brief Main function to initialize and run the ROS 2 node.
 */
int main(int argc, char * argv[])
{
    // Initialize ROS 2
    init(argc, argv);

    // Create and spin the node
    spin(make_shared<DummyTorqueController>());

    // Shutdown ROS 2
    shutdown();
    return 0;
}