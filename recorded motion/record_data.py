import os
import csv
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

class JointAndEERecorder(Node):
    def __init__(self, logging_rate_hz=10):
        super().__init__('joint_and_ee_recorder')

        self.output_file = os.path.join(os.path.dirname(__file__), 'example_x_joint_ee_log.csv')
        self.csv_file = open(self.output_file, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        header = (['elapsed_time'] + [f'joint_{i+1}' for i in range(7)] + ['ee_x', 'ee_y', 'ee_z'])
        self.csv_writer.writerow(header)

        self.latest_joint_positions = None
        self.latest_ee_pose = None
        self.start_time = None

        self.subscription_joint_states = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.subscription_ee_pose = self.create_subscription(PoseStamped, '/franka_robot_state_broadcaster/current_pose', self.ee_pose_callback, 10)
        self.logging_timer = self.create_timer(1.0 / logging_rate_hz, self.log_data)
        self.get_logger().info(f"Recording joint + EE data to: {self.output_file}")

    # ----------------------------- CALLBACKS -----------------------------
    def joint_state_callback(self, msg):
        """Store latest 7 joint positions."""
        if len(msg.position) >= 7:
            self.latest_joint_positions = list(msg.position[:7])

    def ee_pose_callback(self, msg):
        """Store latest EE position."""
        self.latest_ee_pose = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

    # ---------------------------- LOGGING ------------------------------
    def log_data(self):
        """Write time, joints, and EE pose into CSV."""
        if self.latest_joint_positions is None or self.latest_ee_pose is None:
            return
        current_time = time.time()
        if self.start_time is None:
            self.start_time = current_time
        elapsed_time = round(current_time - self.start_time, 6)
        row = [elapsed_time] + self.latest_joint_positions + list(self.latest_ee_pose)
        self.csv_writer.writerow(row)
        self.csv_file.flush()

    # ----------------------------- CLEANUP -----------------------------
    def shutdown_hook(self):
        self.csv_file.close()
        self.get_logger().info(f"Recording stopped. Data saved to: {self.output_file}")

def main(args=None):
    rclpy.init(args=args)
    recorder = JointAndEERecorder()
    rclpy.get_default_context().on_shutdown(recorder.shutdown_hook)

    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        pass
    finally:
        recorder.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
