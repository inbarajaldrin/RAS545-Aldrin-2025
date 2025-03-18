# ros2 topic pub --once /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory '
# joint_names: ["joint2_revolute", "joint4_revolute", "joint6_revolute"]
# points:
#   - positions: [0.5, -0.3, 0.8]  # Target positions in radians
#     velocities: [0.0, 0.0, 0.0]
#     accelerations: [0.0, 0.0, 0.0]
#     time_from_start: {sec: 2, nanosec: 0}
# '

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math

class ScaraJointCommander(Node):
    def __init__(self):
        super().__init__('scara_joint_commander')
        self.publisher = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.get_logger().info("\nSCARA Joint Commander Ready. Enter angles in degrees.")

    def send_joint_command(self, joint_angles):
        msg = JointTrajectory()
        msg.joint_names = ["joint2_revolute", "joint4_revolute", "joint6_revolute"]

        point = JointTrajectoryPoint()
        point.positions = [math.radians(angle) for angle in joint_angles]  # Convert degrees to radians
        point.velocities = [0.0, 0.0, 0.0]
        point.accelerations = [0.0, 0.0, 0.0]
        point.time_from_start.sec = 2  # Move in 2 seconds

        msg.points.append(point)
        self.publisher.publish(msg)
        self.get_logger().info(f"Moving to Joint Positions (Degrees): {joint_angles}")
        self.get_logger().info(f"Moving to Joint Positions (Radians): {point.positions}")

def main():
    rclpy.init()
    node = ScaraJointCommander()

    try:
        while rclpy.ok():
            input_angles = input("\nEnter joint angles in degrees (comma-separated, e.g., 30, -45, 60) or 'exit' to quit: ").strip()
            if input_angles.lower() == 'exit':
                break

            try:
                joint_angles = [float(angle) for angle in input_angles.split(',')]
                if len(joint_angles) != 3:
                    raise ValueError("Please enter exactly 3 values.")

                node.send_joint_command(joint_angles)
                rclpy.spin_once(node, timeout_sec=0.1)  # Allow ROS 2 to process messages
                
            except ValueError as e:
                node.get_logger().error(f"Invalid input: {e}")

    except KeyboardInterrupt:
        node.get_logger().info("Exiting SCARA Joint Commander...")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
