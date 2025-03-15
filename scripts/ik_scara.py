import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

# DH and Manipulator Parameters
# Link lengths from DH parameters:
a1, a2, a3 = 0.4, 0.3, 0.15
# Vertical offsets (z translation):
d1, d2, d3 = 0.2, 0.25, 0.15
fixed_z = d1 + d2 + d3  # 0.6

class InverseKinematicsNode(Node):
    def __init__(self):
        super().__init__('inverse_kinematics_node')
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.run_ik()

    def run_ik(self):
        while rclpy.ok():
            try:
                x = float(input("Enter x position (meters): "))
                y = float(input("Enter y position (meters): "))
                z = float(input("Enter z position (meters): "))
                phi_deg = float(input("Enter end-effector orientation (phi in degrees): "))

                # Check Z is valid
                if abs(z - fixed_z) > 1e-6:
                    print(f"Invalid Z position! z must be {fixed_z}.")
                    continue

                # Convert phi to radians
                phi_rad = np.radians(phi_deg)

                # Compute wrist center
                x_w = x - a3 * np.cos(phi_rad)
                y_w = y - a3 * np.sin(phi_rad)

                # Distance to wrist center
                r = np.sqrt(x_w**2 + y_w**2)

                # Check reach for 2R planar part (a1,a2)
                max_reach = a1 + a2
                min_reach = abs(a1 - a2)
                if r > max_reach or r < min_reach:
                    print("Target (wrist center) is out of reach for the 2R planar arm.")
                    continue

                # Solve for theta2
                cos_theta2 = (r**2 - a1**2 - a2**2) / (2 * a1 * a2)
                cos_theta2 = np.clip(cos_theta2, -1.0, 1.0)
                theta2_rad = np.degrees(np.arccos(cos_theta2))

                # We'll do elbow up solution first
                # Convert to radians for the next step
                elbow_up = np.radians(theta2_rad)

                psi_deg = np.degrees(np.arctan2(y_w, x_w))
                beta_deg = np.degrees(
                    np.arctan2(a2 * np.sin(elbow_up), a1 + a2 * np.cos(elbow_up))
                )
                theta1_up_deg = psi_deg - beta_deg
                theta3_up_deg = phi_deg - (theta1_up_deg + theta2_rad)

                # We can also compute the elbow_down solution
                elbow_down = -elbow_up
                # which means theta2_down_deg = -theta2_rad
                theta2_down_deg = -theta2_rad

                psi_down_deg = np.degrees(np.arctan2(y_w, x_w))
                beta_down_deg = np.degrees(
                    np.arctan2(a2 * np.sin(np.radians(theta2_down_deg)), a1 + a2 * np.cos(np.radians(theta2_down_deg)))
                )
                theta1_down_deg = psi_down_deg - beta_down_deg
                theta3_down_deg = phi_deg - (theta1_down_deg + theta2_down_deg)

                # We'll pick the elbow_up solution by default
                # (Or we could do a forward kinematics check for both and compare error)
                theta1_deg = theta1_up_deg
                theta2_deg = theta2_rad
                theta3_deg = theta3_up_deg

                # Convert to radians for the publisher
                t1_rad = np.radians(theta1_deg)
                t2_rad = np.radians(theta2_deg)
                t3_rad = np.radians(theta3_deg)

                # Publish joint states
                joint_msg = JointState()
                joint_msg.header.stamp = self.get_clock().now().to_msg()
                joint_msg.name = ["joint2_revolute", "joint4_revolute", "joint6_revolute"]
                joint_msg.position = [t1_rad, t2_rad, t3_rad]
                self.joint_pub.publish(joint_msg)

                # Display results
                self.get_logger().info(
                    f"IK solution => theta1={theta1_deg:.2f}, theta2={theta2_deg:.2f}, theta3={theta3_deg:.2f}"
                )

                # (Optional) Forward kinematics check
                # Convert deg to rad for quick FK
                t1 = t1_rad
                t2 = t2_rad
                t3 = t3_rad

                x_fk = (a1 * np.cos(t1)
                        + a2 * np.cos(t1 + t2)
                        + a3 * np.cos(t1 + t2 + t3))
                y_fk = (a1 * np.sin(t1)
                        + a2 * np.sin(t1 + t2)
                        + a3 * np.sin(t1 + t2 + t3))

                self.get_logger().info(f"FK check => (x={x_fk:.3f}, y={y_fk:.3f}, z={fixed_z:.3f})")

                break  # done

            except ValueError as e:
                print("Error:", e)

def main(args=None):
    rclpy.init(args=args)
    node = InverseKinematicsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
