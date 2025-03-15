#!/usr/bin/env python3

import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# DH and Manipulator Parameters
# Link lengths from DH parameters
a1, a2, a3 = 0.4, 0.3, 0.15
# Vertical offsets (z translation)
d1, d2, d3 = 0.2, 0.25, 0.15
fixed_z = d1 + d2 + d3  # 0.6

# Four corners of a square in the XY plane (z=0.6, phi=0)
SQUARE_POINTS = [
    (0.60,  0.15, 0.60, 0.0),
    (0.30,  0.15, 0.60, 0.0),
    (0.30, -0.15, 0.60, 0.0),
    (0.60, -0.15, 0.60, 0.0),
]

# Number of sub-steps per edge (higher -> smoother path)
STEPS_PER_EDGE = 20

# Publish rate in Hz
PUBLISH_RATE = 10.0  # 10 Hz => publish every 0.1s

class InverseKinematicsNode(Node):
    def __init__(self):
        super().__init__('inverse_kinematics_node')
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Run the square drawing sequence
        self.draw_square()

    def draw_square(self):
        """
        Continuously loops around four corners of a square, 
        interpolating sub-points along each edge, and using IK 
        to move the robot in small steps, so it draws a square path.
        """
        while rclpy.ok():
            # For each edge (corner_i -> corner_(i+1))
            for i in range(len(SQUARE_POINTS)):
                # Current corner
                (x1, y1, z1, phi1) = SQUARE_POINTS[i]
                # Next corner (wrap around with mod)
                (x2, y2, z2, phi2) = SQUARE_POINTS[(i+1) % len(SQUARE_POINTS)]

                # Interpolate in Cartesian space
                for step in range(STEPS_PER_EDGE+1):
                    alpha = step / STEPS_PER_EDGE
                    x = x1 + alpha*(x2 - x1)
                    y = y1 + alpha*(y2 - y1)
                    z = z1 + alpha*(z2 - z1)
                    phi = phi1 + alpha*(phi2 - phi1)

                    # Compute IK for each sub-point
                    angles_deg = self.compute_ik(x, y, z, phi)
                    if angles_deg is not None:
                        # Convert to radians
                        angles_rad = [math.radians(a) for a in angles_deg]

                        # Publish joint states
                        self.publish_joint_states(angles_rad)
                    else:
                        self.get_logger().warn(f"No IK solution for x={x:.3f}, y={y:.3f}. Skipping sub-step.")
                    
                    time.sleep(1.0 / PUBLISH_RATE)

    def compute_ik(self, x, y, z, phi_deg):
        """
        Compute IK angles in DEGREES using the logic from your code. 
        Returns [?1_deg, ?2_deg, ?3_deg], or None if unreachable.
        """
        # Check Z
        if abs(z - fixed_z) > 1e-6:
            return None
        
        # Convert phi to radians
        phi_rad = math.radians(phi_deg)

        # Compute wrist center
        x_w = x - a3 * math.cos(phi_rad)
        y_w = y - a3 * math.sin(phi_rad)

        # Distance to wrist center
        r = math.hypot(x_w, y_w)

        # Check 2-link reach
        max_reach = a1 + a2
        min_reach = abs(a1 - a2)
        if r > max_reach or r < min_reach:
            return None
        
        # Solve for ?2
        cos_theta2 = (r**2 - a1**2 - a2**2) / (2 * a1 * a2)
        cos_theta2 = max(min(cos_theta2, 1.0), -1.0)
        theta2_up = math.degrees(math.acos(cos_theta2))

        # We'll pick elbow-up solution
        elbow_up_rad = math.radians(theta2_up)
        psi_deg = math.degrees(math.atan2(y_w, x_w))
        beta_deg = math.degrees(
            math.atan2(a2 * math.sin(elbow_up_rad), a1 + a2 * math.cos(elbow_up_rad))
        )
        theta1_up_deg = psi_deg - beta_deg
        theta3_up_deg = phi_deg - (theta1_up_deg + theta2_up)

        # Return the elbow-up solution in degrees
        return [theta1_up_deg, theta2_up, theta3_up_deg]

    def publish_joint_states(self, angles_rad):
        """
        Publish a single JointState message for the 3 angles (in radians).
        """
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["joint2_revolute", "joint4_revolute", "joint6_revolute"]
        msg.position = angles_rad
        self.joint_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = InverseKinematicsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
