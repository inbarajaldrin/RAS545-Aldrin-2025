import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
import math

class ForwardKinematics(Node):
    def __init__(self, target_angles):
        super().__init__('forward_kinematics_node')
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        
        # TF Listener to get the EE position
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Robot link lengths
        self.L1 = 0.4  # Link 1 length
        self.L2 = 0.3  # Link 2 length
        self.L3 = 0.15  # Link 3 (End-effector length)
        
        # Publish the desired joint state once
        self.publish_joint_states(target_angles)
        self.compute_fk(target_angles)
        # self.get_ee_position()
    
    def publish_joint_states(self, target_angles):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["joint2_revolute", "joint4_revolute", "joint6_revolute"]
        msg.position = target_angles  # Set given angles directly
        
        self.publisher.publish(msg)
        target_angles_degrees = [math.degrees(angle) for angle in target_angles]
        self.get_logger().info(f"Moving robot to angles (radians): {target_angles}")
        self.get_logger().info(f"Moving robot to angles (degrees): {target_angles_degrees}")
    
    def compute_fk(self, target_angles):
        theta1, theta2, theta3 = target_angles  # Angles in radians
        
        # Compute Forward Kinematics
        x = (self.L1 * math.cos(theta1)) + (self.L2 * math.cos(theta1 + theta2)) + (self.L3 * math.cos(theta1 + theta2 + theta3))
        y = (self.L1 * math.sin(theta1)) + (self.L2 * math.sin(theta1 + theta2)) + (self.L3 * math.sin(theta1 + theta2 + theta3))
        z = 0.6  # SCARA robot has a fixed Z height
        
        self.get_logger().info(f"Computed End-Effector Position (FK): x={x:.3f}, y={y:.3f}, z={z:.3f}")
    
    # def get_ee_position(self):
    #     try:
    #         transform = self.tf_buffer.lookup_transform('world', 'link_3', rclpy.time.Time())
    #         x = transform.transform.translation.x
    #         y = transform.transform.translation.y
    #         z = transform.transform.translation.z
    #         self.get_logger().info(f"Actual EE Position from TF: x={x:.3f}, y={y:.3f}, z={z:.3f}")
    #     except Exception as e:
    #         self.get_logger().warn(f"Could not retrieve EE position from TF: {str(e)}")

def main():
    rclpy.init()

    # User Inputs (Angles in Degrees)
    theta1 = float(input("Enter theta1 (degrees): "))
    theta2 = float(input("Enter theta2 (degrees): "))
    theta3 = float(input("Enter theta3 (degrees): "))

    # Convert degrees to radians (ROS uses radians)
    target_angles = [math.radians(theta1), 
                     math.radians(theta2), 
                     math.radians(theta3)]

    # Run FK Node
    node = ForwardKinematics(target_angles)
    rclpy.spin_once(node)  # Only publish once
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()