import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = "ras545_hw3"

    # Directly reference the source directory (not the installed share directory)
    ros2_ws_path = os.path.expanduser("~/ros2_ws/src")  # Change this if your workspace path is different

    # Paths to URDF and RViz config file in the source directory
    urdf_file = os.path.join(ros2_ws_path, package_name, "urdf", "hw3_scara.urdf")
    rviz_config_file = os.path.join(ros2_ws_path, package_name, "rviz", "scara_config.rviz")

    # Read URDF file to pass as a parameter
    with open(urdf_file, "r") as infp:
        robot_description = infp.read()

    return LaunchDescription([
        # Robot State Publisher (publishes TF and robot_description)
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}]
        ),

        # Joint State Publisher GUI (for manually moving joints)
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            output="screen",
        ),

        # RViz2 with pre-configured display settings
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            arguments=["-d", rviz_config_file, "--fixed-frame", "world"]
        ),
    ])
