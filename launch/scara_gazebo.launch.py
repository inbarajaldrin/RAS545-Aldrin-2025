import os
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import xacro

def generate_launch_description():
    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    pkg_path = os.path.join(get_package_share_directory('ras545_hw3'))

    # Ensure Gazebo can find the model and mesh paths
    pkg_share_path = os.path.join(get_package_prefix('ras545_hw3'), 'share')
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] += os.pathsep + pkg_share_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] = pkg_share_path

    # Process the URDF file
    urdf_file = os.path.join(pkg_path, 'urdf', 'hw3_scara_gazebo.urdf')
    # Read URDF file to pass as a parameter
    with open(urdf_file, "r") as infp:
        robot_description = infp.read()

    # Load SCARA controllers
    scara_controllers = os.path.join(pkg_path, "config", "scara_gazebo.yaml")

    # Create a robot_state_publisher node
    params = {'robot_description': robot_description, 'use_sim_time': use_sim_time}

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[params]
    )

    joint_state_publisher = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen"
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        #parameters=[{"robot_description": robot_description}, scara_controllers, {"use_sim_time": use_sim_time}],
        parameters=[scara_controllers],
        output="screen"
    )


    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'
        )]),
    )

    # Run the spawner node from the gazebo_ros package.
    # The entity name doesn't really matter if you only have one
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'hw3_scara'],
        output='screen'
    )

    # Launch them all!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'
        ),
        robot_state_publisher,
        joint_state_publisher,
        controller_manager,
        gazebo,
        spawn_entity,
    ])
