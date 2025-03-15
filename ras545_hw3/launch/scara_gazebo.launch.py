import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_path = get_package_share_directory('ras545_hw3')
    urdf_file = os.path.join(pkg_path, 'urdf', 'hw3_scara.urdf')

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '--ros-args', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': open(urdf_file).read()}]
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-file', urdf_file, '-entity', 'scara_robot'],
            output='screen'
        )
    ])
