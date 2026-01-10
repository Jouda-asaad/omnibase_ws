import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():
    # Get the URDF file path
    package_dir = get_package_share_directory('omnibase_description')
    urdf_file = os.path.join(package_dir, 'urdf', 'omnibase.urdf.xacro')
    
    # Process xacro to get robot description
    robot_description = xacro.process_file(urdf_file).toxml()
    
    # Robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )
    
    return LaunchDescription([
        robot_state_publisher_node
    ])

