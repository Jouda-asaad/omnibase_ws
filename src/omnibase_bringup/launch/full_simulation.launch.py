import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Complete simulation bringup:
    1. Gazebo simulation with robot and UAV
    2. Vision system for LED detection
    3. Drone control GUI
    4. Custom planar move for omnidirectional base control
    """
    
    # Get package directories
    omnibase_gazebo_dir = get_package_share_directory('omnibase_gazebo')
    omnibase_vision_dir = get_package_share_directory('omnibase_vision')
    
    # Gazebo simulation
    simulation_launch_file = os.path.join(
        omnibase_gazebo_dir,
        'launch',
        'simulation.launch.py'
    )
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(simulation_launch_file)
    )
    
    # Vision system
    vision_launch_file = os.path.join(
        omnibase_vision_dir,
        'launch',
        'vision.launch.py'
    )
    vision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(vision_launch_file)
    )
    
    # Drone control GUI
    drone_gui_node = Node(
        package='omnibase_control',
        executable='drone_gui',
        name='drone_gui',
        output='screen'
    )
    
    return LaunchDescription([
        simulation_launch,
        vision_launch,
        drone_gui_node,
    ])

