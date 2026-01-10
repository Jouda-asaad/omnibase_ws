import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    omnibase_gazebo_dir = get_package_share_directory('omnibase_gazebo')
    omnibase_description_dir = get_package_share_directory('omnibase_description')
    
    # Paths
    world_file = os.path.join(omnibase_gazebo_dir, 'worlds', 'tracking_world.sdf')
    bridge_config = os.path.join(omnibase_gazebo_dir, 'config', 'ros_gz_bridge.yaml')
    models_path = os.path.join(omnibase_gazebo_dir, 'models')
    
    # Set Gazebo model path - include models dir and parent of description for meshes
    description_parent = os.path.dirname(omnibase_description_dir)
    gz_model_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    os.environ['GZ_SIM_RESOURCE_PATH'] = f"{models_path}:{description_parent}:{gz_model_path}"
    
    # Set Gazebo plugin path to include gz_ros2_control 
    ros_lib_path = '/opt/ros/jazzy/lib'
    gz_plugin_path = os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')
    if ros_lib_path not in gz_plugin_path:
        os.environ['GZ_SIM_SYSTEM_PLUGIN_PATH'] = f"{ros_lib_path}:{gz_plugin_path}"
    
    # Gazebo simulation - run with GUI (-r for running immediately)
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen'
    )
    
    # Robot state publisher
    description_launch_file = os.path.join(
        omnibase_description_dir,
        'launch',
        'description.launch.py'
    )
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(description_launch_file)
    )
    
    # Spawn robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'omnibase',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1'
        ],
        output='screen'
    )
    
    # ROS-Gazebo bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p', f'config_file:={bridge_config}'
        ],
        output='screen'
    )
    
    # Controller spawners for ros2_control mecanum drive
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
        output='screen'
    )
    
    mecanum_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['mecanum_controller', '-c', '/controller_manager'],
        output='screen'
    )
    
    return LaunchDescription([
        gz_sim,
        robot_description_launch,
        spawn_robot,
        bridge,
        joint_state_broadcaster_spawner,
        mecanum_controller_spawner
    ])
