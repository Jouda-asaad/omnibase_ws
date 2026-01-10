from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # LED Detector Node
    led_detector = Node(
        package='omnibase_vision',
        executable='led_detector',
        name='led_detector',
        output='screen',
        parameters=[{
            'brightness_threshold': 200,
            'min_blob_area': 10,
            'max_blob_area': 1000,
        }]
    )
    
    # Base Tracker Node
    base_tracker = Node(
        package='omnibase_vision',
        executable='base_tracker',
        name='base_tracker',
        output='screen',
        parameters=[{
            'image_width': 640,
            'image_height': 480,
            'gain_x': 0.01,       # Increased from 0.002 for better response
            'gain_y': 0.01,       # Increased from 0.002 for better response
            'deadband': 15.0,     # Slightly reduced deadband
            'max_velocity': 1.0,  # Increased max velocity
        }]
    )
    
    return LaunchDescription([
        led_detector,
        base_tracker
    ])
