#!/usr/bin/env python3
"""
Base Tracker Node - Controls mecanum base to follow UAV based on LED position
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
import math


class BaseTracker(Node):
    def __init__(self):
        super().__init__('base_tracker')
        
        # Parameters
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('gain_x', 0.25)  # 5x faster turning (was 0.05)
        self.declare_parameter('gain_y', 0.25)  # 5x faster driving (was 0.05)
        self.declare_parameter('deadband', 5.0)  # Tighter deadband for better accuracy (was 15.0)
        self.declare_parameter('max_velocity', 5.0)  # 5x higher top speed (was 1.0)
        self.declare_parameter('min_tracking_area', 5.0)  # Be more sensitive to distant targets
        
        self.image_width = self.get_parameter('image_width').value
        self.image_height = self.get_parameter('image_height').value
        self.gain_x = self.get_parameter('gain_x').value
        self.gain_y = self.get_parameter('gain_y').value
        self.deadband = self.get_parameter('deadband').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.min_tracking_area = self.get_parameter('min_tracking_area').value
        
        # Image center
        self.center_x = self.image_width / 2.0
        self.center_y = self.image_height / 2.0
        
        # Subscribers
        self.led_sub = self.create_subscription(
            Point,
            '/led_centroid',
            self.led_callback,
            10
        )
        
        # Publishers - custom_planar_move uses /cmd_vel for omnidirectional control
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.get_logger().info('Base Tracker Node Started')
        self.get_logger().info(f'Image center: ({self.center_x}, {self.center_y})')
        self.get_logger().info(f'Gains: X={self.gain_x}, Y={self.gain_y}')
        self.get_logger().info(f'Min tracking area: {self.min_tracking_area}')
        
    def led_callback(self, msg):
        """Calculate control command based on LED position"""
        # Check if LED is at tracking height (blob area in z field)
        blob_area = msg.z
        
        if blob_area < self.min_tracking_area:
            # LED is too far away (high altitude), don't track
            twist = Twist()
            self.cmd_vel_pub.publish(twist)
            return
        
        # Calculate error from image center
        error_x = msg.x - self.center_x  # Positive = LED is to the right
        error_y = msg.y - self.center_y  # Positive = LED is below center
        
        # Apply deadband
        if abs(error_x) < self.deadband:
            error_x = 0.0
        if abs(error_y) < self.deadband:
            error_y = 0.0
        
        # Proportional control for MECANUM DRIVE with UPWARD-facing camera
        # Mecanum drive uses: linear.x (forward/back) + linear.y (strafe left/right)
        # 
        # Camera looks UP at the sky. Image coordinates:
        # - Image X increases to the RIGHT
        # - Image Y increases DOWNWARD
        #
        # For an upward-facing camera on a robot facing +X:
        # - LED on RIGHT side of image (error_x > 0) = UAV is to robot's RIGHT
        #   -> Robot should strafe RIGHT (positive linear.y in Gazebo MecanumDrive = LEFT)
        #   -> So we need NEGATIVE linear.y to go RIGHT
        #
        # - LED on BOTTOM of image (error_y > 0) = UAV is IN FRONT of robot
        #   -> Robot should move FORWARD (positive linear.x)
        
        vel_x = self.gain_y * error_y      # Move forward if LED is below center
        vel_y = -self.gain_x * error_x     # Strafe: LED right (error>0) -> strafe right (vel_y<0)
        
        # Clamp velocities
        vel_x = max(-self.max_velocity, min(self.max_velocity, vel_x))
        vel_y = max(-self.max_velocity, min(self.max_velocity, vel_y))
        
        # Create and publish Twist message
        twist = Twist()
        twist.linear.x = vel_x
        twist.linear.y = vel_y
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        
        self.cmd_vel_pub.publish(twist)
        
        # Log status
        if error_x != 0.0 or error_y != 0.0:
            self.get_logger().info(
                f'LED at ({msg.x:.1f}, {msg.y:.1f}), Area: {blob_area:.1f}, '
                f'Error: ({error_x:.1f}, {error_y:.1f}), '
                f'Vel: (fwd:{vel_x:.3f}, strafe:{vel_y:.3f})',
                throttle_duration_sec=1.0
            )


def main(args=None):
    rclpy.init(args=args)
    node = BaseTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
