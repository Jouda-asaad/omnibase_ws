#!/usr/bin/env python3
"""
Custom Planar Move Node - Implements omnidirectional movement by directly setting robot pose
This is a workaround for Gazebo Harmonic which doesn't have the planar_move plugin
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from ros_gz_interfaces.srv import SetEntityPose
import math
import time


class CustomPlanarMove(Node):
    def __init__(self):
        super().__init__('custom_planar_move')
        
        # Parameters
        self.declare_parameter('model_name', 'omnibase')
        self.declare_parameter('update_rate', 50.0)  # Hz
        
        self.model_name = self.get_parameter('model_name').value
        self.update_rate = self.get_parameter('update_rate').value
        
        # Current velocity command
        self.cmd_vel = Twist()
        
        # Current pose from odometry
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_yaw = 0.0
        
        # Subscriber for velocity commands
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Subscriber for odometry (to get current position)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Service client for setting pose
        self.set_pose_client = self.create_client(
            SetEntityPose,
            '/world/tracking_world/set_pose'
        )
        
        # Timer to update pose
        self.dt = 1.0 / self.update_rate
        self.timer = self.create_timer(self.dt, self.update_pose)
        
        self.last_cmd_time = time.time()
        
        self.get_logger().info(f'Custom Planar Move Started for model: {self.model_name}')
    
    def cmd_vel_callback(self, msg):
        """Store latest velocity command"""
        self.cmd_vel = msg
        self.last_cmd_time = time.time()
    
    def odom_callback(self, msg):
        """Update current pose from odometry"""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_z = msg.pose.pose.position.z
        
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
    
    def update_pose(self):
        """Apply velocity command by updating robot pose"""
        # Timeout - stop if no command for 0.5 seconds
        if time.time() - self.last_cmd_time > 0.5:
            return
        
        # Skip if no velocity
        if (abs(self.cmd_vel.linear.x) < 0.001 and 
            abs(self.cmd_vel.linear.y) < 0.001 and 
            abs(self.cmd_vel.angular.z) < 0.001):
            return
        
        # Calculate new pose based on velocity
        # Transform velocity from robot frame to world frame
        cos_yaw = math.cos(self.current_yaw)
        sin_yaw = math.sin(self.current_yaw)
        
        vx_world = self.cmd_vel.linear.x * cos_yaw - self.cmd_vel.linear.y * sin_yaw
        vy_world = self.cmd_vel.linear.x * sin_yaw + self.cmd_vel.linear.y * cos_yaw
        
        new_x = self.current_x + vx_world * self.dt
        new_y = self.current_y + vy_world * self.dt
        new_yaw = self.current_yaw + self.cmd_vel.angular.z * self.dt
        
        # Create pose request
        if self.set_pose_client.wait_for_service(timeout_sec=0.1):
            request = SetEntityPose.Request()
            request.entity.name = self.model_name
            request.entity.type = 2  # MODEL type
            
            request.pose.position.x = new_x
            request.pose.position.y = new_y
            request.pose.position.z = self.current_z
            
            # Quaternion from yaw
            request.pose.orientation.x = 0.0
            request.pose.orientation.y = 0.0
            request.pose.orientation.z = math.sin(new_yaw / 2)
            request.pose.orientation.w = math.cos(new_yaw / 2)
            
            # Call service asynchronously
            future = self.set_pose_client.call_async(request)
            
            self.get_logger().debug(
                f'Moving to ({new_x:.3f}, {new_y:.3f}, yaw:{new_yaw:.3f})'
            )


def main(args=None):
    rclpy.init(args=args)
    node = CustomPlanarMove()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
