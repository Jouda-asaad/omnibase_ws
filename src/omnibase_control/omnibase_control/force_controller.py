#!/usr/bin/env python3
"""
Force Controller - Converts cmd_vel to forces applied to base_link for omnidirectional movement
Uses the Gazebo ApplyLinkWrench system for direct force application
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Wrench
from std_msgs.msg import String
import json


class ForceController(Node):
    def __init__(self):
        super().__init__('force_controller')
        
        # Parameters - tune these for your robot's mass
        self.declare_parameter('force_gain', 50.0)  # Force multiplier (N per m/s)
        self.declare_parameter('torque_gain', 20.0)  # Torque multiplier (Nm per rad/s)
        self.declare_parameter('model_name', 'omnibase')
        self.declare_parameter('link_name', 'base_link')
        
        self.force_gain = self.get_parameter('force_gain').value
        self.torque_gain = self.get_parameter('torque_gain').value
        self.model_name = self.get_parameter('model_name').value
        self.link_name = self.get_parameter('link_name').value
        
        # Subscriber for velocity commands
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Publisher for wrench (using gz topic format)
        # The ApplyLinkWrench plugin listens on /world/<world_name>/wrench
        self.wrench_pub = self.create_publisher(
            Wrench,
            '/world/tracking_world/wrench',
            10
        )
        
        # Also need to publish entity info - using a JSON message for the full wrench command
        # This is hacky but works with ros_gz_bridge
        
        self.get_logger().info(f'Force Controller Started')
        self.get_logger().info(f'Force gain: {self.force_gain}, Torque gain: {self.torque_gain}')
    
    def cmd_vel_callback(self, msg):
        """Convert velocity command to force and apply to base"""
        # Calculate forces based on desired velocities
        # F = m * a, but we use a proportional gain for simplicity
        force_x = msg.linear.x * self.force_gain
        force_y = msg.linear.y * self.force_gain
        torque_z = msg.angular.z * self.torque_gain
        
        # Create wrench message
        wrench = Wrench()
        wrench.force.x = force_x
        wrench.force.y = force_y
        wrench.force.z = 0.0
        wrench.torque.x = 0.0
        wrench.torque.y = 0.0
        wrench.torque.z = torque_z
        
        self.wrench_pub.publish(wrench)
        
        # Log occasionally
        if abs(force_x) > 0.1 or abs(force_y) > 0.1 or abs(torque_z) > 0.1:
            self.get_logger().info(
                f'Applying force: ({force_x:.1f}, {force_y:.1f}) N, torque: {torque_z:.1f} Nm',
                throttle_duration_sec=1.0
            )


def main(args=None):
    rclpy.init(args=args)
    node = ForceController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
