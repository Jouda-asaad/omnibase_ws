#!/usr/bin/env python3
"""
UAV Teleop Node - Keyboard control for UAV in simulation
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import select


class UAVTeleop(Node):
    def __init__(self):
        super().__init__('uav_teleop')
        
        self.publisher = self.create_publisher(Twist, '/uav/cmd_vel', 10)
        
        # Velocity parameters
        self.linear_speed = 0.5  # m/s
        self.vertical_speed = 0.3  # m/s
        self.angular_speed = 0.5  # rad/s
        
        self.get_logger().info('UAV Teleop Node Started')
        self.get_logger().info('Controls:')
        self.get_logger().info('  Arrow Keys: Move horizontally')
        self.get_logger().info('  W/S: Move up/down')
        self.get_logger().info('  A/D: Rotate left/right')
        self.get_logger().info('  Q: Quit')
        
        # Save terminal settings
        self.settings = termios.tcgetattr(sys.stdin)
        
    def get_key(self):
        """Get keyboard input"""
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def run(self):
        """Main control loop"""
        try:
            while rclpy.ok():
                key = self.get_key()
                
                twist = Twist()
                
                # Arrow keys (escape sequences)
                if key == '\x1b':  # ESC
                    next_key = sys.stdin.read(2)
                    if next_key == '[A':  # Up arrow
                        twist.linear.x = self.linear_speed
                    elif next_key == '[B':  # Down arrow
                        twist.linear.x = -self.linear_speed
                    elif next_key == '[C':  # Right arrow
                        twist.linear.y = -self.linear_speed
                    elif next_key == '[D':  # Left arrow
                        twist.linear.y = self.linear_speed
                
                # W/S for vertical
                elif key == 'w' or key == 'W':
                    twist.linear.z = self.vertical_speed
                elif key == 's' or key == 'S':
                    twist.linear.z = -self.vertical_speed
                
                # A/D for rotation
                elif key == 'a' or key == 'A':
                    twist.angular.z = self.angular_speed
                elif key == 'd' or key == 'D':
                    twist.angular.z = -self.angular_speed
                
                # Quit
                elif key == 'q' or key == 'Q':
                    break
                
                # Publish command
                self.publisher.publish(twist)
                
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
        
        finally:
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            
            # Stop the UAV
            twist = Twist()
            self.publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    
    node = UAVTeleop()
    node.run()
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
