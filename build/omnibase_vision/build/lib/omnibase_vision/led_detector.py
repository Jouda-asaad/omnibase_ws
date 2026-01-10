#!/usr/bin/env python3
"""
LED Detector Node - Detects bright LED markers in camera image
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np


class LEDDetector(Node):
    def __init__(self):
        super().__init__('led_detector')
        
        # Parameters
        self.declare_parameter('brightness_threshold', 200)
        self.declare_parameter('min_blob_area', 5)
        self.declare_parameter('max_blob_area', 5000)
        self.declare_parameter('smoothing_alpha', 0.15)  # Much smoother (was 0.3) - reduces jitter significantly
        
        self.brightness_threshold = self.get_parameter('brightness_threshold').value
        self.min_blob_area = self.get_parameter('min_blob_area').value
        self.max_blob_area = self.get_parameter('max_blob_area').value
        self.smoothing_alpha = self.get_parameter('smoothing_alpha').value
        
        # Smoothing state
        self.smoothed_x = None
        self.smoothed_y = None
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Publishers
        self.debug_image_pub = self.create_publisher(Image, '/camera/debug', 10)
        self.led_position_pub = self.create_publisher(Point, '/led_centroid', 10)
        
        self.get_logger().info('LED Detector Node Started')
        
    def image_callback(self, msg):
        """Process camera image to detect pink LEDs"""
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Convert to HSV for color detection
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            # Red color range in HSV (red wraps around 0/180)
            # Lower red range: H 0-10
            lower_red1 = np.array([0, 100, 100])
            upper_red1 = np.array([10, 255, 255])
            # Upper red range: H 170-180
            lower_red2 = np.array([170, 100, 100])
            upper_red2 = np.array([180, 255, 255])
            
            # Create masks for red color
            red_mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
            red_mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
            red_mask = cv2.bitwise_or(red_mask1, red_mask2)
            
            # Also check for bright/white objects (emissive materials often appear white)
            lower_bright = np.array([0, 0, 220])
            upper_bright = np.array([180, 50, 255])
            bright_mask = cv2.inRange(hsv, lower_bright, upper_bright)
            
            # Combine masks
            combined_mask = cv2.bitwise_or(red_mask, bright_mask)
            
            # Apply stronger morphological operations to merge fragments and clean noise
            kernel = np.ones((5, 5), np.uint8)
            combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel)
            combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, kernel)
            
            # Find contours
            contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Filter contours by area
            led_contours = []
            for contour in contours:
                area = cv2.contourArea(contour)
                if self.min_blob_area < area < self.max_blob_area:
                    led_contours.append(contour)
            
            # Calculate centroid of all LEDs
            if led_contours:
                all_points = np.vstack(led_contours)
                M = cv2.moments(all_points)
                
                if M['m00'] > 0:
                    raw_cx = float(M['m10'] / M['m00'])
                    raw_cy = float(M['m01'] / M['m00'])
                    
                    # Apply exponential smoothing
                    if self.smoothed_x is None:
                        self.smoothed_x = raw_cx
                        self.smoothed_y = raw_cy
                    else:
                        self.smoothed_x = self.smoothing_alpha * raw_cx + (1 - self.smoothing_alpha) * self.smoothed_x
                        self.smoothed_y = self.smoothing_alpha * raw_cy + (1 - self.smoothing_alpha) * self.smoothed_y
                    
                    cx = int(self.smoothed_x)
                    cy = int(self.smoothed_y)
                    
                    # Calculate total blob area as proxy for height
                    total_area = sum(cv2.contourArea(c) for c in led_contours)
                    
                    # Publish LED centroid position (z = blob area for height estimation)
                    centroid_msg = Point()
                    centroid_msg.x = float(cx)
                    centroid_msg.y = float(cy)
                    centroid_msg.z = float(total_area)  # Area as height proxy
                    self.led_position_pub.publish(centroid_msg)
                    
                    # Draw on debug image
                    debug_image = cv_image.copy()
                    cv2.drawContours(debug_image, led_contours, -1, (0, 255, 0), 2)
                    cv2.circle(debug_image, (cx, cy), 10, (0, 0, 255), -1)
                    cv2.circle(debug_image, (cx, cy), 15, (255, 0, 0), 2)
                    
                    # Draw crosshair at image center
                    h, w = debug_image.shape[:2]
                    cv2.line(debug_image, (w//2 - 20, h//2), (w//2 + 20, h//2), (255, 255, 0), 2)
                    cv2.line(debug_image, (w//2, h//2 - 20), (w//2, h//2 + 20), (255, 255, 0), 2)
                    
                    # Publish debug image
                    debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
                    self.debug_image_pub.publish(debug_msg)
                    
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = LEDDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
