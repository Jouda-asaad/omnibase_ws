#!/usr/bin/env python3
"""
Drone Control GUI - Tkinter interface for UAV control and landing status
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
import tkinter as tk
from tkinter import ttk
import threading


class DroneControlGUI(Node):
    def __init__(self):
        super().__init__('drone_control_gui')
        
        # Parameters
        self.declare_parameter('alignment_threshold', 15.0)  # pixels
        self.declare_parameter('image_center_x', 320.0)
        self.declare_parameter('image_center_y', 240.0)
        
        self.alignment_threshold = self.get_parameter('alignment_threshold').value
        self.center_x = self.get_parameter('image_center_x').value
        self.center_y = self.get_parameter('image_center_y').value
        
        # State
        self.is_aligned = False
        self.led_x = 0.0
        self.led_y = 0.0
        self.is_landing = False
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/uav/cmd_vel', 10)
        
        # Subscribers
        self.led_sub = self.create_subscription(
            Point,
            '/led_centroid',
            self.led_callback,
            10
        )
        
        # Movement speeds
        self.linear_speed = 0.5
        self.altitude_speed = 0.3
        
        # Setup GUI in main thread
        self.root = None
        self.status_label = None
        self.land_button = None
        
        self.get_logger().info('Drone Control GUI Node Started')
    
    def led_callback(self, msg):
        """Check if LED is centered (drone aligned with base)"""
        self.led_x = msg.x
        self.led_y = msg.y
        
        error_x = abs(msg.x - self.center_x)
        error_y = abs(msg.y - self.center_y)
        
        self.is_aligned = (error_x < self.alignment_threshold and 
                          error_y < self.alignment_threshold)
    
    def send_velocity(self, x=0.0, y=0.0, z=0.0):
        """Send velocity command to UAV"""
        twist = Twist()
        twist.linear.x = x
        twist.linear.y = y
        twist.linear.z = z
        self.cmd_vel_pub.publish(twist)
    
    def stop_drone(self):
        """Stop all drone movement"""
        self.send_velocity(0.0, 0.0, 0.0)
    
    def start_landing(self):
        """Begin landing sequence"""
        if self.is_aligned:
            self.is_landing = True
            self.get_logger().info('Landing initiated!')
        else:
            self.get_logger().warn('Cannot land - not aligned!')
    
    def create_gui(self):
        """Create the tkinter GUI"""
        self.root = tk.Tk()
        self.root.title("Drone Control Panel")
        self.root.geometry("400x500")
        self.root.configure(bg='#2b2b2b')
        
        style = ttk.Style()
        style.theme_use('clam')
        
        # Title
        title = tk.Label(self.root, text="🚁 Drone Control", 
                        font=('Arial', 20, 'bold'),
                        bg='#2b2b2b', fg='white')
        title.pack(pady=15)
        
        # Status Frame
        status_frame = tk.Frame(self.root, bg='#3b3b3b', padx=20, pady=15)
        status_frame.pack(fill='x', padx=20, pady=10)
        
        tk.Label(status_frame, text="Alignment Status:", 
                font=('Arial', 12), bg='#3b3b3b', fg='white').pack()
        
        self.status_label = tk.Label(status_frame, text="Waiting for detection...",
                                     font=('Arial', 16, 'bold'),
                                     bg='#3b3b3b', fg='yellow')
        self.status_label.pack(pady=5)
        
        self.position_label = tk.Label(status_frame, text="LED: (-, -)",
                                       font=('Arial', 10),
                                       bg='#3b3b3b', fg='#aaa')
        self.position_label.pack()
        
        # Control Frame
        control_frame = tk.Frame(self.root, bg='#2b2b2b')
        control_frame.pack(pady=20)
        
        # Movement buttons
        btn_style = {'font': ('Arial', 14), 'width': 5, 'height': 2}
        
        # Forward
        tk.Button(control_frame, text="▲\nW", bg='#4a4a4a', fg='white', **btn_style,
                 command=lambda: self.send_velocity(x=self.linear_speed)).grid(row=0, column=1, padx=5, pady=5)
        
        # Left
        tk.Button(control_frame, text="◀\nA", bg='#4a4a4a', fg='white', **btn_style,
                 command=lambda: self.send_velocity(y=self.linear_speed)).grid(row=1, column=0, padx=5, pady=5)
        
        # Stop
        tk.Button(control_frame, text="■\nSTOP", bg='#aa4444', fg='white', **btn_style,
                 command=self.stop_drone).grid(row=1, column=1, padx=5, pady=5)
        
        # Right
        tk.Button(control_frame, text="▶\nD", bg='#4a4a4a', fg='white', **btn_style,
                 command=lambda: self.send_velocity(y=-self.linear_speed)).grid(row=1, column=2, padx=5, pady=5)
        
        # Backward
        tk.Button(control_frame, text="▼\nS", bg='#4a4a4a', fg='white', **btn_style,
                 command=lambda: self.send_velocity(x=-self.linear_speed)).grid(row=2, column=1, padx=5, pady=5)
        
        # Altitude Frame
        alt_frame = tk.Frame(self.root, bg='#2b2b2b')
        alt_frame.pack(pady=10)
        
        tk.Label(alt_frame, text="Altitude:", font=('Arial', 12), 
                bg='#2b2b2b', fg='white').pack(side='left', padx=10)
        
        tk.Button(alt_frame, text="↑ UP", bg='#4a9a4a', fg='white',
                 font=('Arial', 12), width=8,
                 command=lambda: self.send_velocity(z=self.altitude_speed)).pack(side='left', padx=5)
        
        tk.Button(alt_frame, text="↓ DOWN", bg='#9a4a4a', fg='white',
                 font=('Arial', 12), width=8,
                 command=lambda: self.send_velocity(z=-self.altitude_speed)).pack(side='left', padx=5)
        
        # Landing Button
        self.land_button = tk.Button(self.root, text="🛬 LAND", 
                                     font=('Arial', 18, 'bold'),
                                     bg='#666666', fg='white',
                                     width=15, height=2,
                                     state='disabled',
                                     command=self.start_landing)
        self.land_button.pack(pady=20)
        
        # Keyboard bindings
        self.root.bind('w', lambda e: self.send_velocity(x=self.linear_speed))
        self.root.bind('s', lambda e: self.send_velocity(x=-self.linear_speed))
        self.root.bind('a', lambda e: self.send_velocity(y=self.linear_speed))
        self.root.bind('d', lambda e: self.send_velocity(y=-self.linear_speed))
        self.root.bind('<space>', lambda e: self.stop_drone())
        self.root.bind('<Up>', lambda e: self.send_velocity(z=self.altitude_speed))
        self.root.bind('<Down>', lambda e: self.send_velocity(z=-self.altitude_speed))
        self.root.bind('<KeyRelease>', lambda e: self.stop_drone())
        
        # Start update loop
        self.update_gui()
        
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
    
    def update_gui(self):
        """Update GUI status based on alignment"""
        if self.is_landing:
            self.send_velocity(z=-0.2)  # Descend slowly
            self.status_label.config(text="🛬 LANDING...", fg='orange')
            self.land_button.config(state='disabled', bg='#666666')
        elif self.is_aligned:
            self.status_label.config(text="✅ DRONE READY TO LAND", fg='#00ff00')
            self.land_button.config(state='normal', bg='#00aa00')
        else:
            self.status_label.config(text="⏳ Aligning...", fg='yellow')
            self.land_button.config(state='disabled', bg='#666666')
        
        self.position_label.config(text=f"LED: ({self.led_x:.0f}, {self.led_y:.0f})")
        
        # Schedule next update
        if self.root:
            self.root.after(100, self.update_gui)
    
    def on_close(self):
        """Handle window close"""
        self.stop_drone()
        self.root.destroy()
        self.root = None


def main(args=None):
    rclpy.init(args=args)
    node = DroneControlGUI()
    
    # Create GUI in main thread
    node.create_gui()
    
    # Run ROS spin in a separate thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    # Run tkinter mainloop in main thread
    try:
        node.root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
