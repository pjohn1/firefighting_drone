#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from geometry_msgs.msg import Vector3
import termios
import tty
import select
import sys
import threading
import time

class DroneKeyboardController(Node):
    def __init__(self):
        super().__init__('drone_keyboard_controller')
        
        # Create publisher for force commands
        self.force_pub = self.create_publisher(Wrench, '/drone/force', 10)
        
        # Initialize force values
        self.force_x = 0.0
        self.force_y = 0.0
        self.force_z = 0.0
        self.torque_x = 0.0
        self.torque_y = 0.0
        self.torque_z = 0.0
        
        # Force and torque magnitudes
        self.force_magnitude = 10.0  # Newtons
        self.torque_magnitude = 1.0  # Newton-meters
        
        # Start keyboard listener thread
        self.keyboard_thread = threading.Thread(target=self.keyboard_listener)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()
        
        # Create timer for publishing force commands
        self.timer = self.create_timer(0.1, self.publish_force)
        
        self.get_logger().info('Drone keyboard controller started')
        self.get_logger().info('Use WASD keys for horizontal movement')
        self.get_logger().info('Use Q/E keys for vertical movement')
        self.get_logger().info('Use arrow keys for rotation')
        self.get_logger().info('Press Ctrl+C to exit')
    
    def keyboard_listener(self):
        """Listen for keyboard input and update force values"""
        while rclpy.ok():
            if select.select([sys.stdin], [], [], 0)[0]:
                key = sys.stdin.read(1)
                self.process_key(key)
    
    def process_key(self, key):
        """Process keyboard input and update force values"""
        if key == 'w':
            self.force_x = self.force_magnitude
        elif key == 's':
            self.force_x = -self.force_magnitude
        elif key == 'a':
            self.force_y = self.force_magnitude
        elif key == 'd':
            self.force_y = -self.force_magnitude
        elif key == 'q':
            self.force_z = self.force_magnitude
        elif key == 'e':
            self.force_z = -self.force_magnitude
        elif key == '\x1b':  # Escape key
            # Check for arrow keys
            if select.select([sys.stdin], [], [], 0.1)[0]:
                key = sys.stdin.read(1)
                if key == '[':
                    key = sys.stdin.read(1)
                    if key == 'A':  # Up arrow
                        self.torque_x = self.torque_magnitude
                    elif key == 'B':  # Down arrow
                        self.torque_x = -self.torque_magnitude
                    elif key == 'C':  # Right arrow
                        self.torque_y = self.torque_magnitude
                    elif key == 'D':  # Left arrow
                        self.torque_y = -self.torque_magnitude
        elif key == ' ':  # Space bar
            # Reset all forces and torques
            self.force_x = 0.0
            self.force_y = 0.0
            self.force_z = 0.0
            self.torque_x = 0.0
            self.torque_y = 0.0
            self.torque_z = 0.0
    
    def publish_force(self):
        """Publish force command to the drone"""
        msg = Wrench()
        msg.force = Vector3(x=self.force_x, y=self.force_y, z=self.force_z)
        msg.torque = Vector3(x=self.torque_x, y=self.torque_y, z=self.torque_z)
        self.force_pub.publish(msg)
        
        # Reset forces and torques after publishing
        self.force_x = 0.0
        self.force_y = 0.0
        self.force_z = 0.0
        self.torque_x = 0.0
        self.torque_y = 0.0
        self.torque_z = 0.0

def main(args=None):
    # Save terminal settings
    old_settings = termios.tcgetattr(sys.stdin)
    
    try:
        # Set terminal to raw mode
        tty.setraw(sys.stdin.fileno())
        
        rclpy.init(args=args)
        controller = DroneKeyboardController()
        rclpy.spin(controller)
        controller.destroy_node()
        rclpy.shutdown()
    finally:
        # Restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

if __name__ == '__main__':
    main() 