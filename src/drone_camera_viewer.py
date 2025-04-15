#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class DroneCameraViewer(Node):
    def __init__(self):
        super().__init__('drone_camera_viewer')
        
        # Create a subscriber to the camera topic
        self.subscription = self.create_subscription(
            Image,
            '/drone/camera/image_raw',
            self.image_callback,
            10)
            
        self.bridge = CvBridge()
        self.get_logger().info('Drone Camera Viewer has been started')
        
    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Display the image
            cv2.imshow('Drone Camera View', cv_image)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    viewer = DroneCameraViewer()
    
    try:
        rclpy.spin(viewer)
    except KeyboardInterrupt:
        pass
    finally:
        viewer.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main() 