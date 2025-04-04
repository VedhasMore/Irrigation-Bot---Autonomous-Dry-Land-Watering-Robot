#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import time

class IrrigationBot(Node):
    def __init__(self):
        super().__init__('irrigation_bot')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Subscribers
        self.laser_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            10)
        
        self.image_sub = self.create_subscription(
            Image,
            'image',
            self.image_callback,
            10)
        
        # Initialize variables
        self.cv_bridge = CvBridge()
        self.twist = Twist()
        self.obstacle_detected = False
        self.gray_area_detected = False
        self.spraying = False
        self.min_safe_distance = 0.5  # meters
        
        # Timer for movement control
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('Irrigation Bot initialized')
    
    def laser_callback(self, msg):
        """Process LiDAR data for obstacle detection"""
        # Check if there are obstacles within minimum safe distance
        # We'll focus on the front 180 degrees of the scan
        front_ranges = msg.ranges[0:60] + msg.ranges[300:359]
        if min(front_ranges) < self.min_safe_distance:
            self.obstacle_detected = True
            self.get_logger().info('Obstacle detected!')
        else:
            self.obstacle_detected = False
    
    def image_callback(self, msg):
        """Process camera data to detect gray (dry) areas"""
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Convert to grayscale
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            # Define gray (dry) area as pixels with intensity between 100 and 180
            # This is a simplified approach - you might need to adjust based on your environment
            mask = cv2.inRange(gray, 100, 180)
            
            # Count gray pixels
            gray_pixel_count = np.count_nonzero(mask)
            gray_percentage = gray_pixel_count / (cv_image.shape[0] * cv_image.shape[1])
            
            # If more than 20% of the image is gray, consider it a dry area
            self.gray_area_detected = gray_percentage > 0.2
            
            if self.gray_area_detected and not self.spraying:
                self.get_logger().info('Gray area detected! Starting irrigation.')
                self.start_spraying()
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')
    
    def control_loop(self):
        """Main control loop for the robot"""
        if self.obstacle_detected:
            # Stop and turn to avoid obstacle
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.3  # Turn left
        elif self.spraying:
            # Stay in place while spraying
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
        else:
            # Normal exploration mode
            self.twist.linear.x = 0.2  # Move forward at moderate speed
            self.twist.angular.z = 0.05  # Slight curve for exploration
        
        # Publish movement commands
        self.cmd_vel_pub.publish(self.twist)
    
    def start_spraying(self):
        """Start water spraying process"""
        self.spraying = True
        # Create a thread to handle the spraying timing
        threading.Thread(target=self.spray_water).start()
    
    def spray_water(self):
        """Simulate water spraying for a set duration"""
        # In a real system, this would activate water pumps or valves
        self.get_logger().info('Spraying water...')
        time.sleep(5.0)  # Spray for 5 seconds
        self.get_logger().info('Spraying complete')
        self.spraying = False

def main(args=None):
    rclpy.init(args=args)
    irrigation_bot = IrrigationBot()
    rclpy.spin(irrigation_bot)
    irrigation_bot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
