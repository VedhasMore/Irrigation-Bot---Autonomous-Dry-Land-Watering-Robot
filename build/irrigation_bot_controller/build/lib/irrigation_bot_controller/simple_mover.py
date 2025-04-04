#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class SimpleMover(Node):
    def __init__(self):
        super().__init__('simple_mover')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.move_forward)

    def move_forward(self):
        msg = Twist()
        msg.linear.x = 0.3   # Forward speed
        msg.angular.z = 0.0  # No rotation
        self.publisher_.publish(msg)
        self.get_logger().info('Moving forward...')

def main(args=None):
    rclpy.init(args=args)
    node = SimpleMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
