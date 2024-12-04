#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class SimpleOdomPublisher(Node):
    def __init__(self):
        super().__init__('simple_odom_publisher')
        self.publisher = self.create_publisher(Odometry, 'odom', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('Simple odometry publisher started')

    def timer_callback(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        # All other fields will be zero by default
        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = SimpleOdomPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()