#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Odometry

class OdomTransformBroadcaster(Node):
    def __init__(self):
        super().__init__('odom_broadcaster')
        
        # Create a TransformBroadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscribe to odometry messages
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',  # Change this to match your odometry topic
            self.odom_callback,
            10
        )
        
        self.get_logger().info('Odometry transform broadcaster started')

    def odom_callback(self, msg):
        # Create transform message
        transform = TransformStamped()
        
        # Set header
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        
        # Set translation from odometry
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        
        # Set rotation from odometry
        transform.transform.rotation = msg.pose.pose.orientation
        
        # Broadcast transform
        self.tf_broadcaster.sendTransform(transform)

        if self.count_subscribers('tf') > 0:
            self.get_logger().debug(f'Published transform: odom->base_link: '
                                f'pos:[{transform.transform.translation.x:.2f}, '
                                f'{transform.transform.translation.y:.2f}]')

def main():
    rclpy.init()
    node = OdomTransformBroadcaster()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()