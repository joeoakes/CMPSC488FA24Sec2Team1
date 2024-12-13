#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class AutoNavigation(Node):
    def __init__(self):
        super().__init__("movement_publisher")
        self.kinect_sub = self.create_subscription(
            Image, "/kinect2/depth", self.process_image, 10
        )
        self.movement_pub = self.create_publisher(String, "/move_cmd", 1)
        self.movement = String()
        self.movement.data = "stop"
        # self.movement_pub.publish(")

    def process_image(self, image: Image):
        if image.encoding != "mono16":
            self.get_logger().warn("Image recieved is not of type 'mono16' discarding.")
            return
        


def main(args=None):
    rclpy.init(args=args)
    pub = AutoNavigation()
    rclpy.spin(pub)
