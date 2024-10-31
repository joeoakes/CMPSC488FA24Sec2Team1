#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import freenect
import cv2
import numpy as np

class ImagePublisher(Node):
    def __init__(self):
        super().__init__("kinect_publisher")
        self.publisher_ = self.create_publisher(Image, 'video_frames', 10)
          
        timer_period = 0.1  # seconds
          
        self.timer = self.create_timer(timer_period, self.timer_callback)
             
        self.br = CvBridge()

    def timer_callback(self):
        ret = freenect.sync_get_video()
              
        if ret != None:
            frame, timestamp = ret
            self.publisher_.publish(self.br.cv2_to_imgmsg(frame))
     
        self.get_logger().info('Publishing video frame')

def main(args=None):
  rclpy.init(args=args)
  image_publisher = ImagePublisher()
  rclpy.spin(image_publisher)
  image_publisher.destroy_node()
  rclpy.shutdown()
