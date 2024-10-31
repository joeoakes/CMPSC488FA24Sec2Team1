#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import freenect
import cv2
import numpy as np

class ImagePublisher(Node):
    def __init__(self):
        super().__init__("kinect_publisher")
        self.vid_pub = self.create_publisher(Image, '/kinect2/rgb/image', 10)
        self.depth_pub = self.create_publisher(Image, '/kinect2/depth', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/kinect2/rgb/camera_info', 10)
          
        timer_period = 0.1  # seconds
          
        self.timer = self.create_timer(timer_period, self.timer_callback)
             
        self.br = CvBridge()

    def timer_callback(self):
        ret_video = freenect.sync_get_video()
        ret_depth = freenect.sync_get_depth()
              
        if ret_video != None and ret_depth != None:
            video, _ = ret_video
            depth, _ = ret_depth
            camera_info = CameraInfo()
            camera_info.width = 1920
            camera_info.height = 1080
            self.vid_pub.publish(self.br.cv2_to_imgmsg(video))
            self.depth_pub.publish(self.br.cv2_to_imgmsg(depth))
            self.info_pub.publish(camera_info)
     
        self.get_logger().info('Publishing video video')

def main(args=None):
  rclpy.init(args=args)
  image_publisher = ImagePublisher()
  rclpy.spin(image_publisher)
  image_publisher.destroy_node()
  rclpy.shutdown()
