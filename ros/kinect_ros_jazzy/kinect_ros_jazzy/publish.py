#!/usr/bin/env python
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import freenect


class ImagePublisher(Node):
    def __init__(self):
        super().__init__("kinect_publisher")
        self.vid_pub = self.create_publisher(Image, "/kinect2/rgb/image", 10)
        self.depth_pub = self.create_publisher(Image, "/kinect2/depth", 10)
        self.info_pub = self.create_publisher(
            CameraInfo, "/kinect2/rgb/camera_info", 10
        )

        timer_period = 0.1  # seconds

        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.br = CvBridge()

    def timer_callback(self):
        ret_image = self.get_image(freenect.sync_get_video, "rgb8")
        ret_depth = self.get_image(freenect.sync_get_depth, "mono16")

        if ret_image != None and ret_depth != None:
            self.vid_pub.publish(ret_image)
            self.depth_pub.publish(ret_depth)
            self.info_pub.publish(self.get_camera_topic())

    def get_camera_topic(self):
        camera_info = CameraInfo()
        camera_info.header.stamp = self.get_clock().now().to_msg()
        camera_info.width = 640
        camera_info.height = 480
        camera_info.distortion_model = "plumb_bob"
        camera_info.d = [
            0.012947732047700113,
            -0.016278227805096242,
            0.0020719045565612245,
            -0.0012254560479249,
            0.0,
        ]
        camera_info.k = [
            362.02061428537024,
            0.0,
            210.76517637501865,
            0.0,
            405.17059240104345,
            211.97321671296146,
            0.0,
            0.0,
            1.0,
        ]
        camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        camera_info.p = [
            361.9730224609375,
            0.0,
            209.32856710752822,
            0.0,
            0.0,
            405.724365234375,
            212.33942579256836,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
        ]
        return camera_info

    def get_image(self, get_function, format):
        ret_image = get_function()
        if ret_image != None:
            image, _ = ret_image
            image_msg = self.br.cv2_to_imgmsg(image, format)
            image_msg.header.stamp = self.get_clock().now().to_msg()
            image_msg.header.frame_id = "kinect_link"
            return image_msg

        return None


def main(args=None):
    logger = rclpy.logging.get_logger("logger")
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    try:
        logger.info("Started publishing images")
        rclpy.spin(image_publisher)
    except (KeyboardInterrupt, ExternalShutdownException):
        logger.info("Shutting down gracefully")
        pass

    image_publisher.destroy_node()
