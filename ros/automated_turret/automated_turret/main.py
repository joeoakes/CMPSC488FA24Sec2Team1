import apriltag
import cv2
import rclpy
import time
import numpy as np
from typing import List
from rclpy.node import Node
from rclpy import logging
from rclpy.executors import ExternalShutdownException
from robot_interfaces.msg import TurretInstruction
from . import visualizeArilTag

DEAD_ZONE = 10
MAX_SPEED = 5.0

class TurretPublisher(Node):
    def __init__(self):
        super().__init__("automated_turret_pulisher")
        self.camera = cv2.VideoCapture(0)
        height, width, _ = self.camera.read()[1].shape
        self.camera_height = height
        self.camera_width = width
        self.img_center = (width // 2, height // 2)
        options = apriltag.DetectorOptions(families="tag25h9")
        self.detector = apriltag.Detector(options)

        self.turret_pub = self.create_publisher(TurretInstruction, "/turret_cmd", 1)

        self.get_logger().info("Automated Turret Started")
        self.run_turret()

    def get_detection_by_id(
        self, detections: List[apriltag.Detection], id
    ) -> apriltag.Detection | None:
        filtered_detections = list(
            filter(lambda detection: detection.tag_id == id, detections)
        )
        if not filtered_detections:
            return None
        return max(filtered_detections, key=lambda detection: detection.goodness)

    def run_turret(self):
        last_seconds = int(time.time())
        zerod = False
        while 1:
            turret = TurretInstruction()
            _, img = self.camera.read()

            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            result = self.detector.detect(gray)

            detection = None
            detection_center = None
            if result:
                detection = self.get_detection_by_id(result, 1)  # type: ignore
            if detection:
                detection_center = (
                    round(detection.center[0]),
                    round(detection.center[1]),
                )
                cv2.circle(img, detection_center, 10, (255, 0, 0), -1)

            cv2.circle(img, self.img_center, DEAD_ZONE, (0, 255, 0), -1)
            cv2.line(
                img,
                (self.camera_width // 2, self.camera_height),
                (self.camera_width // 2, 0),
                (0, 0, 0),
                thickness=5,
            )
            cv2.line(
                img,
                (self.camera_width, self.camera_height // 2),
                (0, self.camera_height // 2),
                (0, 0, 0),
                thickness=5,
            )

            visualizeArilTag.visualize(img, result, self.detector, annotation=True)

            img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
            cv2.imshow("Camera", img)
            cv2.waitKey(1)

            if not detection_center:
                if int(time.time()) - last_seconds > 3 and not zerod:
                    self.get_logger().info("zeroing...")
                    turret.zero_turret = True
                    zerod = True
                    self.turret_pub.publish(turret)
                continue

            distance = abs(
                np.linalg.norm(
                    [
                        self.img_center[0] - detection_center[0],
                        self.img_center[1] - detection_center[1],
                    ]
                )
            )

            speed = min(((distance * 0.01) ** 3) * MAX_SPEED, MAX_SPEED)
            speed = 0.1 if speed < 0.1 and speed > 0 else speed
            centered = True

            if detection_center[0] - self.img_center[0] > DEAD_ZONE:
                centered = False
                turret.theta = speed

            if detection_center[0] - self.img_center[0] < -DEAD_ZONE:
                centered = False
                turret.theta = -speed

            if detection_center[1] - self.img_center[1] > DEAD_ZONE:
                centered = False
                turret.phi = speed

            if detection_center[1] - self.img_center[1] < -DEAD_ZONE:
                centered = False
                turret.phi = -speed

            if centered:
                turret.laser_duration = 0.5

            self.get_logger().info(
                f"Turning theta: {turret.theta}, phi: {turret.phi}, firing: {turret.laser_duration}"
            )
            self.turret_pub.publish(turret)
            last_seconds = int(time.time())
            zerod = False


def main(args=None):
    rclpy.init(args=args)
    auto_turret = TurretPublisher()
    try:
        rclpy.spin(auto_turret)
    except (KeyboardInterrupt, ExternalShutdownException):
        logging.get_logger("logger").info("Shutting down automated turret")

    auto_turret.destroy_node()
