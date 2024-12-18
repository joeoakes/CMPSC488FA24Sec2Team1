import apriltag
import cv2
import rclpy
from typing import List
from rclpy.node import Node
from robot_interfaces.msg import TurretInstruction
from . import visualizeArilTag

DEAD_ZONE = 5


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
        turret = TurretInstruction()
        while 1:
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

            cv2.circle(img, self.img_center, 10, (0, 255, 0), -1)
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

            if cv2.waitKey(1) == ord("q"):
                break

            if not detection_center:
                continue

            if detection_center[0] - self.img_center[0] > DEAD_ZONE:
                turret.theta = 0.1

            if detection_center[0] - self.img_center[0] < DEAD_ZONE:
                turret.theta = -0.1

            if detection_center[1] - self.img_center[1] > DEAD_ZONE:
                turret.phi = 0.1

            if detection_center[1] - self.img_center[1] < DEAD_ZONE:
                turret.phi = -0.1

            self.get_logger().info("publishing turret")
            self.turret_pub.publish(turret)


def main(args=None):
    rclpy.init(args=args)
    auto_turret = TurretPublisher()
    rclpy.spin(auto_turret)
