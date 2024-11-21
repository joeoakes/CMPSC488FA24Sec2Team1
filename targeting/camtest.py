import apriltag
import cv2
import visualizeArilTag
from typing import List
from turret import default_turret, Turret
from steppers.DRV8825 import DRV8825


def get_detection_by_id(
    detections: List[apriltag.Detection], id
) -> apriltag.Detection | None:
    filtered_detections = list(
        filter(lambda detection: detection.tag_id == id, detections)
    )
    if not filtered_detections:
        return None
    return max(filtered_detections, key=lambda detection: detection.goodness)


cam = cv2.VideoCapture(0)
options = apriltag.DetectorOptions(families="tag25h9")
detector = apriltag.Detector(options)
width, height = cam.read()[1].shape[:2]
img_center = (width // 2, height // 2)
# turret = default_turret()

motor1 = DRV8825(dir_pin=13, step_pin=19, enable_pin=12, mode_pins=(16, 17, 20))
motor2 = DRV8825(dir_pin=24, step_pin=18, enable_pin=4, mode_pins=(21, 22, 27))

theta_limits = (float("-inf"), float("inf"))
phi_limits = (float("-inf"), float("inf"))  # TODO: Check these bounds

turret = Turret(motor1, motor2, theta_limits, phi_limits)

while 1:
    success, img = cam.read()

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    result = detector.detect(gray)

    detection = None
    if result:
        detection = get_detection_by_id(result, 0)
    if detection:
        detection_center = (round(detection.center[0]), round(detection.center[1]))
        cv2.circle(img, detection_center, 10, (255, 0, 0), -1)

    # overlay = visualizeArilTag.visualize(img, result, detector, annotation=True)
    cv2.imshow("Camera", img)

    if cv2.waitKey(1) == ord("q"):
        break

    if not detection:
        continue

    if detection_center[0] - img_center[0] > 5:
        turret.rotate(-1, 0.0)

    if detection_center[0] - img_center[0] < 5:
        turret.rotate(1, 0.0)

    if detection_center[1] - img_center[1] > 5:
        turret.rotate(0.0, -1)

    if detection_center[1] - img_center[1] < 5:
        turret.rotate(0.0, 1)
