import apriltag
import cv2
import visualizeArilTag

cam = cv2.VideoCapture(0)
options = apriltag.DetectorOptions(families="tag25h9")
detector = apriltag.Detector(options)
while 1:
    success, img = cam.read()

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    result = detector.detect(gray)

    overlay = visualizeArilTag.visualize(img, result, detector)
    cv2.imshow("Camera", overlay)

    if cv2.waitKey(1) == ord("q"):
        break
