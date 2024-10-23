import cv2 as cv
import numpy as np

# Initialize video capture
cap = cv.VideoCapture(2)

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Check if frame is successfully captured
    if not ret:
        print("Failed to grab frame")
        break
    cv.imshow('Webcam', frame)
    k = cv.waitKey(1) & 0xFF
    if k == ord(' '):  # SPACE for escape
        break

# Release the capture and close all windows
cap.release()
cv.destroyAllWindows()
