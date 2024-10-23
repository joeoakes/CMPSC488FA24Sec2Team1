import cv2
from jepture import NumpyStream

stream = NumpyStream([(0,"camera")],resolution=(1920,1080),fps=10.0)

for i in range(20):
    frames = stream.next()
    print(frames[0].time_stamp, frames[0].number)
    cv2.imshow("Jepture image",frames[0].array);
