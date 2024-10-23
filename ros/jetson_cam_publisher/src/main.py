#!/usr/bin/env python

#https://stackoverflow.com/questions/55377442/how-to-subscribe-and-publish-images-in-ros
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import numpy as np
from jepture import NumpyStream


if __name__ == '__main__':
    rospy.init_node("jetsonImage", anonymous=True)
    br = CvBridge()
    # Node cycle rate (in Hz).
    loop_rate = rospy.Rate(1)
    # Publisher
    pub = rospy.Publisher("/camera/image_raw", Image,queue_size=10)

    stream = NumpyStream([(0,"camera")],resolution=(1920,1080),fps=10.0)

    rospy.loginfo("Sending images")
    while not rospy.is_shutdown():
        image = stream.next()[0].array

        rospy.loginfo("Published Image")
        if image is not None:
            pub.publish(br.cv2_to_imgmsg(image))
        loop_rate.sleep()
