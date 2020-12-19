#!/usr/bin/env python
# -*- encoding: utf-8 -*-
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import numpy as np

def gstreamer_pipeline(
    capture_width=3280,
    capture_height=2464,
    display_width=820,
    display_height=616,
    framerate=21,
    flip_method=0,
):
    return (
        "nvarguscamerasrc ! "
        "video/x-raw(memory:NVMM), "
        "width=(int)%d, height=(int)%d, "
        "format=(string)NV12, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )

class Node(object):
    def __init__(self):
        # Params
        self.image = None
        self.br = CvBridge()
        # Node cycle rate (in Hz).
        self.loopRate = rospy.Rate(20)

        # Publisher
        self.pub = rospy.Publisher('RGBimage', Image,queue_size=10)

        # Subscribers
        #rospy.Subscriber("/camera/image_color",Image,self.callback)


    #def callback(self, msg):
    #    self.image = self.br.imgmsg_to_cv2(msg)

    ########################CAMERA CAPTURING####################################
    def start(self):
        rospy.loginfo("RGB image provider started")
        #srospy.spin()
        br = CvBridge()
        cap = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)
        if not cap.isOpened():
            while True:
                rospy.loginfo("Camera not available")
        while not rospy.is_shutdown():
            #rospy.loginfo('publishing image')
            #br = CvBridge()
            #getting the image
            ret, img = cap.read()
            self.pub.publish(br.cv2_to_imgmsg(img))
            self.loopRate.sleep()

if __name__ == '__main__':
    rospy.init_node("img_publisher", anonymous=True)
    my_node = Node()
    my_node.start()

