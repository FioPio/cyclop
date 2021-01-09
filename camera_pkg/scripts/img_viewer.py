#!/usr/bin/env python
# -*- encoding: utf-8 -*-
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import numpy as np
from time import time as t

class Node(object):
    
    def __init__(self):
        rospy.init_node("img_viewer", anonymous=True)
        # Params
        self.image = None
        self.br = CvBridge()
        self.lastT = t()

        # Subscribers
        rospy.Subscriber("/RGBimage",Image,self.callback)
        rospy.spin()


    def callback(self, msg):
        self.image = self.br.imgmsg_to_cv2(msg)
        if t()-self.lastT > 1:
            self.lastT = t()
        cv2.imshow("viewer", self.image)
        if cv2.waitKey(100)==27:
            exit()


if __name__ == '__main__':
    my_node = Node()
