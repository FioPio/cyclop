import numpy as np
import cv2
from matplotlib import pyplot as plt
from scipy.optimize import curve_fit

import rospy
import std_msgs.msg as msg

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
        rospy.init_node('node')
        rospy.Subscriber("points", msg.Int16MultiArray, callback)
        rospy. spin()

    def callback(self, msg):
        print('1')

    # rospy.spin()
    # self.pub = rospy.Publisher('motorSpds', Int16MultiArray, queue_size=10)

if __name__ == '__main__':
    my_node = Node()
