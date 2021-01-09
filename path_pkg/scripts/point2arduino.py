import numpy as np
import cv2
from matplotlib import pyplot as plt
from scipy.optimize import curve_fit

import rospy
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Int32


newMsg = False
arr = []

w = 3280
h = 2464

# Parameters
l = 0 # dist between wheel-axis and camera in pixels
R = 730 # dist between wheels in pixels

f = 300 # max forward spd
a = 2   # factor for th-spd

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
        nodeName = rospy.get_name()
        rospy.loginfo("%s started" % nodeName)

        self.pubL = rospy.Publisher('inLM', Int32, queue_size=10)
        self.pubR = rospy.Publisher('inRM', Int32, queue_size=10)

        rospy.Subscriber("points", Int16MultiArray, self.callback)

    def start(self):
        global newMsg, arr
        i = 0
        while not rospy.is_shutdown():
            if newMsg:
                i = 0
                self.calc(arr, i)
                newMsg = False
            else:
                i = i + 1
                self.calc(arr, i)

    def callback(self, msg):
        global newMsg, arr
        newMsg = True
        arr = msg

    def calc(self, msg, i):

        # CHECK INPUT
        point = msg[i]
        x0 = point[0]
        y0 = point[1]

        x = x0 - (w / 2)
        y = h + l + y0
        d = np.sqrt(x * x + y * y)

        theta = np.sign(x) * np.arccos(y/d)

        spdF = f - (f/(w/2)*abs(x))
        spdA = theta*a

        lmspd = spdF + spdA
        rmspd = spdF - spdA

        self.pubL.publish(lmspd)
        self.pubR.publish(rmspd)
        # PUBLISH

    #
    # rospy.spin()


if __name__ == '__main__':
    my_node = Node()
    my_node.start()
