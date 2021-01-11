#!/usr/bin/env python

import numpy as np
import rospy
from std_msgs.msg import String
from path_pkg.msg import Point
from path_pkg.msg import Points

# Parameters
w = 380  # image width
h = 266  # image height

l = 0  # dist between wheel-axis and camera in pixels
R = 50  # dist between wheels in pixels

f = 50  # max forward spd (in counts per .1 sec)
K = 2.5  # factor for th-spd
I = 0# 0.5
D = 1#25
MAXACTION = 25
spin = 30

lad = 80  # look ahead distance (in pixels)


class Node(object):
    def __init__(self):
        self.lE = 0
        self.aE = 0

        self.last_goal = []
        self.lastT = rospy.get_time()

        rospy.init_node('node')
        rospy.loginfo("point2arduino node started")

        self.pub = rospy.Publisher('motor_spds', String, queue_size=10)

        rospy.Subscriber("path_points", Points, self.callback)

    def callback(self, msg):
        # Flip
        points = msg.data
        points.reverse()

        t = rospy.get_time()
        if (t - lastT) > 0.03:
            self.lost()
        else:
            # Determine goal point
            goal = []
            i = 0
            for point in points:
                x0 = point.x
                y0 = point.y

                y = (w / 2) - y0
                x = h - x0 + l
                d = np.sqrt(x * x + y * y)
                if d < lad:
                    i += 1
                    goal = (x, y, d)
                else:
                    break
                last_goal = goal
            if len(goal)==3:
                x = goal[0]
                y = goal[1]
                d = goal[2]

                #print("POINT " + str(i))
                theta = np.arccos(x / d)

                spdF = f - 2*f/3 * abs(theta)
                theta = np.sign(y) * (theta)    # np.sign(y) * a * spdMod

                action = K * theta + D * (theta - self.lE) + I * self.aE
                self.lE = theta
                if action < MAXACTION and action > -MAXACTION:
                    self.aE += theta
                lmspd = spdF - action
                rmspd = spdF + action

                # Publish
                str_out = str(lmspd) + ':' + str(rmspd)
                self.pub.publish(str_out)

            self.lastT = t;

    # Rotates the robot if it is lost, based on previous goal
    def lost(self):
        if last_goal.y > 0:
            lmspd = 0
            rmspd = spin
        else:
            lmspd = spin
            rmspd = 0

        # Publish
        str_out = str(lmspd) + ':' + str(rmspd)
        self.pub.publish(str_out)


if __name__ == '__main__':
    my_node = Node()
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        rate.sleep()
