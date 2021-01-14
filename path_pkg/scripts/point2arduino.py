#!/usr/bin/env python

""" A ROS-node that uses a Points structure to calculates appropriate motor speeds to send
 to the arduino.

This node is subscribed to the topic "path_points". When a message is received, the
look-ahead distance is used to choose an appropriate goal point. Then an approach based on
the follow-the-carrot algorithm is used together with a PD-controller to calculate two
reference speeds: one for the left and one for the right motor. The speeds are in the unit
of encoder-counts per 0.1 seconds. These are then published to the "motor_spds" topic
as a string of the form "left_motor_speed:right_motor_speed;".

"""
# Import necessary libraries
import numpy as np
import rospy
from std_msgs.msg import String
from path_pkg.msg import Point
from path_pkg.msg import Points

# Parameters
w = 380  # image width
h = 266  # image height

l = 0  # dist between wheel-axis and camera in pixels (NOT USED)
R = 50  # dist between wheels in pixels (NOT USED)

f = 40  # max forward spd (in counts per .1 sec)
MAXACTION = 20
lad = 80  # look-ahead distance (in pixels)

# PID parameters
K = 2.5  # factor for th-spd, proportional part of the PID
I = 0  # integer part of PID (NOT USED)
D = 1  # derivative part pf PID


class Node(object):
    def __init__(self):
        self.lE = 0
        self.aE = 0
        self.go_ahead = 1

        # Set up ROS
        rospy.init_node('node')
        rospy.loginfo("point2arduino node started")

        # Create publisher, that will publish to the topic "motor_spds"
        self.pub = rospy.Publisher('motor_spds', String, queue_size=10)

        # Subscribe to the topic "path_points" to receive Points
        rospy.Subscriber("path_points", Points, self.callback)

    # Callback function called by subscriber when receiving a message
    def callback(self, msg):
        # Flip array to get closest point first
        points = msg.data
        points.reverse()

        # Determine goal point
        goal = []
        for point in points:
            x0 = point.x
            y0 = point.y

            # Calculate distance to point
            y = (w / 2) - y0
            x = h - x0 + l
            d = np.sqrt(x * x + y * y)
            # Compare distance to look ahead distance
            if d < lad:
                goal = (x, y, d)
            else:
                break
        # Prevents faulty data/messages
        if len(goal) == 3:
            x = goal[0]
            y = goal[1]
            d = goal[2]

            # Angle to goal point
            theta = np.sign(y) * np.arccos(x / d)

            # Set a base forward speed
            spdF = f - 2 * f / 3 * abs(theta)
            spdF *= (d / lad)

            # Apply PID
            action = K * theta + D * (theta - self.lE) + I * self.aE
            self.lE = theta
            # Anti-windup
            if MAXACTION > action > -MAXACTION:
                self.aE += theta
            # Set motor speeds
            lmspd = spdF - action
            rmspd = spdF + action

            # Convert to string and publish
            str_out = str(lmspd) + ':' + str(rmspd)
            self.pub.publish(str_out)


if __name__ == '__main__':
    my_node = Node()
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        rate.sleep()
