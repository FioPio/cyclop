#!/usr/bin/env python


#using the help of : https://www.youtube.com/watch?v=Tm_7fGolVGE (visited on 10/12/2020)

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from path_pkg.msg import Point
from path_pkg.msg import Points


import numpy as np
import cv2
import math

HEIGHT  = 700 * 0.5
WIDTH   = 460 * 0.5

REMOVEUPTO = 175
MAXVALUE = 100

PRECISION = 5
OMITTING = 3

#cantonades a la imatge
pts1 = np.float32([ [ 260, 124 ],
                    [ 527, 116 ],
                    [ 695, 174 ],
                    [  61, 198 ] ])

#cantonades reals
pts2 = np.float32( [ [ WIDTH *0.9 ,      0 ],
                     [ WIDTH * 1.9,      0 ],
                     [ WIDTH * 1.9, HEIGHT ],
                     [ WIDTH * 0.9, HEIGHT ]] )


#get the transformation matrix
matrix = cv2.getPerspectiveTransform( pts1, pts2)

#apply the transformation imatge
finalSize = ( int(WIDTH*3.07), int(HEIGHT*1.59) ) #(820 , 616)

def dist( m, n, p):
    num = abs(m*p[0]- p[1] + n)
    den = math.sqrt( m*m +1 )

    return num/den

class PathFinder:
    def __init__(self):
        self.image = None
        self.br = CvBridge()
        self.pub = rospy.Publisher('path_points', Points)
        rospy.init_node('path_finding', anonymous=True)
        rospy.loginfo('path_findig node started')
        #subscriber to images
        rospy.Subscriber("/RGBimage",Image,self.getPath)

    def getPath(self, imag):
        #getting the image
        img    = self.br.imgmsg_to_cv2(imag)
        #trsnforms it into grayscale
        out = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        #equalizes it
        #out = cv2.equalizeHist(out)
        #removes upper part
        out[ :REMOVEUPTO, : ] = 255
        #inrange function
        out = cv2.inRange(out, 0, MAXVALUE)
        #filter
        kernel = np.ones((5,5),np.uint8)
        out = cv2.erode(out ,kernel,iterations = 1)
        out = cv2.dilate(out,kernel,iterations = 3)
        #top view
        out = cv2.warpPerspective( out, matrix, finalSize)[290:,150:530]
        #getting the points
        out_h=out.shape[0]
        points = []
        pointsRev = []
        #getting the contours of each subsection
        max_iterations = out_h//PRECISION
        for i in range(0,  max_iterations, OMITTING):
            contours, hierarchy = cv2.findContours(out[i*PRECISION:(i+1)*PRECISION,:], cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            pointsI = []
            #Getting the center of the contourns
            for contour in contours:
                x = 0
                y = 0
                for point in contour:
                    x += point[0] [1]
                    y += point[0] [0]
                x/= len(contour)
                y/= len(contour)
                x+= i*PRECISION
                #if just one contourn add it
                if len(contours)==1:
                    points.append((int(y),int(x)))
                else:
                    pointsI.append((int(y),int(x)))
            if pointsI:
                lenP = len(points)
                if lenP > 1:
                    p1 = points[lenP-1]
                    p2 = points[lenP-2]
                    if p2[0] - p1[0] != 0:
                        m = (p2[1] - p1[1]) / (p2[0] - p1[0])
                    else:
                        m = (p2[1] - p1[1]) *100000.0
                    n = p1[1] - m * p1[0]
                    dists = []
                    for i in range( len(pointsI)):
                        dists.append( dist(m, n, pointsI[i]))
                    shortestD = min(dists)
                    points.append(pointsI[dists.index(shortestD)])
        #points has the variable with the path
        if len(points)>0:
            msg = Points()
            for point in points:
                p = Point()
                p.x = point[0]
                p.y = point[1]
                msg.data.append(p)
            self.pub.publish(msg)

if __name__ == '__main__':
    pth = PathFinder()
    rate = rospy.Rate(100) # 100hz
    while not rospy.is_shutdown():
        rate.sleep()
