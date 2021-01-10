#!/usr/bin/env python

import rospy
from std_msgs.msg import String

import serial

class ArduinoTalker:
    
    def	__init__(self, _dev = '/dev/ttyACM0'):
        self.device = _dev
        self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout=10)
        #Publisher topics
        self.pubLvl = rospy.Publisher('batteryLvl', String, queue_size=10)
        self.pubEn = rospy.Publisher('encoders', String, queue_size=10)
        #starting the node
        rospy.init_node('comunicate', anonymous=True)
        rospy.loginfo('Comunicate node started')
        #subscriber to motor speeds values
        rospy.Subscriber("motor_spds", String, self.callback)

    def run(self):
        #if info available
        if self.ser.in_waiting:
            info = self.ser.readline()
            #rospy.loginfo(info)
            info = info[:-2].split(':')
            #print(info)
            if len(info)>2:
                self.pubLvl.publish(info[2])
                self.pubEn.publish(info[0]+':'+info[1])
            

    def callback(self, data):
        self.ser.write(str(data.data)+';')    

if __name__ == '__main__':
    com = ArduinoTalker()
    rate = rospy.Rate(50) # 100hz
    while not rospy.is_shutdown():
        com.run()
        rate.sleep()
