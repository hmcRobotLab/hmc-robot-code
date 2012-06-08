#!/usr/bin/env python
import roslib; roslib.load_manifest('Frizzle')
import rospy
import cv_bridge
import cv
import sensor_msgs.msg as sm
from std_msgs.msg import String
#import datetime
#import time

def talker():
    pub = rospy.Publisher('keyPress', String)
    rospy.init_node('talker')

    while not rospy.is_shutdown():
        cv.NamedWindow('keys')
        key_press = cv.WaitKey(5)  #get key input, wait at most 5 milliseconds
        key_press = key_press&255
        #print key_press # uncomment in order to see the number being printed
        if key_press == 255:
            output = ""
        else:
            output = chr(key_press)
        pub.publish(String(output))

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
