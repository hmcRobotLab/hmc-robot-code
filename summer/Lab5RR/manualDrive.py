#!/usr/bin/env python
import roslib; roslib.load_manifest('Frizzle')
import rospy
import irobot_create_2_1
from std_msgs.msg import String
from irobot_create_2_1.srv import *
from irobot_create_2_1.msg import *

from nav_msgs.msg import Odometry
from geometry_msgs.msg import *
from tf.msg import *
import sys, select
import math

def callback(data):
    global speed

    key_press = data.data

    #code won't continue until it "hears" the tank 'tank'.  This topic lets 
    # our code, here, send requests to the service, 'tank'.
    rospy.wait_for_service('tank')

    # move will communicate with the Tank function that is in the service, 'tank'
    move = rospy.ServiceProxy('tank', Tank)

    if key_press == 'q' or key_press == chr(27): # if a 'q' or Esc was pressed
        print 'quitting'
        rospy.signal_shutdown( "Quit requested from keyboard" )
    if key_press == 'R':
        print "Forward"
        move(False, speed, speed)
    if key_press == 'T':
        print "Reverse"
        move(False, -speed, -speed)
    if key_press == 'S':
        print "Right"
        move(False, speed, -speed)
    if key_press == 'Q':
        print "Left"
        move(False, -speed, speed)
    if key_press == 's':
        print "Singing"
        # Need to figure out the song stuff here
    if key_press == 'b':
        print "Braking"
        move(False, 0, 0)
    if key_press == 'r':
        print "Resetting"
        move(False, 0, 0)
    if key_press == 'f':
        print "Speeding up"
        speed = 150
    if key_press == 'g':
        print "Slowing Down"
        speed = 50

def listener():
    global speed
    speed = 50
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('keyPress', String, callback)


if __name__ == '__main__':
    """
    this simply runs listener
    """
    listener() # initializes code to listen for the dock
    rospy.spin() #this stops the code from completing.  It will allow other 
        #processes, like listening, to happen, and the code won't end.
