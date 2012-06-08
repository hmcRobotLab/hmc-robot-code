#!/usr/bin/env python
import roslib; roslib.load_manifest('Daneel')
import rospy
import tf
import cv_bridge
import cv
from tf.transformations import euler_from_quaternion
import irobot_create_2_1
from std_msgs.msg import String
from irobot_create_2_1.srv import *
from irobot_create_2_1.msg import *
from math import *

#import basics

### EDITED BY MAIA VALCARCE AND KEIKO HIRANAKA
### MARCH 2012

# To use this script run
# First tab:
#    roscore
# Second tab:
#    rosrun openni_camera openni_node
# Third tab:
#    rosrun openni_tracker openni_tracker
# Following tabs:
#    connect to the robot
# Then run this code with python filename.py

# trans is the delta in X, Y, and Z from origin to target
# rot is the pitch, roll, yaw, and MYSTERY to get origin to align with target

def begin():
   print "Hello, MyCS is working!"
   listener = tf.TransformListener()

   rs = '/right_shoulder_1'
   rh = '/right_hand_1'
   ls = '/left_shoulder_1'
   lh = '/left_hand_1'
   
   while not rospy.is_shutdown():
       try:
           #hand to shoulder transforms
           listener.waitForTransform(rh,rs,rospy.Time(0),rospy.Duration(4.0))
           rtrans, rrot = listener.lookupTransform(rh, rs, rospy.Time(0))
           ltrans, lrot = listener.lookupTransform(lh, ls, rospy.Time(0))

       except (tf.LookupException, tf.ConnectivityException):
          continue # This happens while we wait for a target


       # Should we stop?
       # If both the person's hands have moved significantly in the shoulder's
       # z direction, then we stop the robot. Based on testng I've
       # found the z direction is pointing away from the camera.

       #########################
       ###STOPPING THE SCRIPT###
       #########################
       '''
       Default script stopping:
       Put both hands straight out in front of you to stop the script.
       '''

       if rtrans[2] > 0.35 and ltrans[2] > 0.35:
           print "Stop and Quit"
           basics.drive(0)
           exit(0)


       #############################
       ###CONTROL BY TRANSLATIONS###
       #############################
       '''
       Default controls for movement:
       Arms straight out to the side at shoulder height to stop the robot
       Arms up to go forward, down to go backward
       Right and left arms control right and left wheel speeds independently.
       '''

       rt = -rtrans[1]
       lt = -ltrans[1]

       STOPTOLERANCE = 0.15
       if abs(rt) < STOPTOLERANCE and abs(lt) < STOPTOLERANCE :
           lt = 0
           rt = 0

       basics.tank(lt, rt)


def main():
    """ the main program that sets everything up
    """

    # Initialize our node, variables
    rospy.init_node('bodytrak', anonymous=True)

if __name__ == '__main__':
   main();
