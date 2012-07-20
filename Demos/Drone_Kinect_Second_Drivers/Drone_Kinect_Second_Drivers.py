#!/usr/bin/env python

import roslib; roslib.load_manifest('ardrone2_mudd')
import rospy
import tf
from tf.transformations import euler_from_quaternion
import sensor_msgs.msg as sm
import cv_bridge
import cv
from std_msgs.msg import String
from math import *
import time, threading
import os, random

import Ardrone2
from Ardrone2 import rospy

######################## INSTRUCTIONS ###########################
#
# Be connected to drone.
# Run './magic.sh start'
# Run 'rosrun openni_tracker openni_tracker'
# Get User 1 recognized.
# Get User 1 calibrated.
# Run 'python combine.py'
#
# If too slow after having run before, try running 
# './magic.sh stop'
# stop the openni_tracker
# and try again from running './magic.sh start'
#
#################################################################

########################### INITIALIZATION FUNCTIONS #############################

class myArdrone(Ardrone2.Ardrone):
    def __init__(self):
        Ardrone2.Ardrone.__init__(self)

        # Create cv stuff
        print "Started Making a Window"
        cv.NamedWindow('skeleton')
        cv.MoveWindow('skeleton', 0, 0)
        cv.SetMouseCallback('skeleton', self.onMouse, None)
        print 'Finished Making a Window'

        self.sk_image = cv.CreateImage((640,480),8,1)
        cv.ShowImage('skeleton', self.sk_image)

        ##### Initial Right Hand Info ####
        self.hand_right_x = 0
        self.hand_right_y = 0
        self.hand_right_size = 10

        ##### Initial Left Hand Info ####
        self.hand_left_x = 0
        self.hand_left_y = 0
        self.hand_left_size = 10

        ##### Initial Right Elbow Info ####
        self.elbow_right_x = 0
        self.elbow_right_y = 0

        ##### Initial Left Elbow Info ####
        self.elbow_left_x = 0
        self.elbow_left_y = 0

        ##### Initial Right Shoulder Info ####
        self.shoulder_right_x = 0
        self.shoulder_right_y = 0

        ##### Initial Left Shoulder Info ####
        self.shoulder_left_x = 0
        self.shoulder_left_y = 0

        #### Initial Head Info ####
        self.head = (0,0)

        #### Other Initials ####
        self.state = "Keyboard"
        self.vel = "nothing"
        self.drone_speed = 0.15
        #self.airborne = False
    


        # member variables relating to the drone's control
        self.use_drone = True

        
        #D["run_without_data_stream"] = False
        #D["battery_level"] = "Not read from drone"


            
        # images and other data for the images/windows
        self.bridge = cv_bridge.CvBridge()  # the interface to OpenCV
        self.color_image = None             # the image from the drone
        self.new_image = False              # did we just receive a new image?
        self.threshed_image = None          # thresholded image
        self.font = cv.InitFont(cv.CV_FONT_HERSHEY_PLAIN, 1, 1, 0, 1)
        self.contours_found = False

        
        # variables for state machine
        self.reset_needed = False
        self.begin_end = False

    ############################ IMAGE DRAWING FUNCTIONS #############################

    def drawCircles(self):
        """ Draw circles on the skeleton image """
        cv.Circle(self.sk_image, self.head, 10, cv.RGB(100, 100, 100), 
                  thickness=1, lineType=8, shift=0)
        cv.Circle(self.sk_image, (self.hand_left_x,self.hand_left_y), 
                  self.hand_left_size, cv.RGB(125, 125, 125), 
                  thickness=1, lineType=8, shift=0)
        cv.Circle(self.sk_image, (self.hand_right_x,self.hand_right_y), 
                  self.hand_right_size, cv.RGB(125, 125, 125), 
                  thickness=1, lineType=8, shift=0)
        cv.Circle(self.sk_image, (self.shoulder_left_x,self.shoulder_left_y),
                  10, cv.RGB(255, 255, 255), thickness=1, lineType=8, shift=0)
        cv.Circle(self.sk_image, (self.shoulder_right_x,self.shoulder_right_y), 10, 
                  cv.RGB(255, 255, 255), thickness=1, lineType=8, shift=0)
        cv.Line(self.sk_image, (self.shoulder_right_x,self.shoulder_right_y), 
                (self.hand_right_x,self.hand_right_y), 
                cv.RGB(255, 255, 255), thickness = 1, lineType = 0, shift = 0)
        cv.Line(self.sk_image, (self.shoulder_left_x,self.shoulder_left_y),
                (self.hand_left_x,self.hand_left_y), 
                cv.RGB(255, 255, 255), thickness = 1, lineType = 0, shift = 0)


    def printImage(self):
        """ Draw on and update the skeleton image """
        self.drawCircles()
        # display the image
        cv.ShowImage('skeleton', self.sk_image)

    # handles the drone's data stream (separate from the video and control)

    # put text on the image...
    def text_to_image(self):
        """ write various things on the image ... """
        # the image is 320 pixels wide and 240 pixels high
        # clear a rectangle
        cv.Rectangle(self.color_image, (3,3), (242,30),
                     cv.RGB(255,255,255), cv.CV_FILLED )
                        
        # set up some text           
        llx = 7
        lly = 22
        s = "State: " + str(self.state)
        textllpoint = (llx,lly)
        cv.PutText(self.color_image, s, textllpoint, self.font, cv.RGB(0,0,255))

    ########################## END IMAGE DRAWING FUNCTIONS ###########################

    ################################ INPUT FUNCTIONS #################################

    def onMouse(self,event, x, y, flags, param):
        """ the method called when the mouse is clicked """
        # if the left button was clicked
        if event==cv.CV_EVENT_LBUTTONDOWN and event==cv.CV_EVENT_RBUTTONDOWN:
            print "landing and quitting"
            self.begin_end = True
            self.land()
            rospy.sleep(1.42)
            print "Stop and Quit"
            exit(0)
            return

    ############# WHERE THE KEYPRESSES GET HANDLED! #########
    def get_Drone_Kinect_Key_Presses(self):
        key_press = cv.WaitKey(5) & 255
        if key_press != 255:
            self.key_press_func( key_press )
    ############# WHERE THE KEYPRESSES GET HANDLED! #########
            

    ############################## END INPUT FUNCTIONS ###############################

    ################################### MAIN LOOP ####################################

    # trans is the delta in X, Y, and Z from origin to target
    # rot is the pitch, roll, yaw, and MYSTERY to get origin to align with target
    # elbow is weird. :D

    def begin(self):
        print "Hello, MyCS is working!"
        ##rospy.init_node('bodytrak', anonymous=True)
        listener = tf.TransformListener()
        h = '/head_1'
        rs = '/right_shoulder_1'
        rh = '/right_hand_1'
        re = '/right_elbow_1'
        ls = '/left_shoulder_1'
        lh = '/left_hand_1'
        le = '/left_elbow_1'

        while not self.begin_end:
            drone.get_Drone_Kinect_Key_Presses()
            try:
                #hand to shoulder transforms
                listener.waitForTransform(rh,rs,rospy.Time(0),rospy.Duration(10.0))
                rstrans, rsrot = listener.lookupTransform(rs, h, rospy.Time(0))
                lstrans, lsrot = listener.lookupTransform(ls, h, rospy.Time(0))
                rtrans, rrot = listener.lookupTransform(rh, rs, rospy.Time(0))
                ltrans, lrot = listener.lookupTransform(lh, ls, rospy.Time(0))
                retrans,rerot = listener.lookupTransform(re, rs, rospy.Time(0))
                letrans,lerot = listener.lookupTransform(le, rs, rospy.Time(0))

            except (tf.LookupException, tf.ConnectivityException):
                continue # This happens while we wait for a target
            
            self.head = (320, 240)
            self.shoulder_right_x = int(300-rstrans[0]*50)
            self.shoulder_left_x = int(340-lstrans[0]*50)
            self.shoulder_right_y = int(240-rstrans[1]*100)
            self.shoulder_left_y = int(240-lstrans[1]*100)
            self.hand_right_x= int(self.shoulder_right_x-rtrans[0]*200)
            self.hand_left_x= int(self.shoulder_left_x-ltrans[0]*200)
            self.hand_right_y= int(self.shoulder_right_y+rtrans[1]*240)
            self.hand_left_y= int(self.shoulder_left_y+ltrans[1]*240)
            self.hand_left_size = int(10+ltrans[2]*10)
            self.hand_right_size = int(10+rtrans[2]*10)
            hx = int(self.shoulder_right_x + self.shoulder_left_x)/2
            hy = int(self.shoulder_right_y + self.shoulder_left_y)/2
            self.head = (hx,hy-80)
                
            self.printImage()

            # display the image
            cv.ShowImage('skeleton', self.sk_image)
            # done!
            cv.SetZero(self.sk_image)

    #        print "ltrans[0], rtrans[0]", ltrans[0], rtrans[0]
    #        print "ltrans[1], rtrans[1]", ltrans[1], rtrans[1]
    #        print "ltrans[2], rtrans[2]", ltrans[2], rtrans[2]

            ##################################### INTERPRET GESTURES ###############################
            # if arms behind you
            if rtrans[2] < -0.30 and ltrans[2] < -0.30 and self.vel != "backwards":
                self.vel = "backwards"
                print "backwards"
                if self.airborne:
                    self.backward(self.drone_speed)
        

            elif rtrans[2] > 0.30 and ltrans[2] > 0.30 and self.vel != "going_forward": # if arms dirctly in fornt of you
                self.vel = "going_forward"
                print "forward"
                if self.airborne:
                    self.forward(self.drone_speed)

            # if arms are up
            elif ltrans[1]<-0.35 and rtrans[1]<-0.35 and \
                    self.vel != "up":
                self.vel = "up"
                print "Up"
                if self.airborne:
                    self.up(self.drone_speed)

            # of arms are down
            elif ltrans[1]> 0.45 and rtrans[1]>0.45 and \
                    self.vel != "down":
                self.vel = "down"
                print "Down"
                if self.airborne:
                    self.up(self.drone_speed)

            # if right arm up
            elif ltrans[1]< -0.35 and rtrans[1]>0.35 and \
                    self.vel != "right":
                self.vel = "right"
                print "right"
                if self.airborne:
                    self.strafeRight(self.drone_speed)

            # if left arm up
            elif ltrans[1]> 0.35 and rtrans[1]<-0.35 and \
                    self.vel != "left":
                self.vel = "left"
                print "left"
                if self.airborne:
                    self.strafeLeft(self.drone_speed)

            # Arms straight out to side
            elif abs(-rtrans[1]) < 0.15 and abs(-ltrans[1]) < 0.15 and \
                    abs(-rtrans[0]) > 0.45 and abs(-ltrans[0]) > 0.45 and \
                    not self.airborne and not self.reset_needed:
               self.airborne = True
               print "takeoff"
               self.takeoff()

            # right arm straight out, left arm down
            elif abs(-rtrans[1]) < 0.15 and ltrans[1]> 0.45 and self.airborne and self.vel != "turning_right":
                self.vel = "turning_right"
                print "rotate right"
                self.spinRight(self.drone_speed)

            # left arm straight out, right arm down
            elif abs(-ltrans[1]) < 0.15 and rtrans[1]> 0.45 and self.airborne and self.vel != "turning_left":
                self.vel = "turning_left"
                print "rotate left"
                self.spinLeft(self.drone_speed)

            # hands near belly
            elif abs(ltrans[0]) < 0.2 and abs(rtrans[0]) < 0.2 and \
                    abs(ltrans[1]) < 0.2 and abs(rtrans[1]) < 0.2 and \
                    abs(ltrans[2]) < 0.2 and abs(rtrans[2]) < 0.2 and self.airborne:
                print "landing"
                self.land()
                self.reset_needed = True

            # Arms straight out to side and already taken off
            elif abs(-rtrans[1]) < 0.15 and abs(-ltrans[1]) < 0.15 and \
                    abs(-rtrans[2]) < 0.15 and abs(-ltrans[2]) < 0.15 and \
                    self.airborne and self.vel != "hovering":
                self.vel = "hovering"
                print "hover"
                self.hover()

            
    def key_press_func(self,c):
        """ gets key_press c and deals with it """
        # handle the keypress to quit...
        print c
        if c == 27: # the Esc key is 27
            self.begin_end = True
            self.land()
            rospy.sleep(1.42)
            print "Stop and Quit"
            exit(0)
            return
                
        elif c == ord('+'):
            self.begin()

        else:
            self.keyCmd(chr(c))

                      

################################# END MAIN LOOP ##################################

if __name__ == '__main__':
    '''
    Main driver function for the drone.
    '''
    drone = myArdrone()
    #use_drone = True

    # Display initial message
    print
    print 'Extra Keyboard controls:\n'
    print
    print 'esc -- lands and quits'
    print '+ -- starts the moving control'


    rospy.sleep(1)

    while True:
        drone.get_Drone_Kinect_Key_Presses()
    # start the main loop, the keyboard thread:
    print "the Python program is quitting..."
