#!/usr/bin/env python

import roslib; roslib.load_manifest('Daneel')
import rospy
import tf
from tf.transformations import euler_from_quaternion
import sensor_msgs.msg as sm
import cv_bridge
import cv
import irobot_create_2_1
from std_msgs.msg import String
from irobot_create_2_1.srv import *
from irobot_create_2_1.msg import *
from math import *

D = {}
song = rospy.ServiceProxy('song', Song)
tank = rospy.ServiceProxy('tank', Tank) # tank permits requests, e.g., tank(0,50,50)

def init_globals():
    """ Sets everything up """
    # Create cv stuff
    cv.NamedWindow('skeleton')
    cv.SetMouseCallback('skeleton', onMouse, None)
    D["image"] = cv.CreateImage((640,480),8,1)
    D["hand"] = {'right':{'y':0, 'x':0, 'size':10}, 'left': {'y':0, 'x':0, 'size': 10}}
    D["elbow"] = {'right':{'y':0, 'x':0}, 'left': {'y':0, 'x':0}}
    D["shoulder"] = {'right':{'y':0, 'x':0}, 'left': {'y':0, 'x':0}}
    D["head"] = (0,0)
    D["state"] = "Keyboard"
    D["ltrans"] = {'0':0, '1':0}
    D["rtrans"] = {'0':0, '1':0}
    D["vel"] = "nothing"

def drawCircles():
    cv.Circle(D["image"], D["head"], 10, cv.RGB(100, 100, 100), 
              thickness=1, lineType=8, shift=0)
    cv.Circle(D["image"], (D["hand"]["left"]["x"],D["hand"]["left"]["y"]), 
              D["hand"]["left"]["size"], cv.RGB(125, 125, 125), 
              thickness=1, lineType=8, shift=0)
    cv.Circle(D["image"], (D["hand"]["right"]["x"],D["hand"]["right"]["y"]), 
              D["hand"]["right"]["size"], cv.RGB(125, 125, 125), 
              thickness=1, lineType=8, shift=0)
    cv.Circle(D["image"], (D["shoulder"]["left"]["x"],D["shoulder"]["left"]["y"]),
              10, cv.RGB(255, 255, 255), thickness=1, lineType=8, shift=0)
    cv.Circle(D["image"], (D["shoulder"]["right"]["x"],D["shoulder"]["right"]["y"]), 10, 
              cv.RGB(255, 255, 255), thickness=1, lineType=8, shift=0)
    cv.Line(D["image"], (D["shoulder"]["right"]["x"],D["shoulder"]["right"]["y"]), 
            (D["hand"]["right"]["x"],D["hand"]["right"]["y"]), 
            cv.RGB(255, 255, 255), thickness = 1, lineType = 0, shift = 0)
    cv.Line(D["image"], (D["shoulder"]["left"]["x"],D["shoulder"]["left"]["y"]),
            (D["hand"]["left"]["x"],D["hand"]["left"]["y"]), 
            cv.RGB(255, 255, 255), thickness = 1, lineType = 0, shift = 0)
    
  
def printImage():
    # handle key presses
    key_press = cv.WaitKey(5) & 255
    if key_press != 255: check_key_press( key_press )
    drawCircles()
    # display the image
    cv.ShowImage('skeleton', D["image"])


def onMouse(event, x, y, flags, param):
    """ the method called when the mouse is clicked """
    # if the left button was clicked
    if event==cv.CV_EVENT_LBUTTONDOWN:
        cv.SetZero(D["image"])
        print " being reset? "
        


def check_key_press(key_press):
    """ this method handles user key presses appropriately """
    # if a 'q' or ESC was pressed
    if key_press == 27 or key_press == ord('q'):
        print "quitting"
        rospy.signal_shutdown( "Quit requested from keyboard" )

    elif key_press == 32:
        D['state'] = "Keyboard"
        tank(0,0,0)

    elif key_press == 81:
        print "turning left"
        tank(0, -200, 200)

    elif key_press == 83:
        print "you pressed right"
        tank(0, 200, -200)

    elif key_press == 82:
        print "going forwards"
        tank(0, 200, 200)

    elif key_press == 84:
        print "going backwards"
        tank(0, -200, -200)

    elif key_press == ord('F'):
        print "FSM!"
        D["state"] = "WaitforInput"
        FSM1(timer_event = None)

def FSM1(timer_event = None):
    print D["state"]
    if D["state"] == "Keyboard":
        tank(0,0,0)
        return
    elif D["state"] == "WaitforInput":
        handmove()
        rospy.Timer(rospy.Duration(0.5),FSM1,oneshot=True)
        return
        # needs work
        
    elif D["state"] == "Forward":
        print "I made it"
        tank(0,50,50)
        rospy.Timer(rospy.Duration(0.5),FSM1,oneshot=True)
        handmove()
        return
    elif D["state"] == "Backward":
        tank(0,-50,-50)
        rospy.Timer(rospy.Duration(0.5),FSM1,oneshot=True)
        handmove()
        return
    elif D["state"] == "Left":
        tank(0,50,-50)
        rospy.Timer(rospy.Duration(0.5),FSM1,oneshot=True)
        handmove()
        return
    
    elif D["state"] == "Right":
        tank(0,-50,50)
        rospy.Timer(rospy.Duration(0.5),FSM1,oneshot=True)
        handmove()
        return

def handmove():
    print "HandMove"
    if  D["ltrans"]['1']<-0.35 and D["rtrans"]['1']<-0.35:
        print "FORWARD"
        D["state"] = "Forward"
        return

    elif D["ltrans"]['1']>0.35 and D["rtrans"]['1']>0.35:
        print "STOPPING"
        D["state"] = "Keyboard"
        return
    else:
        D["state"] = "WaitforInput"
        return


# trans is the delta in X, Y, and Z from origin to target
# rot is the pitch, roll, yaw, and MYSTERY to get origin to align with target
# elbow is weird. :D

def begin():
    print "Hello, MyCS is working!"
    rospy.init_node('bodytrak', anonymous=True)
    listener = tf.TransformListener()
    h = '/head_1'
    rs = '/right_shoulder_1'
    rh = '/right_hand_1'
    re = '/right_elbow_1'
    ls = '/left_shoulder_1'
    lh = '/left_hand_1'
    le = '/left_elbow_1'

    while not rospy.is_shutdown():
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
        
        D["head"] = (320, 240)
        D["shoulder"]["right"]["x"] = int(300-rstrans[0]*50)
        D["shoulder"]["left"]["x"] = int(340-lstrans[0]*50)
        D["shoulder"]["right"]["y"] = int(240-rstrans[1]*100)
        D["shoulder"]["left"]["y"] = int(240-lstrans[1]*100)
##        D["elbow"]["right"]["x"]= int(D["shoulder"]["right"]["x"]-retrans[0]*200)
##        D["elbow"]["left"]["x"]= int(D["shoulder"]["left"]["x"]-letrans[0]*200)
##        D["elbow"]["right"]["y"]= int(D["shoulder"]["right"]["y"]-retrans[1]*240)
##        D["elbow"]["left"]["y"]= int(D["shoulder"]["left"]["y"]-letrans[1]*240)
        D["hand"]["right"]["x"]= int(D["shoulder"]["right"]["x"]-rtrans[0]*200)
        D["hand"]["left"]["x"]= int(D["shoulder"]["left"]["x"]-ltrans[0]*200)
        D["hand"]["right"]["y"]= int(D["shoulder"]["right"]["y"]+rtrans[1]*240)
        D["hand"]["left"]["y"]= int(D["shoulder"]["left"]["y"]+ltrans[1]*240)
        D["hand"]["left"]["size"] = int(10+ltrans[2]*10)
        D["hand"]["right"]["size"] = int(10+rtrans[2]*10)
        hx = int(D["shoulder"]["right"]["x"] + D["shoulder"]["left"]["x"])/2
        hy = int(D["shoulder"]["right"]["y"] + D["shoulder"]["left"]["y"])/2
        D["head"] = (hx,hy-80)
            
        key_press = cv.WaitKey(5) & 255
        if key_press != 255:
            check_key_press( key_press )
        drawCircles()
        # display the image
        cv.ShowImage('skeleton', D["image"])
        # done!

        if ltrans[1]<-0.35 and rtrans[1]<-0.35 and \
           D["vel"] != "forward":
            D["vel"] = "forward"
            print "FORWARD"
            tank(0,200,200)

        if ltrans[1]> 0.45 and rtrans[1]>0.45 and \
           D["vel"] != "stop":
            D["vel"] = "stop"
            print "stop"
            tank(0,0,0)

        if ltrans[1]< -0.35 and rtrans[1]>0.35 and \
           D["vel"] != "right":
            D["vel"] = "right"
            print "right"
            tank(0,200,-200)

        if ltrans[1]> 0.35 and rtrans[1]<-0.35 and \
           D["vel"] != "left":
            D["vel"] = "left"
            print "left"
            tank(0,-200,200)

        if rtrans[2] < -0.45 and ltrans[2] < -0.45 and \
           D["vel"] != "backwards":
            D["vel"] = "backwards"
            print "backwards"
            tank(0,-200,-200)

    
        cv.SetZero(D["image"])
        #D["ltrans"]['0'] = ltrans[0]
        #D["ltrans"]['1'] = ltrans[1]
        #D["rtrans"]['0'] = rtrans[0]
        #D["rtrans"]['1'] = rtrans[1]
        #print "da y's", ltrans[1], rtrans[1]
        #print "da x's",ltrans[0], rtrans[0]

        #ros_services()


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

        if rtrans[2] > 0.45 and ltrans[2] > 0.45:
            print "Stop and Quit"
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
        # print rt, lt

        #basics.tank(lt, rt)


if __name__ == '__main__':
    init_globals()
    begin();
