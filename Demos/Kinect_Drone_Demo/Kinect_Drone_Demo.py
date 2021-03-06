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
import time, threading
import pydrone.srv
import pydrone.ARDrone as ARDrone
import pydrone.msg as msg
import os, random

D = {}
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

def init_globals(use_drone=False):
    """ Sets everything up """
    global D
    # Create cv stuff
    print "started making a window"
    cv.NamedWindow('skeleton')
    cv.MoveWindow('skeleton', 0, 0)
    cv.SetMouseCallback('skeleton', onMouse, None)
    print 'finished making a window'

    D["sk_image"] = cv.CreateImage((640,480),8,1)
    cv.ShowImage('skeleton', D["sk_image"])

    D["hand"] = {'right':{'y':0, 'x':0, 'size':10}, 'left': {'y':0, 'x':0, 'size': 10}}
    D["elbow"] = {'right':{'y':0, 'x':0}, 'left': {'y':0, 'x':0}}
    D["shoulder"] = {'right':{'y':0, 'x':0}, 'left': {'y':0, 'x':0}}
    D["head"] = (0,0)
    D["state"] = "Keyboard"
    D["ltrans"] = {'0':0, '1':0}
    D["rtrans"] = {'0':0, '1':0}
    D["vel"] = "nothing"

    # member variables relating to the drone's control
    D["use_drone"] = use_drone    
    init_drone_parameters()
    D["airborne"] = False
    D["run_without_data_stream"] = False
    D["battery_level"] = "Not read from drone"

    # variables for the off-board images
    D["location"] = ('c',1) # the starting location (always a tuple)
    #self.location = ( random.choice(['c','n','s']), random.randint(0,6) )
    D["image_hour"] = 3 # heading at 3 o'clock == toward the markers
    #self.angle_deg = random.randint(180,360) # the angle it's facing
    D["last_image_time"] = time.time() # last time an image was grabbed
        
    # images and other data for the images/windows
    D["bridge"] = cv_bridge.CvBridge()  # the interface to OpenCV
    D["color_image"] = None             # the image from the drone
    D["new_image"] = False              # did we just receive a new image?
    D["threshed_image"] = None          # thresholded image
    D["font"] = cv.InitFont(cv.CV_FONT_HERSHEY_PLAIN, 1, 1, 0, 1)
    D["contours_found"] = False
    D["saw_capital_c"] = False
    
    # variables for state machine
    D["state"] = "Keyboard"

    D["reset_needed"] = False

def init_drone_parameters():
    global D

    print "Constructing the drone software object..."
    if D["use_drone"] == True:
        D["drone"] = ARDrone.ARDrone()
        # Drone's motion speed
        D["drone"].speed = 0.15
        D["drone"].frequency = 8 # Drone's ultrasound frequency - 7 for 22.22 Hz, 8 for 25 Hz
        D["drone"].altmax_value = 10000 # Drone's maximum altitude between 500
                                        # and 5000 or 10000 for unlimited

        # print "Trying to connect to the drone"
        D["drone"].connect()
        D["drone"].freq(D["drone"].frequency)
        D["drone"].altmax(D["drone"].altmax_value)

        # set the video stream
        D["drone"].cam = 0 # 0 for forward, 1 for floor camera
        D["drone"].feed(D["drone"].cam)
        rospy.sleep(1)
        print "Drone initialized."
    else:
        print "Not using drone."

######################### END INITIALIZATION FUNCTIONS ###########################

############################ IMAGE DRAWING FUNCTIONS #############################

def drawCircles():
    """ Draw circles on the skeleton image """
    global D
    cv.Circle(D["sk_image"], D["head"], 10, cv.RGB(100, 100, 100), 
              thickness=1, lineType=8, shift=0)
    cv.Circle(D["sk_image"], (D["hand"]["left"]["x"],D["hand"]["left"]["y"]), 
              D["hand"]["left"]["size"], cv.RGB(125, 125, 125), 
              thickness=1, lineType=8, shift=0)
    cv.Circle(D["sk_image"], (D["hand"]["right"]["x"],D["hand"]["right"]["y"]), 
              D["hand"]["right"]["size"], cv.RGB(125, 125, 125), 
              thickness=1, lineType=8, shift=0)
    cv.Circle(D["sk_image"], (D["shoulder"]["left"]["x"],D["shoulder"]["left"]["y"]),
              10, cv.RGB(255, 255, 255), thickness=1, lineType=8, shift=0)
    cv.Circle(D["sk_image"], (D["shoulder"]["right"]["x"],D["shoulder"]["right"]["y"]), 10, 
              cv.RGB(255, 255, 255), thickness=1, lineType=8, shift=0)
    cv.Line(D["sk_image"], (D["shoulder"]["right"]["x"],D["shoulder"]["right"]["y"]), 
            (D["hand"]["right"]["x"],D["hand"]["right"]["y"]), 
            cv.RGB(255, 255, 255), thickness = 1, lineType = 0, shift = 0)
    cv.Line(D["sk_image"], (D["shoulder"]["left"]["x"],D["shoulder"]["left"]["y"]),
            (D["hand"]["left"]["x"],D["hand"]["left"]["y"]), 
            cv.RGB(255, 255, 255), thickness = 1, lineType = 0, shift = 0)


def printImage():
    """ Draw on and update the skeleton image """
    global D
    drawCircles()
    # display the image
    cv.ShowImage('skeleton', D["sk_image"])

def change_location(direction):
    """ changes the location from which an image is taken in one of six ways """
    global D
    # get the state
    current_ns_character = D["location"][0]
    current_ew_number = D["location"][1]
    current_image_hour = D["image_hour"]

    # make appropriate changes
    if direction == 'west':
        current_ew_number += 1
        if current_ew_number > 6: current_ew_number = 6
    elif direction == 'east':
        current_ew_number -= 1
        if current_ew_number < 0: current_ew_number = 0
    elif direction == 'north':
        if current_ns_character == 'n': pass
        elif current_ns_character == 's': current_ns_character = 'c'
        elif current_ns_character == 'c': current_ns_character = 'n'
    elif direction == 'south':
        if current_ns_character == 'n': current_ns_character = 'c'
        elif current_ns_character == 's': pass
        elif current_ns_character == 'c': current_ns_character = 's'
    elif direction == 'counterclockwise':
        current_image_hour -= 1
        if current_image_hour < 0: current_image_hour = 23
    elif direction == 'clockwise':
        current_image_hour += 1
        if current_image_hour > 23: current_image_hour = 0
    else:
        print "a direction of", direction,
        print "was not recognized in change_location"
            
    # reset the state
    D["location"] = (current_ns_character,current_ew_number)
    D["image_hour"] = current_image_hour
    print "location and image_hour are now", D["location"], D["image_hour"]

def handle_next_image(data):
    """Displays the image, calls find_info"""        
    global D
    # get the image from the Kinect, if use_drone == True
    # kinect images: image = bridge.imgmsg_to_cv(data, "32FC1")
    # drone images:
    if D["use_drone"] == True:
        D["color_image"] = D["bridge"].imgmsg_to_cv(data, "bgr8")
        # tell the keyboard thread that we have a new image
        D["new_image"] = True
        # otherwise, get an image from file every so often
    else:
        cur_time = time.time()
        if cur_time - D["last_image_time"] > 0.25: # number of seconds per update
            D["last_image_time"] = cur_time # reset the last image time
            folder = D["folder_names"][D["location"]]
            fn = folder + "/" + str(D["image_hour"]) + ".png"
            #print "filename", fn
            D["color_image"]=cv.LoadImageM(fn)
            D["new_image"] = True
        # otherwise, we don't get a new image
            
        # in case we want randomness:
        #image_hour_number = random.randint(0,23)
        #self.image_hour = random.randint(3,3)

# handles the drone's data stream (separate from the video and control)
def getData(data):
    """ the drone's navigation and status data """
    global D
    if D["use_drone"] == True and D["run_without_data_stream"] == False:
        try:
            data_dictionary = eval(data.data)
            D["battery_level"] = data_dictionary['battery']
            # also available:
            # 'phi', 'psi', 'num_frames', 'altitude', 'ctrl_state',
            # 'vx', 'vy', 'vz', 'theta'
        except:
            print "\n\n  You're not connected to the drone's data stream.\n\n"
            D["run_without_data_stream"] = True
    else:
        pass # do nothing if no drone

# put text on the image...
def text_to_image():
    """ write various things on the image ... """
    global D
    # the image is 320 pixels wide and 240 pixels high
    # clear a rectangle
    cv.Rectangle(D["color_image"], (3,3), (242,30),
                 cv.RGB(255,255,255), cv.CV_FILLED )
                    
    # set up some text           
    llx = 7
    lly = 22
    s = "State: " + str(D["state"])
    textllpoint = (llx,lly)
    cv.PutText(D["color_image"], s, textllpoint, D["font"], cv.RGB(0,0,255))

########################## END IMAGE DRAWING FUNCTIONS ###########################

############################### CONTROL FUNCTIONS ################################
# defines commands to control the drone
def control_drone(command_string):
    """ this method provides the various drone controls
        it allows simulation when self.use_drone == False
        and actual control when self.use_drone == True
    """
    global D

    if D["use_drone"] == True: # real commands!
        if command_string == "halt":
            D["drone"].land()
            D["airborne"] = False
            D["drone"].halt()
        elif command_string == "land":
            D["drone"].land()
            rospy.sleep(0.1)
            D["airborne"] = False
        elif command_string == "takeoff":
            D["drone"].takeoff()
            rospy.sleep(0.1)
            D["airborne"] = True
        elif command_string == "hover":
            D["drone"].hover()
        elif command_string == "left":
            D["drone"].moveRight(-D["drone"].speed)
        elif command_string == "right":
            D["drone"].moveRight(D["drone"].speed)
        elif command_string == "up":
            D["drone"].moveUp(D["drone"].speed)
        elif command_string == "down":
            D["drone"].moveUp(-D["drone"].speed)
        elif command_string == "forward":
            D["drone"].moveForward(D["drone"].speed)
        elif command_string == "backward":
            D["drone"].moveForward(-D["drone"].speed)
        elif command_string == "turn_right":
            D["drone"].moveRotate(2*D["drone"].speed)
        elif command_string == "turn_left":
            D["drone"].moveRotate(-2*D["drone"].speed)
        # no else!
                
        print "control_drone command (real): ", command_string

    else:  # simulated commands
        print "control_drone command (sim): ", command_string


############################# END CONTROL FUNCTIONS ##############################

################################ INPUT FUNCTIONS #################################

def onMouse(event, x, y, flags, param):
    """ the method called when the mouse is clicked """
    global D
    # if the left button was clicked
    if event==cv.CV_EVENT_LBUTTONDOWN:
        cv.SetZero(D["image"])
        print " being reset? "

def check_key_press(key_press):
    """ this method handles user key presses appropriately """
    global D
    # if a 'q' or ESC was pressed
    if key_press == 27 or key_press == ord('q'):
        print "quitting"
        rospy.signal_shutdown( "Quit requested from keyboard" )

    elif key_press == 32:
        D['state'] = "Keyboard"
        #tank(0,0,0)


############################## END INPUT FUNCTIONS ###############################

################################### MAIN LOOP ####################################

# trans is the delta in X, Y, and Z from origin to target
# rot is the pitch, roll, yaw, and MYSTERY to get origin to align with target
# elbow is weird. :D

def begin():
    global D

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
            key_press_func( key_press )
        drawCircles()

        # display the image
        cv.ShowImage('skeleton', D["sk_image"])
        # done!
        cv.SetZero(D["sk_image"])

#        print "ltrans[0], rtrans[0]", ltrans[0], rtrans[0]
#        print "ltrans[1], rtrans[1]", ltrans[1], rtrans[1]
#        print "ltrans[2], rtrans[2]", ltrans[2], rtrans[2]

        ##################################### INTERPRET GESTURES ###############################
        # if arms behind you
        if rtrans[2] < -0.30 and ltrans[2] < -0.30 and \
                D["vel"] != "backwards":
            D["vel"] = "backwards"
            print "backwards"
            if D["airborne"]:
                control_drone("backward")
    

        elif rtrans[2] > 0.30 and ltrans[2] > 0.30 and D["vel"] != "going_forward": # if arms dirctly in fornt of you
            D["vel"] = "going_forward"
            print "forward"
            if D["airborne"]:
                control_drone("forward")

        # if arms are up
        elif ltrans[1]<-0.35 and rtrans[1]<-0.35 and \
                D["vel"] != "forward":
            D["vel"] = "forward"
            print "Up"
            if D["airborne"]:
                control_drone("up")

        # of arms are down
        elif ltrans[1]> 0.45 and rtrans[1]>0.45 and \
                D["vel"] != "stop":
            D["vel"] = "stop"
            print "Down"
            if D["airborne"]:
                control_drone("down")

        # if right arm up
        elif ltrans[1]< -0.35 and rtrans[1]>0.35 and \
                D["vel"] != "right":
            D["vel"] = "right"
            print "right"
            if D["airborne"]:
                control_drone("right")

        # if left arm up
        elif ltrans[1]> 0.35 and rtrans[1]<-0.35 and \
                D["vel"] != "left":
            D["vel"] = "left"
            print "left"
            if D["airborne"]:
                control_drone("left")

        # Arms straight out to side
        elif abs(-rtrans[1]) < 0.15 and abs(-ltrans[1]) < 0.15 and \
                abs(-rtrans[0]) > 0.45 and abs(-ltrans[0]) > 0.45 and \
                not D["airborne"] and not D["reset_needed"]:
           print "takeoff"
           control_drone("takeoff")

        # right arm straight out, left arm down
        elif abs(-rtrans[1]) < 0.15 and ltrans[1]> 0.45 and D["airborne"] and D["vel"] != "turning_right":
            D["vel"] = "turning_right"
            print "rotate right"
            control_drone("turn_right")

        # left arm straight out, right arm down
        elif abs(-ltrans[1]) < 0.15 and rtrans[1]> 0.45 and D["airborne"] and D["vel"] != "turning_left":
            D["vel"] = "turning_left"
            print "rotate left"
            control_drone("turn_left")

        # hands near belly
        elif abs(ltrans[0]) < 0.2 and abs(rtrans[0]) < 0.2 and \
                abs(ltrans[1]) < 0.2 and abs(rtrans[1]) < 0.2 and \
                abs(ltrans[2]) < 0.2 and abs(rtrans[2]) < 0.2 and D["airborne"]:
            print "landing"
            control_drone("land")
            D["reset_needed"] = True

        # Arms straight out to side and already taken off
        elif abs(-rtrans[1]) < 0.15 and abs(-ltrans[1]) < 0.15 and \
                abs(-rtrans[2]) < 0.15 and abs(-ltrans[2]) < 0.15 and \
                D["airborne"] and D["vel"] != "hovering":
            D["vel"] = "hovering"
            print "hover"
            control_drone("hover")

        
def key_press_func(c):
    """ gets key_press c and deals with it """
    global D
    print "in key_press_func"

    if c == ord('r'):
        print "reset drone"
        D["reset_needed"] = False

    # handle the keypress to quit...
    if c == ord('q') or c == 27 and not D["airborne"]: # the Esc key is 27
        #D["state"] = "Keyboard" # stop the FSM
        #control_drone( "halt" ) # lands and halts
        rospy.sleep(1.42) # wait a bit for everything to finish...
        print "Stop and Quit"
        exit(0)
        # it's important that the FSM not re-schedule itself further than
        # 1 second in the future, so that a thread won't be left running
        # after this Python program quits right here:
        return
            

    if c == ord('F'):  # capital-F starts and stops the finite-state machine
        # only run it when it's not already running!
        if D["state"] == "Keyboard":  # it's not yet running, so
            print "Starting state machine..."
            D["state"] = "takeoff"  # these two lines start FSM1()
            FSM1()
            # if it's already running, stop it!
        else:  # if it's in any other state, we bring it back to "Keyboard"
            D["state"] = "Keyboard" # should stop the state machine
            print "Stopping state machine..."


    if c == ord('C'):
        D["saw_capital_c"] = True


        # here are the keyboard commands for the drone...
        #
        # ** NEW THIS WEEK **   you must hit the space bar to stop a motion
        #
        # that is, the space bar hovers; it no longer automatically hovers...
            
    if c == ord('\n'):
        control_drone( "land" )   #self.drone.land()

    elif c == ord('!'): 
        control_drone( "takeoff" )   #self.drone.takeoff()
        
    elif c == ord(' '):  # the space bar will hover
        control_drone( "hover" )   #self.drone.hover()
        
    elif c == ord('e'):   # 'e' switches the video feed
        print 'switch video feed'
        D["drone"].cam = (1 + D["drone"].cam) % 4
        D["drone"].feed(D["drone"].cam)
        
    elif c == ord('b'):   # 'b' for battery level
        print "Battery: ", D["battery_level"]
                

        # this *should* be an if, not an elif - it should always run
    if D["airborne"]:   # the different direction controls
        if c == ord(' '):
            control_drone( "hover" )      #self.drone.hover()
        elif c == ord('a'):
            control_drone( "left" )       #self.drone.moveRight(-self.drone.speed)
        elif c == ord('d'):
            control_drone( "right" )      #self.drone.moveRight(self.drone.speed)
        elif c == ord('w'):
            control_drone( "forward" )    #self.drone.moveForward(self.drone.speed)
        elif c == ord('s'):
            control_drone( "backward" )   #self.drone.moveForward(-self.drone.speed)
        elif c == ord('j'):
            control_drone( "turn_left" )  #self.drone.moveRotate(-2*self.drone.speed)
        elif c == ord('l'):
            control_drone( "turn_right" ) #self.drone.moveRotate(2*self.drone.speed)
        elif c == ord('i'):
            control_drone( "up" )         #self.drone.moveUp(self.drone.speed)
        elif c == ord('k'):
            control_drone( "down" )       #self.drone.moveUp(-self.drone.speed)

        # if self.use_drone == False (simulated mode)
        # the arrow keys allow you to change the images you see...
    if D["use_drone"] == False:
        if c == 82:  # up arrow is the same as ord('R')
            change_location( 'west' )
        if c == 84:  # down arrow is the same as ord('T')
            change_location( 'east' )
        if c == 81:  # left arrow is the same as ord('Q')
            change_location( 'north' )
        if c == 83:  # right arrow is the same as ord('S')
            change_location( 'south' )
        if c == ord('-'):  # '-' decreases the image_hour
            change_location( 'counterclockwise' )
        if c == ord('=') or c == ord('+'):  # '=' or '+' increases the image_hour
            change_location( 'clockwise' )

################################# END MAIN LOOP ##################################

if __name__ == '__main__':
    '''
    Main driver function for the drone.
    '''
    # Setup stuff
#    rospy.init_node('fancydronekinect')  # names this ROS node

    use_drone = True
    init_globals(use_drone)

    #if use_drone == True:
    #    rospy.Subscriber('arimage',sm.Image,controller.handle_next_image)
    #    rospy.Subscriber('arnavdata', msg.NavData, controller.getData)

    # Display initial message
    print 'Keyboard controls:\n'
    print '! (exclamation point) takes off'
    print '\\n (return/enter) lands the drone'
    print 'space bar hovers (stops the drone\'s motion)\n'
    print 'The wasd keys translate at a fixed height and orientation'
    print 'i/k move the drone up/down'
    print 'j/l turn the drone left/right\n'
    print 'q or esc lands the drone and quits the program.'

    rospy.sleep(1)
    
    # start the main loop, the keyboard thread:
    begin();
    
    print "the Python program is quitting..."
