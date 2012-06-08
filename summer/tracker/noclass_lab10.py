#!/usr/bin/env python
import roslib; roslib.load_manifest('Daneel')
import rospy

import sensor_msgs.msg as sm
import cv_bridge
import cv
import time
import threading

import pydrone.srv
import pydrone.ARDrone as ARDrone
import pydrone.msg as msg

import os
import random

D = {}

#
# OpenCV Python documentation:
#    http://opencv.willowgarage.com/documentation/python/
#

def init_globals(use_drone=False):
    """ setting up data """
    global D

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
    D["folder_names"] = {('c',0):"./drone_images/c_0_Ginny",
                         ('c',4):"./drone_images/c_4_Bill",
                         ('n',3):"./drone_images/n_3_Albus",
                         ('s',0):"./drone_images/s_0_Draco",
                         ('s',4):"./drone_images/s_4_Ron",
                         ('c',1):"./drone_images/c_1_Fred",
                         ('c',5):"./drone_images/c_5_Molly",
                         ('n',0):"./drone_images/n_0_Neville",
                         ('n',4):"./drone_images/n_4_Rubeus",
                         ('s',1):"./drone_images/s_1_Sirius",
                         ('s',5):"./drone_images/s_5_Hermione",
                         ('c',2):"./drone_images/c_2_George",
                         ('c',6):"./drone_images/c_6_Arthur",
                         ('n',1):"./drone_images/n_1_Luna",
                         ('n',5):"./drone_images/n_5_Severus",
                         ('s',2):"./drone_images/s_2_Remus",
                         ('s',6):"./drone_images/s_6_Harry",
                         ('c',3):"./drone_images/c_3_Charlie",
                         ('n',2):"./drone_images/n_2_Minerva",
                         ('n',6):"./drone_images/n_6_Dobby",
                         ('s',3):"./drone_images/s_3_Fawkes"}

    # thresholds for image processing
    D["thresholds"] = {'low_red': 0, 'high_red': 256,
                       'low_green': 0, 'high_green': 256, 
                       'low_blue': 0, 'high_blue':256, 
                       'low_hue': 0, 'high_hue': 256,
                       'low_sat': 0, 'high_sat': 256, 
                       'low_val': 0, 'high_val': 256}
        
    # windows for image processing
    cv.NamedWindow('image')
    cv.MoveWindow('image', 100, 0)
    cv.SetMouseCallback('image', onMouse, None)
    cv.NamedWindow('threshold')
    cv.MoveWindow('threshold',100,400)
    cv.SetMouseCallback('threshold', onMouse, None)
    make_slider_window()

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

def create_all_images():
    """ a one-time method that creates all of the images needed
        for the image processing...
            
        This should be called only if self.color_image is NOT None
        but self.threshed_image IS None
    """
    #Find the size of the images
    D["size"] = cv.GetSize(D["color_image"])
    
    temp_list = ["red", "green", "blue", "hue", "sat", "val",
                 "copy", "threshed_image"]

    for i in temp_list:
        D[i] = cv.CreateImage(D["size"], 8, 1)

    D["hsv"] = cv.CreateImage(D["size"], 8, 3)     # HSV image
    D["storage"] = cv.CreateMemStorage(0) # create memory storage for contours


def save_thresholds():
    """ saves the current thresholds to data.txt """
    f = open("data.txt","w")
    print >> f, D["thresholds"]
    f.close()
    print "thresholds saved to data.txt"

def load_thresholds():
    """ loads the thresholds from data.txt into self.thresholds """
    try:
        f = open("data.txt","r")
        data = f.read()
        f.close()
        D["thresholds"] = eval(data)
        print "thresholds loaded from data.txt"
        make_slider_window()
    except:
        print "An error occurred in loading data.txt"
        print "Check if it's there and if it contains"
        print "  a dictionary of thresholds..."

def change_location(direction):
    """ changes the location from which an image is taken in one of six ways """
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


def make_slider_window():
    """ a method to make a window full of sliders """
    global D
    
    # create slider window
    cv.NamedWindow('sliders')
    cv.MoveWindow('sliders', 742, 100)

    D["threshold_keys"] = ["low_red", "high_red", "low_green", "high_green", 
                           "low_blue", "high_blue", "low_hue", "high_hue",
                           "low_sat", "high_sat", "low_val", "high_val"]

    for i in D["threshold_keys"]:
        cv.CreateTrackbar(i, 'sliders', D["thresholds"][i], 256, 
                          lambda x, j=i: change_slider(x, j))

def change_slider(new_val, slider):
    global D
    D["thresholds"][slider] = new_val

def handle_next_image(data):
    """Displays the image, calls find_info"""        
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


# defines commands to control the drone
def control_drone(command_string):
    """ this method provides the various drone controls
        it allows simulation when self.use_drone == False
        and actual control when self.use_drone == True
    """
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


# handle mouse events
def onMouse(event,x,y,flags,param):
    """ mouse-click handler from OpenCV """
    if event == cv.CV_EVENT_LBUTTONDOWN and D["color_image"] != None:
        b, g, r = D["color_image"][y,x]
        if D["hsv"] != None: h, s, v = D["hsv"][y,x]
        print "Pixel has (r,g,b) =", (r,g,b),
        print "and (h,s,v) =", (h,s,v)


# do all of the image processing
def find_landmark():
    """ here is where the image should be processed to get the bounding box """
    # check if we've created the supporting images yet
    if D["threshed_image"] == None:
        if D["color_image"] != None:
            create_all_images()

    # from the old method call def threshold_image(self):
    cv.Split(D["color_image"], D["blue"], D["green"], D["red"], None)
    cv.CvtColor(D["color_image"], D["hsv"], cv.CV_RGB2HSV)
    cv.Split(D["hsv"], D["hue"], D["sat"], D["val"], None)

    # replace each channel with its thresholded version
    cv.InRangeS(D["red"], D["thresholds"]['low_red'],
                D["thresholds"]['high_red'], D["red"])
    cv.InRangeS(D["green"], D["thresholds"]['low_green'],
                D["thresholds"]['high_green'], D["green"])
    cv.InRangeS(D["blue"], D["thresholds"]['low_blue'],
                D["thresholds"]['high_blue'], D["blue"])
    cv.InRangeS(D["hue"], D["thresholds"]['low_hue'],
                D["thresholds"]['high_hue'], D["hue"])
    cv.InRangeS(D["sat"], D["thresholds"]['low_sat'],
                D["thresholds"]['high_sat'], D["sat"])
    cv.InRangeS(D["val"], D["thresholds"]['low_val'],
                D["thresholds"]['high_val'], D["val"])

    # AND (multiply) all the thresholded images into one "output" image,
    # named self.threshed_image
    cv.Mul(D["red"], D["green"], D["threshed_image"])
    cv.Mul(D["threshed_image"], D["blue"], D["threshed_image"])
    cv.Mul(D["threshed_image"], D["hue"], D["threshed_image"])
    cv.Mul(D["threshed_image"], D["sat"], D["threshed_image"])
    cv.Mul(D["threshed_image"], D["val"], D["threshed_image"])
    # erode and dilate shave off and add edge pixels respectively
    cv.Erode(D["threshed_image"], D["threshed_image"], iterations = 1)
    cv.Dilate(D["threshed_image"], D["threshed_image"], iterations = 1)
        
    find_biggest_region()


def find_biggest_region():
    """ this code should find the biggest region and
        then determine some of its characteristics, which
        will help direct the drone
    """
    # copy the thresholded image
    cv.Copy( D["threshed_image"], D["copy"] )  # copy self.threshed_image
    # this is OpenCV's call to find all of the contours:
    contours = cv.FindContours(D["copy"], D["storage"], cv.CV_RETR_EXTERNAL,
                               cv.CV_CHAIN_APPROX_SIMPLE)

# put text on the image...
def text_to_image():
    """ write various things on the image ... """
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

# our drone's state machine
def FSM1(timer_event=None):
    """ the finite-state machine """
    print "Now in state", D["state"]
    
    if D["state"] == "Keyboard": # user stopped our state machine!
        print "State machine has been stopped!"
        print "Sending hover command"
        if D["airborne"] == True: # only hover if airborne!
            control_drone( "hover" )
        return # officially ends the state machine

    elif D["state"] == "start":
        print "Starting the state machine!"
        D["state"] = "takeoff"
        rospy.Timer( rospy.Duration(0.1), FSM1, oneshot=True )
            
    elif D["state"] == "takeoff":
        print "taking off in the FSM"
        control_drone( "takeoff" )
        rospy.sleep(5.0)  # wait for takeoff...
        D["state"] = "hover"
            # reschedule the next state-machine callback
        rospy.Timer( rospy.Duration(0.1), FSM1, oneshot=True )
        return

    elif D["state"] == "wait_to_c":
        rospy.sleep(0.2)
        if D["saw_capital_c"]:
            D["state"] = "hover"
        rospy.Timer( rospy.Duration(0.1), FSM1, oneshot=True )
        return

    elif D["state"] == "hover":
        control_drone( "hover" )
        rospy.sleep(0.1)
        D["state"] = "wait_to_c"
        D["saw_capital_c"] = False
        rospy.Timer( rospy.Duration(0.1), FSM1, oneshot=True )
        return

    else:  # state not recognized
        print "the state", D["state"], "was not recognized"
        print "Changing to Keyboard state"
        D["state"] = "Keyboard"
        rospy.Timer( rospy.Duration(0.1), FSM1, oneshot=True )
        return

# the keyboard thread is the "main" thread for this program
def keyboardThread():
    """ the main keypress-handling thread to control the drone """
    D["airborne"] = False
    
    # this is the main loop for the keyboard thread
    #
    # don't use 'Q', 'R', 'S', or 'T'  (they're the arrow keys)
    #
    while True:
        
        if D["use_drone"] == False: # if no drone, we get our own images
            handle_next_image(None)
            
        # handle the image processing if we have a new Kinect image
        if D["new_image"] == True:
            D["new_image"] = False # until we get a new one...
            # now, do the image processing
            find_landmark()
            # now, put text information on the image
            text_to_image()
            # show the image
            cv.ShowImage('image', D["color_image"])
            cv.ShowImage('threshold', D["threshed_image"])
            
            # get the next keypress
        c = cv.WaitKey(25)&255

            # handle the keypress to quit...
        if c == ord('q') or c == 27: # the Esc key is 27
            D["state"] = "Keyboard" # stop the FSM
            control_drone( "halt" ) # lands and halts
            rospy.sleep(1.42) # wait a bit for everything to finish...
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


        # saving and loading the thresholds from data.txt
        if c == ord('$'):  # dollar sign to save (looks like an S)
            save_thresholds()

        if c == ord('&'):  # ampersand loads thresholds from data.txt
            load_thresholds()

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


if __name__ == '__main__':
    '''
    Main driver function for the drone.
    '''
    # Setup stuff
    rospy.init_node('dronekinect')  # names this ROS node

    use_drone = False
    init_globals(use_drone)
#    controller = DroneController(use_drone)

    if use_drone == True:
        rospy.Subscriber('arimage',sm.Image,controller.handle_next_image)
        rospy.Subscriber('arnavdata', msg.NavData, controller.getData)

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
    keyboardThread()
    
    print "the Python program is quitting..."
