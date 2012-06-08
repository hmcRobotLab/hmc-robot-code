#!/usr/bin/env python
import roslib; roslib.load_manifest('Frizzle')
import rospy
import cv_bridge
import cv
import sensor_msgs.msg as sm
from irobot_create_2_1.srv import *
from irobot_create_2_1.msg import *
import irobot_create_2_1
import math

# Dictionary to hold all globals in
D = {}


#################### INITIALIZATION FUNCTIONS ######################

def init_globals():
    """ sets up all the globals in the dictionary D
    """
    # get D so that we can change values in it
    global D

    # put threshold values into D
    D['thresholds'] = {'low_red':0, 'high_red':255,\
                       'low_green':0, 'high_green':255,\
                       'low_blue':0, 'high_blue':255,\
                       'low_hue':0, 'high_hue':255,\
                       'low_sat':0, 'high_sat':255,\
                       'low_val':0, 'high_val':255 }
    D['thresholds2'] = {'low_red':0, 'high_red':255,\
                       'low_green':0, 'high_green':255,\
                       'low_blue':0, 'high_blue':255,\
                       'low_hue':0, 'high_hue':255,\
                       'low_sat':0, 'high_sat':255,\
                       'low_val':0, 'high_val':255 }
    try:
        f = open( "./thresh.txt", "r" ) # open the file "thresh.txt" for reading
        data = f.read() # read everything from f into data
        x = eval( data ) # eval is Python's evaluation function
        # eval evaluates strings as if they were at the Python shell
        f.close() # its good to close the file afterwards

        # Set threshold values in D
        D['thresholds'] = x
    except:
        print "Could not open file 'thresh.txt' "
    
    try:
        f = open( "./thresh2.txt", "r" ) # open the file "thresh.txt" for reading
        data = f.read() # read everything from f into data
        x = eval( data ) # eval is Python's evaluation function
        # eval evaluates strings as if they were at the Python shell
        f.close() # its good to close the file afterwards#

        # Set threshold values in D
        D['thresholds2'] = x
    except:
        print "Could not open file 'thresh2.txt' "
    
        
    # Set up the windows containing the image from the kinect,
    # the altered image, and the threshold sliders.
    cv.NamedWindow('image')
    cv.MoveWindow('image', 0, 0)
    cv.NamedWindow('threshold')
    cv.MoveWindow('threshold', 640, 0)

    cv.NamedWindow('threshold2')
    cv.MoveWindow('threshold2', 640, 640)

    cv.NamedWindow('sliders')
    cv.MoveWindow('sliders', 1280, 0)

    cv.NamedWindow('sliders2')
    cv.MoveWindow('sliders2', 100, 100)

    # Create the sliders within the 'sliders' window
    D["key_list"] = ['low_red', 'high_red', 'low_green', 'high_green', 
                     'low_blue', 'high_blue', 'low_hue', 'high_hue', 
                     'low_sat', 'high_sat', 'low_val', 'high_val']

    for i in D["key_list"]:
        cv.CreateTrackbar(i, 'sliders', D['thresholds'][i], 255, \
                              lambda x, j=i: change_slider(j, x) )
        cv.CreateTrackbar(i, 'sliders2', D['thresholds2'][i], 255, \
                              lambda x, j=i: change_slider2(j, x) )

    # Set the method to handle mouse button presses
    cv.SetMouseCallback('image', onMouse, None)

    # We have not created our "scratchwork" images yet
    D["created_images"] = False

    # Variable for key presses
    D["last_key_pressed"] = 255
    # The current image we want to display in the threshold window
    D["current_threshold"] = "threshed_image"
    
    # Not currently clicking down
    D["mouse_down"] = False

    # Highlighted area does not exist yet
    D["down_coord"] = (-1,-1)
    D["up_coord"] = (-1,-1)
    D["sections"] = []
    D["mode"] = "clear"

    # State machine variables
    D["state"] = "keyboard"
    D["curr_loc"] = (0,0)
    D["curr_loc2"] = (0,0)
    D["for_loc"] = (0,0)
    D["back_loc"] = (0,0)
    D["first_back"] = True

    # Location to be going
#    D["right_coord"] = (0,0)
#    D["r_heading"] = 0
    D["n_heading"] = 0
    D["font"] = cv.InitFont(cv.CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5, 0, 2)

    # Create a connection to the Kinect
#    D["bridge"] = cv_bridge.CvBridge()
