#!/usr/bin/env python
import roslib; roslib.load_manifest('Frizzle')
import rospy
import sensor_msgs.msg as sm
import cv_bridge
import cv
import numpy
import math, cmath

D = {}

########################## INITIALIZATION FUNCTIONS ##########################

def init_globals():
    """ Sets everything up """
    global D
    # Create image stuff
    cv.NamedWindow('dist')
    cv.MoveWindow('dist', 0, 0)
    D["bridge"] = cv_bridge.CvBridge()
    cv.SetMouseCallback('dist', onMouse, None)

    # for display purposes
    D["mode"] = "kinect"
    D["init"] = True
    D["changed"] = False
    D["font"] = cv.InitFont(cv.CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5, 0, 2)
    D["white"] = cv.RGB(255,255,255)
    D["black"] = cv.RGB(0,0,0)


def init_loader():
    """ Sets up a loaded image ready for display """
    global D

    D["size"] = cv.GetSize(D["loaded_image"])
    D["image"] = cv.CreateImage(D["size"], 8, 1)
    D["init"] = True
    
    redraw_loaded()

######################## END INITIALIZATION FUNCTIONS ########################

######################### IMAGE PROCESSING FUNCTIONS #########################

def redraw_loaded():
    """ redraws a loaded image if necessary """
    global D
    D["changed"] = False
    cv.CvtColor(D["loaded_image"], D["image"], cv.CV_BGR2GRAY)
    calculate_angles()
    draw_on_image()


def draw_on_image():
    """ draws circles on image at points p1 and p2 """
    global D
    
    # Draw circles on the points you choose


def calculate_angles():
    """ Looks at two points in the depth image and gets the angles in between them """
    global D

    # Calculate the angles between the points you choose
    
####################### END IMAGE PROCESSING FUNCTIONS #######################

############################# CALLBACK FUNCTIONS #############################

def handle_next_image(data=None):
    """ Called when the kinect sends an image """
    global D

    if D["mode"] == "kinect": # get the image from the Kinect
        D["image"] = D["bridge"].imgmsg_to_cv(data, "32FC1")
        calculate_angles()
        draw_on_image()

    # handle key presses
    key_press = cv.WaitKey(5) & 255
    if key_press != 255: check_key_press( key_press )

    # display the image
    if D["mode"] == "kinect":
        cv.ShowImage('dist', D["image"])
    

def handle_update( timer_event=None ):
    """ deals with updates when looking at loaded images"""
    global D

    # init loader will take care of initializing if necessary
    if not D["init"]:
        init_loader()

    # redraw will redraw things on the image if it has changed
    if D["changed"]:
        redraw_loaded()
    
    # handle key presses
    key_press = cv.WaitKey(5) & 255
    if key_press != 255: check_key_press( key_press )

    # display the image
    cv.ShowImage('dist', D["image"])

    # make sure we will come back to this function if we need to
    if not D["mode"] == "kinect":
        rospy.Timer( rospy.Duration(0.1), handle_update, oneshot=True )


def onMouse(event, x, y, flags, param):
    """ the method called when the mouse is clicked """
    global D

    # if the left button was clicked
    if event==cv.CV_EVENT_LBUTTONDOWN:
        print "x, y are", x, y
        pixel_val= D["image"][y,x]
        print "the pixel's depth value is", pixel_value


def check_key_press(key_press):
    """ this method handles user key presses appropriately """
    global D

    loader = { ord('x'):"angle-30.png", ord('c'):"angle-15.png", 
               ord('v'):"angle0.png", ord('b'):"angle15.png",
               ord('n'):"angle30.png"}

    # if a 'q' or ESC was pressed
    if key_press == 27 or key_press == ord('q'):
        print "quitting"
        rospy.signal_shutdown( "Quit requested from keyboard" )

    elif key_press == ord('h'):
        print " Help menu!"
        print " ==================================="
        print " ESC   : quits the program"
        print " q     : quits the program"
        print " h     : displays help menu"
        print " r     : get range images straight from the kinect"
        print " x     : loads -30 degree angle image into main window"
        print " c     : loads -15 degree angle image into main window"
        print " v     : loads 0 degree angle image into main window"
        print " b     : loads 15 degree angle image into main window"
        print " n     : loads 30 degree angle image into main window"

    elif key_press == ord('r'):
        D["mode"] = "kinect"

    elif key_press in loader.keys():
        D["prev_mode"] = D["mode"]
        D["mode"] = "loader"
        try:
            D["loaded_image"] = cv.LoadImage(loader[key_press], 
                                             iscolor=cv.CV_LOAD_IMAGE_UNCHANGED)
            D["init"] = False
            print "loaded:", loader[key_press]
        except:
            D["mode"] = "blank"
            print "Image not loaded from file:", loader[key_press]
        
        if D["prev_mode"] == "kinect":
            rospy.Timer( rospy.Duration(0.1), handle_update, oneshot=True )

########################### END CALLBACK FUNCTIONS ###########################

if __name__ == "__main__":
    """ Main function, performs setup and starts program running """

    # Initialize our node
    rospy.init_node('distanceReader')

    # Initialize globals
    init_globals()

    # Subscribe to the image topic
    rospy.Subscriber('/camera/depth/image',sm.Image, handle_next_image)

    # Run until something stops us
    rospy.spin()
