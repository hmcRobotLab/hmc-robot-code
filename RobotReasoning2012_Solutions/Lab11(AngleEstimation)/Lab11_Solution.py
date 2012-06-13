#!/usr/bin/env python
import roslib; roslib.load_manifest('irobot_mudd')
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

    # Create slider stuff
    D["sliders"] = {"left_x":200, "right_x":440, "left_y":240, "right_y":240}
    cv.NamedWindow('sliders')
    cv.MoveWindow('sliders', 640, 0)

    cv.CreateTrackbar("left_x", 'sliders', D["sliders"]["left_x"], 640, 
                      lambda x: change_slider("left_x", x) )
    cv.CreateTrackbar("right_x", 'sliders', D["sliders"]["right_x"], 640, 
                      lambda x: change_slider("right_x", x) )
    cv.CreateTrackbar("left_y", 'sliders', D["sliders"]["left_y"], 480, 
                      lambda x: change_slider("left_y", x) )
    cv.CreateTrackbar("right_y", 'sliders', D["sliders"]["right_y"], 480, 
                      lambda x: change_slider("right_y", x) )

    
    # Points we will look at
    D["p1"] = (200, 240)
    D["p2"] = (440, 240)
    D["x_angle"] = "nan"
    D["y_angle"] = "nan"

    # for display purposes
    D["mode"] = "kinect"
    D["init"] = True
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
    
    value_string = ""
    if D["x_angle"] == "nan":
        value_string = "X Angle: NaN"
    else:
        value_string = ("X Angle: %.1f" % D["x_angle"]) # formatted printing
    
    # display y angle on screen
    value_string2 = ""
    if D["y_angle"] == "nan":
        value_string2 = "Y Angle: NaN"
    else:
        value_string2 = ("Y Angle: %.1f" % D["y_angle"]) # formatted printing

    # draw circles around the points
    cv.Circle(D["image"], D["p1"], 10, D["white"], thickness=1, lineType=8, shift=0)
    cv.Circle(D["image"], D["p1"], 9, D["black"], thickness=1, lineType=8, shift=0)
    cv.Circle(D["image"], D["p1"], 11, D["black"], thickness=1, lineType=8, shift=0)
    
    cv.Circle(D["image"], D["p2"], 10, D["white"], thickness=1, lineType=8, shift=0)
    cv.Circle(D["image"], D["p2"], 9, D["black"], thickness=1, lineType=8, shift=0)
    cv.Circle(D["image"], D["p2"], 11, D["black"], thickness=1, lineType=8, shift=0)

        # display x angle on screen
    cv.Rectangle(D["image"], (213, 17), (426, 97), D["white"], cv.CV_FILLED)
    cv.PutText(D["image"], value_string, (280, 42), D["font"], D["black"])
    cv.PutText(D["image"], value_string2, (280, 82), D["font"], D["black"])


def calculate_angles():
    """ Looks at two points in the depth image and gets the angles in between them """
    global D

    D["p1"] = (D["sliders"]["left_x"], D["sliders"]["left_y"])
    D["p2"] = (D["sliders"]["right_x"], D["sliders"]["right_y"])

    # Find x angles
    x1 = find_x_angle((D["sliders"]["left_x"], D["sliders"]["left_y"]), 
                      (D["sliders"]["right_x"], D["sliders"]["left_y"]))
    
    x2 = find_x_angle((D["sliders"]["left_x"], D["sliders"]["right_y"]),
                      (D["sliders"]["right_x"], D["sliders"]["right_y"]))

    if (x1 == "nan") or (x2 == "nan"):
        D["x_angle"] = "nan"
    else:
        D["x_angle"] = (x1 + x2) / 2.0

    # Find y angles
    y1 = find_y_angle((D["sliders"]["left_x"], D["sliders"]["left_y"]), 
                      (D["sliders"]["left_x"], D["sliders"]["right_y"]))
    
    y2 = find_y_angle((D["sliders"]["right_x"], D["sliders"]["left_y"]),
                      (D["sliders"]["right_x"], D["sliders"]["right_y"]))

    if (y1 == "nan") or (y2 == "nan"):
        D["y_angle"] = "nan"
    else:
        D["y_angle"] = (y1 + y2) / 2.0


def find_y_angle(p1, p2):
    """ Takes in two points as tuples, finds their depths in image and computes the
        y angle """
    global D
    
    deg_per_pix_y = 43.0 / 479.0
    
    # get depths
    d1 = D["image"][p1[1], p1[0]]
    d2 = D["image"][p2[1], p2[0]]

    if math.isnan( d1 ) or math.isnan( d2 ):
        return "nan"

    dz = d1 - d2

    # beta depends on if pixel_y is below or above 240 (halfway point in y)
    b1 = 68.5 + p1[1]*deg_per_pix_y
    if p1[1] > 240:
        b1 = 180 - (68.5 + p1[1]*deg_per_pix_y)

    b2 = 180 - (68.5 + p2[1]*deg_per_pix_y)
    if p2[1] < 240:
        b2 = 68.5 + p2[1]*deg_per_pix_y

    y1 = d1 / (math.tan(math.radians(b1)))
    y2 = d2 / (math.tan(math.radians(b2)))

    dy = y1 + y2
    if (p1[1] < 240 and p2[1] < 240) or (p1[1] > 240 and p2[1] > 240):
        dy = abs(y1 - y2)

    return math.degrees(math.atan2(dz, dy))


def find_x_angle(p1, p2):
    """ Takes in two points as tuples, finds their depths in image and computes the
        x angle """
    global D

    deg_per_pix_x =  57.0 / 639.0

    # get depths
    d1 = D["image"][p1[1], p1[0]] 
    d2 = D["image"][p2[1], p2[0]] 

    if math.isnan( d1 ) or math.isnan( d2):
        return "nan"

    # compute angle between pixels
    dy = d1 - d2

    # beta depends on if pixel_x is below or above 320 (halfway point in x)
    b1 = 61.5 + p1[0]*deg_per_pix_x
    if p1[0] > 320:
        b1 = 180 - (61.5 + p1[0]*deg_per_pix_x)
    
    b2 = 180 - (61.5 + p2[0]*deg_per_pix_x)
    if p2[0] < 320:
        b2 = 61.5 + p2[0]*deg_per_pix_x

    x1 = d1 / (math.tan(math.radians(b1)))
    x2 = d2 / (math.tan(math.radians(b2)))
    
    # dx depends on location of pixel_x of both p1 and p2
    dx = x1 + x2
    if (p1[0] < 320 and p2[0] < 320) or (p1[0] > 320 and p2[0] > 320):
        dx = abs(x1 - x2)

    return math.degrees(math.atan2(dy, dx))
    
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
    """ makes sure keyboard presses are still registered """
    global D

    # init loader will take care of drawing on the image - if we have
    # already drawn on it we don't need to do it again, since it hasn't
    # changed since last time!
    if not D["init"]:
        init_loader()

    if D["changed"]:
        redraw_loaded()
    
    key_press = cv.WaitKey(5) & 255
    if key_press != 255: check_key_press( key_press )

    cv.ShowImage('dist', D["image"])

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

    elif key_press == ord('p'):
        print "X angles: "
        print find_x_angle((D["sliders"]["left_x"], D["sliders"]["left_y"]), 
                           (D["sliders"]["right_x"], D["sliders"]["left_y"]))
        
        print find_x_angle((D["sliders"]["left_x"], D["sliders"]["right_y"]),
                           (D["sliders"]["right_x"], D["sliders"]["right_y"]))
        print "Y angles: "
        print find_y_angle((D["sliders"]["left_x"], D["sliders"]["left_y"]), 
                           (D["sliders"]["left_x"], D["sliders"]["right_y"]))

        print find_y_angle((D["sliders"]["right_x"], D["sliders"]["left_y"]),
                           (D["sliders"]["right_x"], D["sliders"]["right_y"]))
    
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

def change_slider(name, new_point):
    """ changes slider value """
    global D
    D["sliders"][name] = new_point
    D["changed"] = True

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
