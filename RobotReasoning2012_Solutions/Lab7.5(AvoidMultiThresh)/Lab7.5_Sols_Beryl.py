#!/usr/bin/env python
import roslib; roslib.load_manifest('irobot_mudd')
import rospy
import cv_bridge
import cv
import sensor_msgs.msg as sm
from irobot_mudd.srv import *
from irobot_mudd.msg import *
import irobot_mudd
import math, cmath

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
    D["right_coord"] = (0,0)
    D["r_heading"] = 0
    D["n_heading"] = 0
    D["font"] = cv.InitFont(cv.CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5, 0, 2)

    # Create a connection to the Kinect
    D["bridge"] = cv_bridge.CvBridge()

    # variables for vector fields
    D["source"] = (-1,-1)
    D["target"] = (260,280)
    D["obstacle"] = [-1,-1,-1,-1]
    D["alpha"] = 0.5
    # functions for fields:
    def obstacle_func(dist):
        return 50 + 1000.0/(dist)

    def target_func(dist):
        return 100

    D["o_func"] = obstacle_func
    D["t_func"] = target_func

def init_images():
    """ Creates all the images we'll need. Is separate from init_globals 
        since we need to know what size the images are before we can make
        them
    """
    # get D so that we can change values in it
    global D

    # Find the size of the image (we set D["image"] right before calling this function)
    D["size"] = cv.GetSize(D["image"])

    # Create images for each color channel and to save the thresholded images to
    temp_list = ["red", "blue", "green", "hue", "sat", "val", \
                     "red_threshed", "green_threshed", "blue_threshed", \
                     "hue_threshed", "sat_threshed", "val_threshed", \
                     "threshed_image", "threshed_image2"]

    for i in temp_list:
        D[i] = cv.CreateImage(D["size"], 8, 1)

    # Create the hsv image
    D["hsv"] = cv.CreateImage(D["size"], 8, 3)


def ros_services():
    """ sets data members tank and song, analogous to lab 1 """
    global D

    # Obtain the tank service
    rospy.wait_for_service('tank') # won't continue until te "tank" service is on
    D["tank"] = rospy.ServiceProxy('tank', Tank) # tank permits requests, e.g., tank(50,50)
    
    # Obtain the song service
    rospy.wait_for_service('song')
    D["song"] = rospy.ServiceProxy('song', Song)

    # this next line subscribes to sensorPacket
    # it tells ROS that the method handle_sensor_data
    # will handle each incoming sensorPacket
    rospy.Subscriber('sensorPacket', SensorPacket, handle_sensor_data)

################## END INITIALIZATION FUNCTIONS ####################

################### IMAGE PROCESSING FUNCTIONS #####################

def threshold_image_mul(img, thresh):
    """ runs the image processing in order to create a 
        black and white thresholded image out of D["image"]
        into D["threshed_image"]
    """
    # get D so that we can change values in it
    global D

    # Use OpenCV to split the image up into channels,
    # saving them in their respective bw images
    cv.Split(D["image"], D["blue"], D["green"], D["red"], None)

    # This line creates a hue-saturation-value image
    cv.CvtColor(D["image"], D["hsv"], cv.CV_RGB2HSV)
    cv.Split(D["hsv"], D["hue"], D["sat"], D["val"], None)

    # Here is how OpenCV thresholds the images based on the slider values:
    cv.InRangeS(D["red"], D[thresh]["low_red"], \
                    D[thresh]["high_red"], D["red_threshed"])
    cv.InRangeS(D["blue"], D[thresh]["low_blue"], \
                    D[thresh]["high_blue"], D["blue_threshed"])
    cv.InRangeS(D["green"], D[thresh]["low_green"], \
                    D[thresh]["high_green"], D["green_threshed"])
    cv.InRangeS(D["hue"], D[thresh]["low_hue"], \
                    D[thresh]["high_hue"], D["hue_threshed"])
    cv.InRangeS(D["sat"], D[thresh]["low_sat"], \
                    D[thresh]["high_sat"], D["sat_threshed"])
    cv.InRangeS(D["val"], D[thresh]["low_val"], \
                    D[thresh]["high_val"], D["val_threshed"])

    # Multiply all the thresholded images into one "output" image,
    # named D["threshed_image"]
    cv.Mul(D["red_threshed"], D["green_threshed"], D[img])
    cv.Mul(D[img], D["blue_threshed"], D[img])
    cv.Mul(D[img], D["hue_threshed"], D[img])
    cv.Mul(D[img], D["sat_threshed"], D[img])
    cv.Mul(D[img], D["val_threshed"], D[img])

    # Erode and Dilate shave off and add edge pixels respectively
    cv.Erode(D[img], D[img], iterations = 1)
    cv.Dilate(D[img], D[img], iterations = 1)


def find_biggest_region(img):
    """ finds all the contours in threshed image, finds the largest of those,
        and then marks it in the main image, and saves it as the obstacle
    """
    # get D so that we can change values in it
    global D

    # Create a copy image of thresholds then find contours on that image
    storage = cv.CreateMemStorage(0) # Create memory storage for contours
    copy = cv.CreateImage(D["size"], 8, 1)
    cv.Copy( D[img], copy ) # copy threshed image

    # this is OpenCV's call to find all of the contours:
    contours = cv.FindContours(copy, storage, cv.CV_RETR_EXTERNAL, \
                                   cv.CV_CHAIN_APPROX_SIMPLE)

    # Next we want to find the *largest* contour
    if len(contours) > 0:
        biggest = contours
        biggestArea = cv.ContourArea(contours)
        while contours != None:
            nextArea = cv.ContourArea(contours)
            if biggestArea < nextArea:
                biggest = contours
                biggestArea = nextArea
            contours = contours.h_next()
        
        # Use OpenCV to get a bounding rectangle for the largest contour
        br = cv.BoundingRect(biggest, update=0)
        D["obstacle"] = br
        # Draw a box around the largest contour, and a circle at its center
        cv.PolyLine(D["image"], [[(br[0], br[1]), (br[0], br[1] + br[3]), \
                                      (br[0] + br[2], br[1] + br[3]), \
                                      (br[0] + br[2], br[1])]],\
                        1, cv.RGB(255, 0, 0))

        # Draw the circle:
        cv.Circle(D["image"], (br[0] + int(br[2]/2), br[1] + int(br[3]/2)), 10, \
                      cv.RGB(255, 255, 0), thickness=1, lineType=8, shift=0)
        
        # Track the center of the area:
        D["o"] = (br[0] + int(br[2]/2), br[1] + int(br[3]/2))

        # Draw the contours in white with inner ones in green
        cv.DrawContours(D["image"], biggest, cv.RGB(255, 255, 255), \
                            cv.RGB(0, 255, 0), 1, thickness=2, lineType=8, \
                            offset=(0,0))

def find_biggest_region2():
    """ finds all the contours in threshed image, finds the largest of those,
        and then marks in in the main image
    """
    # get D so that we can change values in it
    global D

    # Create a copy image of thresholds then find contours on that image
    storage = cv.CreateMemStorage(0) # Create memory storage for contours
    copy = cv.CreateImage(D["size"], 8, 1)
    cv.Copy( D["threshed_image2"], copy ) # copy threshed image

    # this is OpenCV's call to find all of the contours:
    contours = cv.FindContours(copy, storage, cv.CV_RETR_EXTERNAL, \
                                   cv.CV_CHAIN_APPROX_SIMPLE)
    # Next we want to find the *largest* contour
    if len(contours) > 0:
        biggest = contours
        biggestArea = cv.ContourArea(contours)
        while contours != None:
            nextArea = cv.ContourArea(contours)
            if biggestArea < nextArea:
                biggest = contours
                biggestArea = nextArea
            contours = contours.h_next()
        
        # Use OpenCV to get a bounding rectangle for the largest contour
        br = cv.BoundingRect(biggest, update=0)
        
        # Draw a box around the largest contour, and a circle at its center
        cv.PolyLine(D["image"], [[(br[0], br[1]), (br[0], br[1] + br[3]), \
                                      (br[0] + br[2], br[1] + br[3]), \
                                      (br[0] + br[2], br[1])]],\
                        1, cv.RGB(255, 255, 0))

        # Draw the circle:
        cv.Circle(D["image"], (br[0] + int(br[2]/2), br[1] + int(br[3]/2)), 10, \
                      cv.RGB(255, 0, 0), thickness=1, lineType=8, shift=0)
        
        # Track the center of the area:
        D["curr_loc"] = (br[0] + int(br[2]/2), br[1] + int(br[3]/2))
        D["source"] = (br[0] + int(br[2]/2), br[1] + int(br[3]/2))

        # Draw the contours in white with inner ones in green
        cv.DrawContours(D["image"], biggest, cv.RGB(255, 255, 0), \
                            cv.RGB(0, 255, 0), 1, thickness=2, lineType=8, \
                            offset=(0,0))


def going_places():
    """ draws a magenta circle where the robot will be heading amd computes 
        headers for FSM2
    """
    global D
    # if we are within 20px of where we want to be STOP
    if math.hypot( D["curr_loc"][0] - D["target"][0], \
                       D["curr_loc"][1] - D["target"][1] ) < 20 \
                       and D["state"] != "keyboard":
        D["tank"](0,0) #stop!
        D["state"] = "keyboard"
        print "Close enough to destination!"
    
    # compute header we want and display it
    D["r_heading"] = math.degrees( math.atan2( D["back_loc"][0] - D["for_loc"][0], 
                                               D["back_loc"][1] - D["for_loc"][1] ) )
    cv.Line(D["image"], D["source"], (int(D["source"][0]+D["back_loc"][0]-D["for_loc"][0]), 
                                      int(D["source"][1]+D["back_loc"][1] - D["for_loc"][1])), 
            cv.RGB(255, 255, 255), thickness=3, lineType=8, shift=0)

    # print info in top right corner of image:
    # desired heading
    # current heading
    # distance to destination
    rect_upper_left = (37, 42-25)
    rect_lower_right = (167, 97)
    cv.Rectangle(D["image"], (37, 17), (167, 97), cv.RGB(255,255,255), cv.CV_FILLED)
    value_string = ("Des h: %.1f" % D["n_heading"]) # formatted printing
    value_string2 = ("Cur h: %.1f" % D["r_heading"])
    value_string3 = ("Dest d: %.1f" % math.hypot( D["curr_loc"][0] - D["right_coord"][0], 
                                                  D["curr_loc"][1] - D["right_coord"][1] ) )
    cv.PutText(D["image"], value_string, (42, 42), D["font"], cv.RGB(0,0,255))
    cv.PutText(D["image"], value_string2, (42,62), D["font"], cv.RGB(0,0,255))
    cv.PutText(D["image"], value_string3, (42,82), D["font"], cv.RGB(0,0,255))

def mouse_section():
    """ displays a rectangle defined by dragging the mouse """
    global D

    if D["mouse_down"]:
        x0 = D["down_coord"][0]
        y0 = D["down_coord"][1]
        x1 = D["up_coord"][0]
        y1 = D["up_coord"][1]
        cv.PolyLine(D["image"], [[(x0,y0), (x0,y1), (x1,y1), (x1,y0)]], 1, cv.RGB(255, 255, 0))

def process_section():
    """ calculates the min/max slider values for a given section and sets the
        slider values to them
    """
    global D
    print "sections:", D["sections"]

    if len(D["sections"]) > 0:
        # separate sections by prefix
        adds = [[x[1], x[2]] for x in D["sections"] if x[0] == 'a']
        subs = [[x[1], x[2]] for x in D["sections"] if x[0] == 's']

        # mins default to highest value, maxs default to lowest value
        minmax = [[255,0] for x in xrange(6)]

        for i in adds:            
            # get coords for easy access
            x0 = i[0][0]
            y0 = i[0][1]
            x1 = i[1][0]
            y1 = i[1][1]

            # Go through pixel by pixel
            for x in xrange(x0, x1):
                for y in xrange(y0, y1):
                    in_range = True

                    # the point will be out of range if it is in a subtracted section somewhere
                    for p in subs:
                        if x > p[0][0] and y > p[0][1] and x < p[1][0] and y < p[1][1]:
                            in_range = False

                    if in_range:
                        # Know it isn't part of a subtracted region
                        (b,g,r) = D["image"][y,x]
                        (h,s,v) = D["hsv"][y,x]
                        allv = [r,g,b, h, s, v]
                        for j in xrange(len(allv)):
                            if allv[j] < minmax[j][0]:
                                minmax[j][0] = allv[j]
                            if allv[j] > minmax[j][1]:
                                minmax[j][1] = allv[j]
    
        # Now set sliders to those values
        names = [["low_red", "high_red"], ["low_green", "high_green"],\
                     ["low_blue", "high_blue"], ["low_hue", "high_hue"],\
                     ["low_sat", "high_sat"], ["low_val", "high_val"]]

        for i in xrange(len(names)):
            for j in xrange(2):
                D["thresholds"][names[i][j]] = minmax[i][j]
                cv.SetTrackbarPos(names[i][j], 'sliders', minmax[i][j])
            

def make_vectors():
    """ draws the vectors for vector field movement on image as well as marking
        target and goal """
    if D["source"] != (-1,-1):
        # Draw a circle centered on the source
        cv.Circle(D["image"], (D["source"][0], D["source"][1]), 10, \
                      cv.RGB(255, 127, 0), thickness=3, lineType=8, shift=0)
    if D["target"] != (-1,-1):
        # Draw a circle centered on the target
        cv.Circle(D["image"], (D["target"][0], D["target"][1]), 10, \
                      cv.RGB(255, 0, 255), thickness=3, lineType=8, shift=0)
    if D["obstacle"] != [-1,-1,-1,-1]:
        # Draw a circle centered on the obstacle
        br = D["obstacle"]
        D["o_center"] = (br[0] + int(br[2]/2), br[1] + int(br[3]/2))
        cv.Circle(D["image"], (br[0] + int(br[2]/2), br[1] + int(br[3]/2)), 10, \
                      cv.RGB(0, 255, 255), thickness=3, lineType=8, shift=0)

    if D["source"] != (-1,-1) and D["target"] != (-1,-1) and D["obstacle"] != [-1,-1,-1,-1]:
        # we know where all things are so we can make vectors
        v_t_s = complex(D["target"][0] - D["source"][0],D["target"][1] - D["source"][1])
        v_o_s = complex(D["o_center"][0] - D["source"][0], D["o_center"][1] - D["source"][1])

        p_vts = cmath.polar(v_t_s)
        p_vos = cmath.polar(v_o_s)

        # get new vectors with adjusted magnitude
        v_t_s = cmath.rect(D["t_func"](p_vts[0]), p_vts[1])
        v_o_s = cmath.rect(D["o_func"](p_vos[0]), p_vos[1])
        v_dest = v_t_s - v_o_s
        D["v_dest"] = v_dest

        # make n_heading from v_dest
        D["n_heading"] = math.degrees( math.atan2( v_dest.real, v_dest.imag ) )

        # draw vectors on image from source
        cv.Line(D["image"], D["source"], (int(D["source"][0]+v_t_s.real), int(D["source"][1]+v_t_s.imag)), 
                cv.RGB(255, 0, 255), thickness=3, lineType=8, shift=0)
        cv.Line(D["image"], D["source"], (int(D["source"][0]-v_o_s.real), int(D["source"][1]-v_o_s.imag)), 
                cv.RGB(0, 255, 255), thickness=3, lineType=8, shift=0)
        cv.Line(D["image"], D["source"], (int(D["source"][0] + v_dest.real), int(D["source"][1]+v_dest.imag)),
                cv.RGB(255, 0, 0), thickness=3, lineType=8, shift=0)

################# END IMAGE PROCESSING FUNCTIONS ###################

####################### CALLBACK FUNCTIONS #########################

def onMouse(event, x, y, flags, param):
    """ the method called when the mouse is clicked """
    global D

    if event==cv.CV_EVENT_LBUTTONDOWN: # clicked the left button
        print "x, y are", x, y
        (b,g,r) = D["image"][y,x]
        print "r,g,b is", int(r), int(g), int(b)
        (h,s,v) = D["hsv"][y,x]
        print "h,s,v is", int(h), int(s), int(v)
        D["down_coord"] = (x,y)
        D["mouse_down"] = True

    elif event==cv.CV_EVENT_LBUTTONUP: # let go of the left button
        print "x, y are", x, y
        (b,g,r) = D["image"][y,x]
        print "r,g,b is", int(r), int(g), int(b)
        (h,s,v)  = D["hsv"][y,x]
        print "h,s,v is", int(h), int(s), int(v)
        D["up_coord"] = (x,y)
        D["mouse_down"] = False

        if D["mode"] == "clear":
            D["sections"] = []
        else: # start, add, or subtract
            # put lower coordinates first
            x0 = D["down_coord"][0]
            y0 = D["down_coord"][1]
            x1 = D["up_coord"][0]
            y1 = D["up_coord"][1]

            if x0 > x1:
                x0, x1 = x1, x0
            if y0 > y1:
                y0, y1 = y1, y0
 
            if D["mode"] == "start":
                D["sections"] = []
            mode_dict = {"start":'a', "add":'a', "subtract":'s'}
            D["sections"].append([mode_dict[D["mode"]], (x0, y0), (x1, y1)])
            process_section()

    elif D["mouse_down"] and event==cv.CV_EVENT_MOUSEMOVE: # mouse just moved
        D["up_coord"] = (x,y)

    elif event==cv.CV_EVENT_RBUTTONDOWN:
        D["right_coord"] = (x,y)
        # set the target
        D["target"] = (x,y)

def check_key_press(key_press):
    """ this handler is called when a real key press has been
        detected, and updates everything appropriately
    """
    global D
    D["last_key_pressed"] = key_press

    # So we know what we'll do if we get one of these key presses
    key_dictionary = {ord('w'): "threshed_image",\
                      ord('r'): "red",\
                      ord('t'): "green",\
                      ord('y'): "blue",\
                      ord('f'): "red_threshed",\
                      ord('g'): "green_threshed",\
                      ord('h'): "blue_threshed",\
                      ord('a'): "hue",\
                      ord('s'): "sat",\
                      ord('d'): "val",\
                      ord('z'): "hue_threshed",\
                      ord('x'): "sat_threshed",\
                      ord('c'): "val_threshed"}
                      

    if key_press == ord('q') or key_press == 27: # if a 'q' or ESC was pressed
        print "quitting"
        rospy.signal_shutdown( "Quit requested from keyboard" )

    elif key_press == ord('l'):
        print " Keyboard Command Menu"
        print " =============================="
        print " q    : quit"
        print " ESC  : quit"
        print " l    : help menu"
        print " e    : reset sliders to default values"
        print " w    : show total threshold image in threshold window"
        print " m    : show total threshold image 2 in threshold window"
        print " r    : show red image in threshold window"
        print " t    : show green image in threshold window"
        print " y    : show blue image in threshold window"
        print " f    : show thresholded red image in threshold window"
        print " g    : show thresholded blue image in threshold window"
        print " h    : show thresholded green image in threshold window"
        print " a    : show hue image in threshold window"
        print " s    : show saturation image in threshold window"
        print " d    : show value image in threshold window"
        print " z    : show thresholded hue image in threshold window"
        print " x    : show thresholded saturation image in threshold window"
        print " c    : show thresholded value image in threshold window"
        print " v    : prints threshold values to file ( will overwrite old file )"
        print " b    : loads threshold values from file"
        print " u    : mousedrags will no longer set thresholds, kept values will be cleared"
        print " i    : mousedrag will set thresholds to area within drag, " + \
            "resets on new click or drag"
        print " j    : mousedrags will remove the area under consideration, " + \
            " must have set an area in 'i' mode first"
        print " k    : mousedrags will add the area under consideration, " + \
            " must have set an area in 'i' mode first"
        print " p    : start the Finite State Machine 1 - press ' ' to stop"
        print " o    : start the Finite State Machine 2 - press ' ' to stop"
        print " ' '  : goes into keyboard mode, stops robot from moving"
        print " l arr: keyboard mode - rotate robot left"
        print " r arr: keyboard mode - rotate robot right"
        print " u arr: keyboard mode - move robot forward"
        print " d arr: keyboard mode - move robot down"

    elif key_press == ord('v'):
        x = D['thresholds']
        f = open( "./thresh.txt", "w" ) # open the file "thresh.txt" for writing
        print >> f, x # print x to the file object f
        f.close() # it's good to close the file afterwards

        x = D['thresholds2']
        f = open( "./thresh2.txt", "w" ) # open the file "thresh.txt" for writing
        print >> f, x # print x to the file object f
        f.close() # it's good to close the file afterwards

    elif key_press == ord('b'):
        f = open( "./thresh.txt", "r" ) # open the file "thresh.txt" for reading
        data = f.read() # read everything from f into data
        x = eval( data ) # eval is Python's evaluation function
        # eval evaluates strings as if they were at the Python shell
        f.close() # its good to close the file afterwards

        # Set threshold values in D
        D['thresholds'] = x

        # Update threshold values on actual sliders
        for i in D["key_list"]:
            cv.SetTrackbarPos(i, 'sliders', D['thresholds'][i])

        f = open( "./thresh2.txt", "r" ) # open the file "thresh.txt" for reading
        data = f.read() # read everything from f into data
        x = eval( data ) # eval is Python's evaluation function
        # eval evaluates strings as if they were at the Python shell
        f.close() # its good to close the file afterwards

        # Set threshold values in D
        D['thresholds2'] = x

        # Update threshold values on actual sliders
        for i in D["key_list"]:
            cv.SetTrackbarPos(i, 'sliders2', D['thresholds2'][i])

    elif key_press == ord('e'):
        # Reset sliders to default values (255,0)
        D['thresholds'] = {'low_red':0, 'high_red':255, 'low_green':0, 'high_green':255,\
                               'low_blue':0, 'high_blue':255, 'low_hue':0, 'high_hue':255,\
                               'low_sat':0, 'high_sat':255, 'low_val':0, 'high_val':255 }
        D['thresholds2'] = {'low_red':0, 'high_red':255, 'low_green':0, 'high_green':255,\
                               'low_blue':0, 'high_blue':255, 'low_hue':0, 'high_hue':255,\
                               'low_sat':0, 'high_sat':255, 'low_val':0, 'high_val':255 }
        for i in D["key_list"]:
            cv.SetTrackbarPos(i, 'sliders', D['thresholds'][i])
            cv.SetTrackbarPos(i, 'sliders2', D['thresholds2'][i])

    elif key_press == ord('u'):
        D["mode"] = "clear"
        D["sections"] = []

    elif key_press == ord('i'):
        D["mode"] = "start"

    elif key_press == ord('j'):
        if len(D["sections"]) > 0:
            D["mode"] = "subtract"
        else:
            print "Cannot switch modes, need a starting area first. Press 'i' " + \
                "to select a starting area."

    elif key_press == ord('k'):
        if len(D["sections"]) > 0:
            D["mode"] = "add"
        else:
            print "Cannot switch modes, need a starting area first. Press 'i' " + \
                 "to select a starting area."

    elif key_press == 81: # left arrow
        D["tank"](-100, 100)
    
    elif key_press == 83: # right arrow
        D["tank"](100, -100)

    elif key_press == 82: # up arrow 
        D["tank"](100, 100)

    elif key_press == 84: # down arrow
        D["tank"](-100, -100)

    elif key_press == ord(' '):
        D["tank"](0, 0)
        D["state"] = "keyboard"

    elif key_press == ord('o'):
        print "Starting finite state machine"
        D["state"] = "s_head"
        state_start_head()

    elif key_press in key_dictionary.keys():
        D["current_threshold"] = key_dictionary[key_press]


# Function for changing the slider values
def change_slider(name, new_threshold):
    global D
    D['thresholds'][name] = new_threshold

def change_slider2(name, new_threshold):
    global D
    D['thresholds2'][name] = new_threshold

def handle_data(data):
    """ this method processes data given:
        - key presses
        - images from Kinect
    """
    global D

    # Get the incoming image from the Kinect
    D["image"] = D["bridge"].imgmsg_to_cv(data, "bgr8")

    # Init additional images we need for processing - only need to run once
    if D["created_images"] == False:
        init_images()
        D["created_images"] = True

    # Recalculate images + all extra drawing functions
    threshold_image_mul("threshed_image", "thresholds")
    threshold_image_mul("threshed_image2", "thresholds2")
    find_biggest_region("threshed_image")
    find_biggest_region2()
    mouse_section()
    going_places() # for fsm2
    make_vectors() # for vector fields

    # Get any incoming keypresses
    # To get input from keyboard, we use cv.WaitKey
    # Only the lowest eight bits matter (so we get rid of the rest):
    key_press_raw = cv.WaitKey(5) # gets a raw key press
    key_press = key_press_raw & 255 # sets all but the low 8 bits to 0
    
    # Handle key presses only if it's a real key (255 = "no key pressed")
    if key_press != 255:
        check_key_press(key_press)

    # Update the displays:
    cv.ShowImage('image', D["image"])
    cv.ShowImage('threshold', D[ D["current_threshold"] ] )
    cv.ShowImage('threshold2', D["threshed_image2"])


def handle_sensor_data( data):
    """ handle_sensor_data is called every time the robot gets a new sensorPacket
    """
    global D

    # check for a bump
    if data.bumpRight or data.bumpLeft:
        print "Bumped!"

    if data.wheeldropCaster:
        print "Wheel drop!"
        self.tank(0 , 0)
        rospy.signal_shutdown("robot picked up... so we're shutting down")

##################### END CALLBACK FUNCTIONS #######################

##################### STATE MACHINE FUNCTIONS ######################

def transition( time, next_state ):
    """ time is a float, next_state is a funcion """
    if D["state"] == "keyboard":
        D["tank"](0,0)
        return
    rospy.Timer( rospy.Duration(time), next_state, oneshot=True )

# Used in first heading
def state_start_head( timer_event=None ):
    """ starts finding the heading """
    global D

    print "Now finding a heading!"
    D["for_loc"] = D["curr_loc"]
    D["tank"](50,50)
    transition(2.0, state_end_head)

# Used in first heading
def state_end_head( timer_event=None ):
    """finishes finding the heading """
    global D
    print "Now finishing finding the heading!"
    D["back_loc"] = D["curr_loc"]
    D["tank"](-50,-50)
    transition(2.0, state_first_turn)


def state_first_turn( timer_event=None ):
    """ making first turn so we face the wanted heading"""
    global D
    deg = 40

    # make the robots current heading at theta = 0
    # get angle : is dist btwn new and origin plus dost btwn old and origin
    #angle = 180 - D["n_heading"] + 180 - D["r_heading"]
    angle = max(D["n_heading"], D["r_heading"]) - min(D["n_heading"], D["r_heading"])

    if angle > 359:
        angle = angle - 360

    print "Now in first turn!"

    if D["n_heading"] > D["r_heading"]:
        D["tank"](-100, 100)
    else:
        D["tank"](100, -100)

    x = angle / deg
    transition(x, state_go)

# Used in whenever heading
def state_head1( timer_event=None ):
    """ starts finding the heading """
    global D

    print "Now finding a heading!"
    D["for_loc"] = D["curr_loc"]
    D["tank"](50,50)
    transition(2.0, state_head2)

# Used in whenever heading
def state_head2( timer_event=None ):
    """finishes finding the heading """
    global D
    print "Now finishing finding the heading!"
    D["back_loc"] = D["curr_loc"]
    D["tank"](-50,-50)
    transition(2.0, state_go)

def state_forward( timer_event=None ):
    """ just goes forward... """
    global D
    print "Now going forward"
    D["tank"](50, 50)
    D["state"] = "keyboard"

def state_go( timer_event=None ):
    """ Goes forward for a little bit """
    global D
    print "Now going forward!"

    #translate n_heading into local coordinates
    theta = math.degrees( math.atan2( D["v_dest"].real, D["v_dest"].imag ) ) 
    (r,temp) = cmath.polar(D["v_dest"])
    loc_dest = cmath.rect(r, theta+math.radians(D["r_heading"]))

    tr = loc_dest.real
    rot = loc_dest.imag

    #D["tank"](int(tr - D["alpha"]*rot),int(tr + D["alpha"]*rot))
    D["tank"](100, 100)
    transition(1.0, state_start_head)

################### END STATE MACHINE FUNCTIONS ####################

def main():
    """ the main program that sets everything up
    """
    
    # Initialize our node, variables
    rospy.init_node('blobFinder')
    init_globals()

    # Subscribe to the image_color topic - each time new data comes in 
    # (Kinect color image, keypresses), we will call handle_data to deal with it
    rospy.Subscriber('/camera/rgb/image_color', sm.Image, handle_data)

    # Get robot services
    ros_services()

    # Run until something stops us, such as a call to rospy.signal_shutdown
    rospy.spin()

# this is the "main" trick: it tells Python
# what code to run when you run this as a stand-alone script:
if __name__ == "__main__":
    main()
