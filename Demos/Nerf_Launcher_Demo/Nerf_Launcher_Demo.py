#!/usr/bin/env python
import roslib; roslib.load_manifest('Frizzle')
import rospy
import cv_bridge
import cv
import sensor_msgs.msg as sm
import time

import os
import sys
import usb.core

############# INSTRUCTIONS ##################
#
# have missile launcher plugged in
# run 'roscore'
# run 'sudo lab8.py'
#
#############################################

# Dictionary to hold all globals in
D = {}
CODE_UP = [0x02,0x02,0x00,0x00,0x00,0x00,0x00,0x00]
CODE_DOWN = [0x02,0x01,0x00,0x00,0x00,0x00,0x00,0x00]
CODE_LEFT = [0x02,0x04,0x00,0x00,0x00,0x00,0x00,0x00]
CODE_RIGHT = [0x02,0x08,0x00,0x00,0x00,0x00,0x00,0x00]
CODE_STOP = [0x02,0x20,0x00,0x00,0x00,0x00,0x00,0x00]
CODE_FIRE = [0x02,0x10,0x00,0x00,0x00,0x00,0x00,0x00]

class launchControl():
   
   def __init__(self):
      self.connectToLauncher();

   def connectToLauncher(self):
      self.launcher = usb.core.find(idVendor=0x2123, idProduct=0x1010)
      if self.launcher is None:
         raise ValueError('Launcher not found.')
      if self.launcher.is_kernel_driver_active(0) is True:
         self.launcher.detach_kernel_driver(0)
      self.launcher.set_configuration()
      
   def sendCodeToLauncher(self, code):
      self.launcher.ctrl_transfer(0x21,0x09,0,0,code)


#################### INITIALIZATION FUNCTIONS ######################
def init_globals():
    """ sets up all the globals in the dictionary D
    """
    # get D so that we can change values in it
    global D

    # put threshold values into D
    D['thresholds'] = {'low_red':0, 'high_red':256,\
                       'low_green':0, 'high_green':256,\
                       'low_blue':0, 'high_blue':256,\
                       'low_hue':0, 'high_hue':256,\
                       'low_sat':0, 'high_sat':256,\
                       'low_val':0, 'high_val':256 }

    # Set up the windows containing the image from the kinect,
    # the altered image, and the threshold sliders.
    cv.NamedWindow('image')
    cv.MoveWindow('image', 964, 0)
    cv.NamedWindow('threshold')
    cv.MoveWindow('threshold', 322, 0)
    cv.NamedWindow('sliders')
    cv.MoveWindow('sliders', 0, 0)

    # Create the sliders within the 'sliders' window
    D["key_list"] = ["low_red", "high_red", "low_green", "high_green",
                     "low_blue", "high_blue", "low_hue", "high_hue",
                     "low_sat", "high_sat", "low_val", "high_val"]
    D["small_key_list"] = ["red", "green", "blue", "hue", "sat", "val"]

    for i in D["key_list"]:
       cv.CreateTrackbar(i, 'sliders', D['thresholds'][i], 256,
                         lambda x, j=i: change_slider(j, x) )

    # Set the method to handle mouse button presses
    cv.SetMouseCallback('image', onMouse, None)

    # We have not created our "scratchwork" images yet
    D["created_images"] = False

    # Variable for key presses - set it to something that could not have been pressed
    D["last_key_pressed"] = 255
    D["diff"] = 40

    # The current image we want to display in the threshold window
    D["current_threshold"] = "threshed_image"

    # Create connections to launcher
    D["bridge"] = cv_bridge.CvBridge()
    D["camera"] = cv.CaptureFromCAM(-1)
    D["running"] = True
    D['launchControl'] = launchControl()

    # Variables for following things
    D["center"] = (430, 400)
    D["current"] = (0, 0)
    D["state"] = "keyboard"
    D["num_wait"] = 0
    D["exit_boundary"] = CODE_RIGHT


def init_images():
    """ Creates all the images we'll need. Is separate from init_globals 
        since we need to know what size the images are before we can make
        them
    """
    # get D so that we can change values in it
    global D

    # Find the size of the image 
    # (we set D["image"] right before calling this function)
    D["size"] = cv.GetSize(D["image"])

    # Create images for each color channel, thresholded images, final result
    D["img_list"] = ["red", "green", "blue", "hue", "sat", "val", "red_threshed",
                     "green_threshed", "blue_threshed", "hue_threshed",
                     "sat_threshed", "val_threshed", "threshed_image"]

    for i in D["img_list"]:
       D[i] = cv.CreateImage(D["size"], 8, 1)

    # Create the hsv image
    D["hsv"] = cv.CreateImage(D["size"], 8, 3)


################## END INITIALIZATION FUNCTIONS ####################

################### IMAGE PROCESSING FUNCTIONS #####################
def threshold_image():
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
    for i in D["small_key_list"]:
       cv.InRangeS(D[i], D['thresholds']["low_"+i], D['thresholds']["high_"+i],
                   D[i+"_threshed"])

    # Multiply all the thresholded images into one "output" image,
    # named D["threshed_image"]
    cv.Mul(D["red_threshed"], D["green_threshed"], D["threshed_image"])
    for i in D["small_key_list"][2:]:
       cv.Mul(D[i+"_threshed"], D["threshed_image"], D["threshed_image"])

    # Erode and Dilate shave off and add edge pixels respectively
    cv.Erode(D["threshed_image"], D["threshed_image"], iterations = 1)
    cv.Dilate(D["threshed_image"], D["threshed_image"], iterations = 1)


def find_biggest_region():
    """ finds all the contours in threshed image, finds the largest of those,
        and then marks in in the main image
    """
    global D

    # Create a copy image of thresholds then find contours on that image
    storage = cv.CreateMemStorage(0) # Create memory storage for contours
    copy = cv.CreateImage(D["size"], 8, 1)
    cv.Copy( D["threshed_image"], copy ) # copy threshed image

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

        # Example of drawing a red box
        # Variables: LeftBound, RightBound, TopBound, BottomBound
        D["LB"], D["RB"], D["TB"], D["BB"] = (br[0], br[0]+br[2], br[1], br[1]+br[3])
        cv.PolyLine(D["image"], [[(D["LB"],D["TB"]), (D["RB"],D["TB"]), 
                                  (D["RB"],D["BB"]), (D["LB"],D["BB"])]], 
                    1, cv.RGB(255, 0, 0))

        # Example of drawing a yellow circle
        # Variables: XCenter, YCenter
        D["XC"], D["YC"] = (0.5*(D["LB"]+D["RB"]), 0.5*(D["TB"]+D["BB"]))
        D["current"] = (D["XC"], D["YC"])
        cv.Circle(D["image"], (D["XC"],D["YC"]), 8, cv.RGB(255, 255, 0), 
                  thickness=1, lineType=8, shift=0)

        # Draw the contours in white with inner ones in green
        cv.DrawContours(D["image"], biggest, cv.RGB(255, 255, 255), 
                        cv.RGB(0, 255, 0), 1, thickness=2, lineType=8, 
                        offset=(0,0))
    else:
       D["current"] = (-1, -1)

################# END IMAGE PROCESSING FUNCTIONS ###################

####################### CALLBACK FUNCTIONS #########################
def onMouse(event, x, y, flags, param):
    """ the method called when the mouse is clicked """

    if event==cv.CV_EVENT_LBUTTONDOWN: # clicked the left button
        print "x, y are", x, y
        (b,g,r) = D["image"][y,x]
        print "r,g,b is", int(r), int(g), int(b)
        (h,s,v) = D["hsv"][y,x]
        print "h,s,v is", int(h), int(s), int(v)

    elif event==cv.CV_EVENT_RBUTTONDOWN: # clicked the right button
       print "x,y are", x, y
       (b,g,r) = D["image"][y,x]
       print "r,g,b is", int(r), int(g), int(b)
       (h,s,v) = D["hsv"][y,x]
       print "h,s,v is", int(h), int(s), int(v)
       get_lock([int(r), int(g), int(b), int(h), int(s), int(v)])

def get_lock(val_list):
   """ sets the thresholds to somewhere around the given values """
   global D
   for i in xrange(len(D["small_key_list"])):
      D['thresholds']['low_'+D["small_key_list"][i]] = val_list[i] - D["diff"]
      D['thresholds']['high_'+D["small_key_list"][i]] = val_list[i] + D["diff"]
      cv.SetTrackbarPos('low_'+D["small_key_list"][i], 'sliders', 
                        D['thresholds']['low_'+D["small_key_list"][i]])
      cv.SetTrackbarPos('high_'+D["small_key_list"][i], 'sliders', 
                        D['thresholds']['high_'+D["small_key_list"][i]])

def check_key_press(key_press):
    """ this handler is called when a real key press has been
        detected, and updates everything appropriately
    """
    # get D so that we can change values in it
    global D
    D["last_key_pressed"] = key_press

    # So we know what we'll do if we get one of these key presses
    key_dictionary = {ord('t'): "threshed_image",\
                      ord('r'): "red",\
                      ord('g'): "green",\
                      ord('b'): "blue",\
                      ord('y'): "red_threshed",\
                      ord('u'): "green_threshed",\
                      ord('i'): "blue_threshed" }

    control_dictionary = {82:CODE_UP, 84:CODE_DOWN, 81:CODE_LEFT, 83:CODE_RIGHT,
                          32:CODE_FIRE, ord('z'):CODE_STOP}

    if key_press == ord('q') or key_press == 27: # if a 'q' or ESC was pressed
        print "quitting"
        D["running"] = False

    elif key_press in control_dictionary.keys():
            D['launchControl'].sendCodeToLauncher(control_dictionary[key_press])
            D["state"] = "keyboard"

    elif key_press == ord('h'):
        print " Keyboard/Mouse Command Menu"
        print " =============================="
        print " q    : quit"
        print " ESC  : quit"
        print " h    : help menu"
        print " t    : show total threshold image in threshold window"
        print " r    : show red image in threshold window"
        print " g    : show green image in threshold window"
        print " b    : show blue image in threshold window"
        print " y    : show red threshed image"
        print " u    : show green threshed image"
        print " i    : show blue threshed image"
        print " s    : save the current threshold settings"
        print " l    : load a previously saved threshold"
        print " z    : stops movement of the launcher"
        print " up arrow     : tilts launcher up"
        print " down arrow   : tilts launcher down"
        print " left arrow   : turns launcher clockwise"
        print " right arrow  : turns launcher counter-clockwise"
        print " spacebar     : fires a dart and reloads"
        print " left click   : show current x,y,r,g,b,h,s,v values"
        print " right click  : set rgb,hsv values at click to center of thresholds"
        print " ,    : decrement sensitivity of right click"
        print " .    : increment sensitivity of left click"
        print " o    : starts the state machine, press arrows or z to stop"
        
    elif key_press == ord('s'):
        x = D['thresholds']   # the value we will save
        f = open( "./thresh.txt", "w" )   # open the file "thresh.txt" for writing
        print >> f, x   # print x to the file object f
        f.close()   # it's good to close the file afterwards
        print "Saving current slider threshold values..."
        
    elif key_press == ord('l'):
        f = open( "./thresh.txt", "r" )   # open the file "thresh.txt" for reading
        data = f.read()   # read everything from f into data
        x = eval( data )  # eval is Python's evaluation functio
        f.close()   # it's good to close the file afterwards
        D['thresholds'] = x

        for i in D["key_list"]:
           cv.SetTrackbarPos(i, 'sliders', D['thresholds'][i])
        print "Loading saved slider threshold values..."

    elif key_press == ord(','):
       if D["diff"] > 5:
          D["diff"] = D["diff"] - 5
          print "New sensitivity", D["diff"]

    elif key_press == ord('.'):
       if D["diff"] < 250:
          D["diff"] = D["diff"] + 5
          print "New sensitivity", D["diff"]

    elif key_press == ord('o'):
       D["state"] = "machine"
       state_moving()

    elif key_press in key_dictionary.keys(): # If we said what the key_press maps to already
        D["current_threshold"] = key_dictionary[key_press]


def change_slider(name, new_threshold):
    """ changes the slider values given the name of the slider and the new value """
    global D
    D['thresholds'][name] = new_threshold


def handle_data():
    """ this method processes data given:
        - key presses
        - images from webcam
    """
    global D

    while D["running"]:

        # Grab incoming image
        D["image"] = cv.QueryFrame(D['camera'])

        if D["created_images"] == False:
            # Initialize the additional images we need for processing
            # We only need to run this one time
            init_images()
            D["created_images"] = True

        # Recalculate threshold image
        threshold_image()

        # Recalculate blob in main image
        find_biggest_region()

        # Get any incoming keypresses
        # Only the lowest eight bits matter (so we get rid of the rest):
        key_press_raw = cv.WaitKey(5) # gets a raw key press
        key_press = key_press_raw & 255 # sets all but the low 8 bits to 0
        
        # Handle key presses only if it's a real key (255 = "no key pressed")
        if key_press != 255:
            check_key_press(key_press)

        # Update the displays:
        cv.ShowImage('image', D["image"])
        cv.ShowImage('threshold', D[ D["current_threshold"] ] )


##################### END CALLBACK FUNCTIONS #######################

################# FINITE STATE MACHINE FUNCTIONS ###################

def transition( time, next_state):
   """ time is a float, next_state is a function """
   global D
   if D["running"] and D["state"] != "keyboard":
      rospy.Timer( rospy.Duration(time), next_state, oneshot=True )
      return
   D['launchControl'].sendCodeToLauncher(CODE_STOP)

def speed(dist, d):
   if abs(dist) > 200:
      if d == "x" and dist < 0:
         D["exit_boundary"] = CODE_LEFT
      elif d == "x" and dist > 0:
         D["exit_boundary"] = CODE_RIGHT
      elif d == "y" and dist < 0:
         D["exit_boundary"] = CODE_UP
      else:
         D["exit_boundary"] = CODE_DOWN
      return 0.0004
   elif abs(dist) > 100:
      return 0.0003
   else:
      return 0.0002

def sign(num):
   if num < 0:
      return 0
   else:
      return 1

def state_moving( timer_event=None ):
   """ Still moving to center target """
   global D

   print "In moving state"

   dx = D["current"][0] - D["center"][0]
   dy = D["current"][1] - D["center"][1]

   code_dict = {dx:(CODE_LEFT, CODE_RIGHT), dy:(CODE_UP, CODE_DOWN)}
   if D["current"][0] == -1 and D["current"][1] == -1:
      wait_time = 0.01
      transition(wait_time, state_scan)
      return
   elif abs(dx) > 30:
      print "x direction!"
      D['launchControl'].sendCodeToLauncher(code_dict[dx][sign(dx)])
      time.sleep(speed(dx, "x"))
      D['launchControl'].sendCodeToLauncher(CODE_STOP)
      wait_time = 0.01
      transition(wait_time, state_moving)
      return
   elif abs(dy) > 30:
      print "y direction!"
      D['launchControl'].sendCodeToLauncher(code_dict[dy][sign(dy)])
      time.sleep(speed(dy, "y"))
      D['launchControl'].sendCodeToLauncher(CODE_STOP)
      wait_time = 0.01
      transition(wait_time, state_moving)
      return
   elif abs(dy) < 30 and abs(dy) < 30:
      print "done moving"
      D['launchControl'].sendCodeToLauncher(CODE_STOP)
      wait_time = 0.01
      transition(wait_time, state_check_move)
      return

def state_check_move( timer_event=None ):
   """ checks if the center has moved """
   global D

   print "In waiting state"

   dx = D["current"][0] - D["center"][0]
   dy = D["current"][1] - D["center"][1]
   
   if D["num_wait"] == 30:
      D["num_wait"] = 0
      D["state"] = "keyboard"
      print "Exiting FSM"
      D['launchControl'].sendCodeToLauncher(CODE_FIRE)
      return

   if abs(dx) < 30 and abs(dy) < 30:
      if D["num_wait"] < 35:
         D["num_wait"] = D["num_wait"] + 1
      transition(0.01, state_check_move)
      return
   else:
      D["num_wait"] = 0
      transition(0.01, state_moving)

def state_scan( timer_event=None ):
   """ scans area looking for the target """
   global D

   print "In scanning state"
   if D["current"][0] != -1 and D["current"][1] != -1:
      "Found a target"
      transition(0.01, state_moving)
      return
   else:
      D['launchControl'].sendCodeToLauncher(D["exit_boundary"])
      time.sleep(0.0004)
      D['launchControl'].sendCodeToLauncher(CODE_STOP)
      transition(0.01, state_scan)

############### END FINITE STATE MACHINE FUNCTIONS #################

def main():
    """ the main program that sets everything up
    """    
    # Initialize our node
    rospy.init_node('blobFinder')

    # Initialize all the global variables we will need
    init_globals()

    # Call the function that begins our process
    handle_data()


# this is the "main" trick: it tells Python
# what code to run when you run this as a stand-alone script:
if __name__ == "__main__":
    main()
