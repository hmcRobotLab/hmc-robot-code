#!/usr/bin/env python
from gl import D

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
                      ord('c'): "val_threshed" }

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
        print " p    : start the Finite State Machine - press ' ' to stop"
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

    elif key_press == ord('b'):
        f = open( "./thresh.txt", "r" ) # open the file "thresh.txt" for reading
        data = f.read() # read everything from f into data
        x = eval( data ) # eval is Python's evaluation function
        # eval evaluates strings as if they were at the Python shell
        f.close() # its good to close the file afterwards

        # Set threshold values in D
        D['thresholds'] = x

        # Update threshold values on actual sliders
        for i in ['low_red', 'high_red', 'low_green', 'high_green', 'low_blue', 'high_blue',\
                      'low_hue', 'high_hue', 'low_sat', 'high_sat', 'low_val', 'high_val']:
            cv.SetTrackbarPos(i, 'sliders', D['thresholds'][i])

    elif key_press == ord('e'):
        # Reset sliders to default values (255,0)
        D['thresholds'] = {'low_red':0, 'high_red':255, 'low_green':0, 'high_green':255,\
                               'low_blue':0, 'high_blue':255, 'low_hue':0, 'high_hue':255,\
                               'low_sat':0, 'high_sat':255, 'low_val':0, 'high_val':255 }

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
        D["tank"](0, -100, 100)
    
    elif key_press == 83: # right arrow
        D["tank"](0, 100, -100)

    elif key_press == 82: # up arrow 
        D["tank"](0, 100, 100)

    elif key_press == 84: # down arrow
        D["tank"](0, -100, -100)

    elif key_press == ord(' '):
        D["tank"](0, 0, 0)
        D["state"] = "keyboard"

    elif key_press == ord('p'):
        print "Starting finite state machine"
        D["state"] = "s_head"
        FSM2()

    elif key_press in key_dictionary.keys():
        D["current_threshold"] = key_dictionary[key_press]


# Function for changing the slider values
def change_slider(name, new_threshold):
    global D
    D['thresholds'][name] = new_threshold


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
    threshold_image()
    find_biggest_region()
    mouse_section()
    going_places()

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


def handle_sensor_data( data):
    """ handle_sensor_data is called every time the robot gets a new sensorPacket
    """
    global D

    # check for a bump
    if data.bumpRight or data.bumpLeft:
        print "Bumped!"

    if data.wheeldropCaster:
        print "Wheel drop!"
        self.tank(0, 0 , 0)
        rospy.signal_shutdown("robot picked up... so we're shutting down")

##################### END CALLBACK FUNCTIONS #######################
