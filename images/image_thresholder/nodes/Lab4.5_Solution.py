#!/usr/bin/env python
import roslib; roslib.load_manifest('irobot_mudd')
import rospy
import cv_bridge
import cv
import sensor_msgs.msg as sm

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

    # Set up the windows containing the image from the kinect,
    # the altered image, and the threshold sliders.
    cv.NamedWindow('image')
    cv.MoveWindow('image', 0, 0)
    cv.NamedWindow('threshold')
    cv.MoveWindow('threshold', 640, 0)
    cv.NamedWindow('sliders')
    cv.MoveWindow('sliders', 1280, 0)

    # Create the sliders within the 'sliders' window
    cv.CreateTrackbar('low_red', 'sliders', D['thresholds']['low_red'], 255, \
                          lambda x: change_slider('low_red', x) )
    cv.CreateTrackbar('high_red', 'sliders', D['thresholds']['high_red'], 255, \
                          lambda x: change_slider('high_red', x) )
    cv.CreateTrackbar('low_green', 'sliders', D['thresholds']['low_green'], 255, \
                          lambda x: change_slider('low_green', x) )
    cv.CreateTrackbar('high_green', 'sliders', D['thresholds']['high_green'], 255, \
                          lambda x: change_slider('high_green', x) )
    cv.CreateTrackbar('low_blue', 'sliders', D['thresholds']['low_blue'], 255, \
                          lambda x: change_slider('low_blue', x) )
    cv.CreateTrackbar('high_blue', 'sliders', D['thresholds']['high_blue'], 255, \
                          lambda x: change_slider('high_blue', x) )
    cv.CreateTrackbar('low_hue', 'sliders', D['thresholds']['low_hue'], 255, \
                          lambda x: change_slider('low_hue', x) )
    cv.CreateTrackbar('high_hue', 'sliders', D['thresholds']['high_hue'], 255, \
                          lambda x: change_slider('high_hue', x) )
    cv.CreateTrackbar('low_sat', 'sliders', D['thresholds']['low_sat'], 255, \
                          lambda x: change_slider('low_sat', x) )
    cv.CreateTrackbar('high_sat', 'sliders', D['thresholds']['high_sat'], 255, \
                          lambda x: change_slider('high_sat', x) )
    cv.CreateTrackbar('low_val', 'sliders', D['thresholds']['low_val'], 255, \
                          lambda x: change_slider('low_val', x) )
    cv.CreateTrackbar('high_val', 'sliders', D['thresholds']['high_val'], 255, \
                          lambda x: change_slider('high_val', x) )

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

    # Create a connection to the Kinect
    D["bridge"] = cv_bridge.CvBridge()


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

    # Create images for each color channel
    D["red"] = cv.CreateImage(D["size"], 8, 1)
    D["blue"] = cv.CreateImage(D["size"], 8, 1)
    D["green"] = cv.CreateImage(D["size"], 8, 1)
    D["hue"] = cv.CreateImage(D["size"], 8, 1)
    D["sat"] = cv.CreateImage(D["size"], 8, 1)
    D["val"] = cv.CreateImage(D["size"], 8, 1)

    # Create images to save the thresholded images to
    D["red_threshed"] = cv.CreateImage(D["size"], 8, 1)
    D["green_threshed"] = cv.CreateImage(D["size"], 8, 1)
    D["blue_threshed"] = cv.CreateImage(D["size"], 8, 1)
    D["hue_threshed"] = cv.CreateImage(D["size"], 8, 1)
    D["sat_threshed"] = cv.CreateImage(D["size"], 8, 1)
    D["val_threshed"] = cv.CreateImage(D["size"], 8, 1)

    # The final thresholded result
    D["threshed_image"] = cv.CreateImage(D["size"], 8, 1)

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
    cv.InRangeS(D["red"], D['thresholds']["low_red"], \
                    D['thresholds']["high_red"], D["red_threshed"])
    cv.InRangeS(D["blue"], D['thresholds']["low_blue"], \
                    D['thresholds']["high_blue"], D["blue_threshed"])
    cv.InRangeS(D["green"], D['thresholds']["low_green"], \
                    D['thresholds']["high_green"], D["green_threshed"])
    cv.InRangeS(D["hue"], D['thresholds']["low_hue"], \
                    D['thresholds']["high_hue"], D["hue_threshed"])
    cv.InRangeS(D["sat"], D['thresholds']["low_sat"], \
                    D['thresholds']["high_sat"], D["sat_threshed"])
    cv.InRangeS(D["val"], D['thresholds']["low_val"], \
                    D['thresholds']["high_val"], D["val_threshed"])

    # Multiply all the thresholded images into one "output" image,
    # named D["threshed_image"]
    cv.Mul(D["red_threshed"], D["green_threshed"], D["threshed_image"])
    cv.Mul(D["threshed_image"], D["blue_threshed"], D["threshed_image"])
    cv.Mul(D["threshed_image"], D["hue_threshed"], D["threshed_image"])
    cv.Mul(D["threshed_image"], D["sat_threshed"], D["threshed_image"])
    cv.Mul(D["threshed_image"], D["val_threshed"], D["threshed_image"])

    # Erode and Dilate shave off and add edge pixels respectively
    cv.Erode(D["threshed_image"], D["threshed_image"], iterations = 1)
    cv.Dilate(D["threshed_image"], D["threshed_image"], iterations = 1)


def find_biggest_region():
    """ finds all the contours in threshed image, finds the largest of those,
        and then marks in in the main image
    """
    # get D so that we can change values in it
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
        
        # Draw a box around the largest contour, and a circle at its center
        cv.PolyLine(D["image"], [[(br[0], br[1]), (br[0], br[1] + br[3]), \
                                      (br[0] + br[2], br[1] + br[3]), \
                                      (br[0] + br[2], br[1])]],\
                        1, cv.RGB(255, 0, 0))

        # Draw the circle:
        cv.Circle(D["image"], (br[0] + int(br[2]/2), br[1] + int(br[3]/2)), 10, \
                      cv.RGB(255, 255, 0), thickness=1, lineType=8, shift=0)

        # Draw the contours in white with inner ones in green
        cv.DrawContours(D["image"], biggest, cv.RGB(255, 255, 255), \
                            cv.RGB(0, 255, 0), 1, thickness=2, lineType=8, \
                            offset=(0,0))

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
        # pull out all sections with 'a' in the front
        adds = [[x[1], x[2]] for x in D["sections"] if x[0] == 'a']

        # pull out all sections with a 's' in the front
        subs = [[x[1], x[2]] for x in D["sections"] if x[0] == 's']

        print "adds: ", adds
        print "subs: ", subs
        # mins default to highest value, maxs default to lowest value
        minmax = [[255,0] for x in xrange(6)]

        for i in adds:
            
            # get coords for easy access
            x0 = i[0][0]
            y0 = i[0][1]
            x1 = i[1][0]
            y1 = i[1][1]

            # Make sure that x0, y0 are smaller than x1, y1
            if x0 > x1:
                x0, x1 = x1, x0
            if y0 > y1:
                y0, y1 = y1, y0

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
                                minmax[j][0] = int(allv[j])
                            if allv[j] > minmax[j][1]:
                                minmax[j][1] = int(allv[j])
    
        # Now set sliders to those values
        names = [["low_red", "high_red"], ["low_green", "high_green"],\
                     ["low_blue", "high_blue"], ["low_hue", "high_hue"],\
                     ["low_sat", "high_sat"], ["low_val", "high_val"]]

        for i in xrange(len(names)):
            for j in xrange(2):
                D["thresholds"][names[i][j]] = minmax[i][j]
                cv.SetTrackbarPos(names[i][j], 'sliders', minmax[i][j])
            

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

        if D["mode"] == "start":
            D["sections"] = [ ['a', D["up_coord"], D["down_coord"]] ]
            process_section()
        elif D["mode"] == "add":
            D["sections"].append(['a', D["up_coord"], D["down_coord"]])
            process_section()
        elif D["mode"] == "subtract":
            # put lower coordinates first
            x0 = D["down_coord"][0]
            y0 = D["down_coord"][1]
            x1 = D["up_coord"][0]
            y1 = D["up_coord"][1]

            if x0 > x1:
                x0, x1 = x1, x0
            if y0 > y1:
                y0, y1 = y1, y0
            
            D["sections"].append(['s', (x0, y0), (x1, y1)])
            process_section()
        elif D["mode"] == "clear":
            D["sections"] = []

    elif D["mouse_down"] and event==cv.CV_EVENT_MOUSEMOVE: # mouse just moved
        D["up_coord"] = (x,y)


def check_key_press(key_press):
    """ this handler is called when a real key press has been
        detected, and updates everything appropriately
    """
    # get D so that we can change values in it
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

    elif key_press == ord('h'):
        print " Keyboard Command Menu"
        print " =============================="
        print " q    : quit"
        print " ESC  : quit"
        print " h    : help menu"
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

    elif key_press in key_dictionary.keys():
        D["current_threshold"] = key_dictionary[key_press]


# Function for changing the slider values
def change_slider(name, new_threshold):
    # get D so that we can change values in it
    global D
    D['thresholds'][name] = new_threshold


def handle_data(data):
    """ this method processes data given:
        - key presses
        - images from Kinect
    """
    # get D so that we can change values in it
    global D

    # Get the incoming image from the Kinect
    D["image"] = D["bridge"].imgmsg_to_cv(data, "bgr8")

    if D["created_images"] == False:
        # Initialize the additional images we need for processing
        # We only need to run this one time
        init_images()
        D["created_images"] = True

    # Recalculate threshold image
    threshold_image()

    # Recalculate blob in main image
    find_biggest_region()

    # Check on the display of dragged section
    mouse_section()

    # Get any incoming keypresses
    # To get input from keyboard, we use cv.WaitKey
    # Only the lowest eight bits matter (so we get rid of the rest):
    key_press_raw = cv.WaitKey(5) # gets a raw key press
    key_press = key_press_raw & 255 # sets all but the low 8 bits to 0
    
    # Handle key presses only if it's a real key (255 = "no key pressed")
    if key_press != 255:
        check_key_press(key_press)

    # Update the displays:
    # Main image:
    cv.ShowImage('image', D["image"])

    # Currently selected threshold image:
    cv.ShowImage('threshold', D[ D["current_threshold"] ] )

##################### END CALLBACK FUNCTIONS #######################

def main():
    """ the main program that sets everything up
    """
    
    # Initialize our node
    rospy.init_node('blobFinder')

    # Initialize all the global variables we will need
    init_globals()

    # Subscribe to the image_color topic
    # Each time new data comes in (Kinect color image, keypresses), we will call handle_data 
    # to deal with it
    rospy.Subscriber('/camera/rgb/image_color', sm.Image, handle_data)

    # Run until something stops us, such as a call to rospy.signal_shutdown
    rospy.spin()

# this is the "main" trick: it tells Python
# what code to run when you run this as a stand-alone script:
if __name__ == "__main__":
    main()
