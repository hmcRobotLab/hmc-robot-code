#!/usr/bin/env python
import roslib; roslib.load_manifest('Frizzle')
import rospy
import cv_bridge
import cv
import sensor_msgs.msg as sm

# Dictionary to hold all globals in
D = {}

####################### CALLBACK FUNCTIONS #########################
def onMouse(event, x, y, flags, param):
    """ the method called when the mouse is clicked """

    if event==cv.CV_EVENT_LBUTTONDOWN: # clicked the left button
        print "x, y are", x, y
        (b,g,r) = D["image"][y,x]
        print "r,g,b is", int(r), int(g), int(b)
        (h,s,v) = D["hsv"][y,x]
        print "h,s,v is", int(h), int(s), int(v)


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
                      ord('i'): "blue_threshed",\
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
        print " t    : show total threshold image in threshold window"
        print " r    : show red image in threshold window"
        print " g    : show green image in threshold window"
        print " b    : show blue image in threshold window"
        print " y    : show thresholded red image in threshold window"
        print " u    : show thresholded blue image in threshold window"
        print " i    : show thresholded green image in threshold window"
        print " a    : show hue image in threshold window"
        print " s    : show saturation image in threshold window"
        print " d    : show value image in threshold window"
        print " z    : show thresholded hue image in threshold window"
        print " x    : show thresholded saturation image in threshold window"
        print " c    : show thresholded value image in threshold window"
        print " p    : prints threshold values to file ( will overwrite old file )"
        print " l    : loads threshold values from file"
    elif key_press == ord('p'):
        x = D['thresholds']
        f = open( "./thresh.txt", "w" ) # open the file "thresh.txt" for writing
        print >> f, x # print x to the file object f
        f.close() # it's good to close the file afterwards
    elif key_press == ord('l'):
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

    elif key_press in key_dictionary.keys():
        D["current_threshold"] = key_dictionary[key_press]


# Function for changing the slider values
def change_slider(name, new_threshold):
    # get D so that we can change values in it
    global D
    D['thresholds'][name] = new_threshold


##################### END CALLBACK FUNCTIONS #######################

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

################# END IMAGE PROCESSING FUNCTIONS ###################

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
