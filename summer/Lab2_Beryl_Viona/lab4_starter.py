#!/usr/bin/env python
import roslib; roslib.load_manifest('Frizzle')
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
                       'low_green':0, 'high_green':255 }

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

    # Set the method to handle mouse button presses
    cv.SetMouseCallback('image', onMouse, None)

    # We have not created our "scratchwork" images yet
    D["created_images"] = False

    # Variable for key presses - set it to something that could not have been pressed
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
    D["green"] = cv.CreateImage(D["size"], 8, 1)
    D["blue"] = cv.CreateImage(D["size"], 8, 1)

    # Create images to save the thresholded images to
    D["red_threshed"] = cv.CreateImage(D["size"], 8, 1)
    D["green_threshed"] = cv.CreateImage(D["size"], 8, 1)

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
    #cv.Split(D["hsv"], D["hue"], D["sat"], D["val"], None)

    # Here is how OpenCV thresholds the images based on the slider values:
    cv.InRangeS(D["red"], D['thresholds']["low_red"], \
                    D['thresholds']["high_red"], D["red_threshed"])
    cv.InRangeS(D["green"], D['thresholds']["low_green"], \
                    D['thresholds']["high_green"], D["green_threshed"])

    # Multiply all the thresholded images into one "output" image,
    # named D["threshed_image"]
    cv.Mul(D["red_threshed"], D["green_threshed"], D["threshed_image"])

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

        #print "in find_regions, br is", br

        # You will want to change these so that they draw a box
        # around the largest contour and a circle at its center:

        # Example of drawing a red box
        cv.PolyLine(D["image"], [[(42,42), (42,100), (100,100), (100,42)]], \
                        1, cv.RGB(255, 0, 0))

        # Example of drawing a yellow circle
        cv.Circle(D["image"], (42,42), 10, \
                      cv.RGB(255, 255, 0), thickness=1, lineType=8, shift=0)

        # Draw the contours in white with inner ones in green
        cv.DrawContours(D["image"], biggest, cv.RGB(255, 255, 255), \
                            cv.RGB(0, 255, 0), 1, thickness=2, lineType=8, \
                            offset=(0,0))

################# END IMAGE PROCESSING FUNCTIONS ###################

####################### CALLBACK FUNCTIONS #########################
def onMouse(event, x, y, flags, param):
    """ the method called when the mouse is clicked """

    if event==cv.CV_EVENT_LBUTTONDOWN: # clicked the left button
        print "x, y are", x, y
        (b,g,r) = D["image"][y,x]
        print "r,g,b is", int(r), int(g), int(b)


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
                      ord('g'): "green" }

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
    elif key_press in key_dictionary.keys(): # If we said what the key_press maps to already
        D["current_threshold"] = key_dictionary[key_press]


def change_slider(name, new_threshold):
    """ changes the slider values given the name of the slider and the new value """
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
