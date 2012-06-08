#!/usr/bin/env python
import roslib; roslib.load_manifest('Frizzle')
import rospy
import cv_bridge
import cv
import sensor_msgs.msg as sm


class BlobFinder:

    def __init__(self):
        """ the constructor for the BlobFinder class -- this method contains
            the set-up needed for our color segmentation
        """
        
        #Dictionary containing the threshold values that the sliders correspond
        #to, each is initially set to 128, the mid value.
        self.thresholds = {'low_red': 0, 'high_red': 255,\
                           'low_green': 0, 'high_green': 255, \
                           'low_blue': 0, 'high_blue': 255,\
                           'low_hue':0, 'high_hue':255,\
                           'low_sat':0, 'high_sat':255,\
                           'low_val':0, 'high_val':255}        
        
        # Set up the windows containing the image from the kinect, the altered
        # image, and the threshold sliders.
        cv.NamedWindow('image')
        cv.MoveWindow('image',0,0)
        cv.NamedWindow('threshold')
        cv.MoveWindow('threshold',640,0)
        # making the slider window involves a bit more code, so
        # we factor it out into its own method
        self.make_slider_window()

        # sets the method to handle mouse button presses
        cv.SetMouseCallback('image', self.onMouse, None)

        # have we created the "scratch work" images yet?
        self.created_images = False

        # creates the connection to the Kinect
        self.bridge = cv_bridge.CvBridge()

        # variable for key presses
        self.last_key_pressed = 255


    def onMouse(self,event,x,y,flags,param):
        """ the method called when the mouse is clicked """
        
        if event==cv.CV_EVENT_LBUTTONDOWN: # clicked the left button
            print "x, y are", x, y
            (b,g,r) = self.image[y,x]
            print "r,g,b is", int(r), int(g), int(b)
            (h,s,v) = self.hsv[y,x]
            print "h,s,v is", h, s, v
            # should above be intified? Maybe make sure to have section on what rgb is
            # and what hsv is in relevant section as opposed to in intro

            
    
    def make_slider_window(self):
        """ a method to make a window full of sliders """
        
        #Create slider window
        cv.NamedWindow('sliders')
        cv.MoveWindow('sliders', 1280, 0)
        
        #Create sliders
        cv.CreateTrackbar('low_red', 'sliders', self.thresholds['low_red'],\
                          255, self.change_low_red)
        cv.CreateTrackbar('high_red', 'sliders', self.thresholds['high_red'],\
                           255, self.change_high_red)
        cv.CreateTrackbar('low_green', 'sliders', self.thresholds['low_green'],\
                          255, self.change_low_green)
        cv.CreateTrackbar('high_green', 'sliders',
                          self.thresholds['high_green'], 255,
                          self.change_high_green)
        cv.CreateTrackbar('low_blue', 'sliders', self.thresholds['low_blue'],\
                          255, self.change_low_blue)
        cv.CreateTrackbar('high_blue', 'sliders', self.thresholds['high_blue'],\
                          255, self.change_high_blue)
        cv.CreateTrackbar('low_hue', 'sliders', self.thresholds['low_hue'],\
                          255, self.change_low_hue)
        cv.CreateTrackbar('high_hue', 'sliders', self.thresholds['high_hue'],\
                          255, self.change_high_hue)
        cv.CreateTrackbar('low_sat', 'sliders', self.thresholds['low_sat'],\
                          255, self.change_low_sat)
        cv.CreateTrackbar('high_sat', 'sliders', self.thresholds['high_sat'],\
                          255, self.change_high_sat)
        cv.CreateTrackbar('low_val', 'sliders', self.thresholds['low_val'],\
                          255, self.change_low_val)
        cv.CreateTrackbar('high_val', 'sliders', self.thresholds['high_val'],\
                          255, self.change_high_val)
        
                          
    #Functions for changing the slider values  
    def change_low_red(self, new_threshold):
        self.thresholds['low_red'] = new_threshold
        
    def change_high_red(self, new_threshold):
        self.thresholds['high_red'] = new_threshold
        
    def change_low_green(self, new_threshold):
        self.thresholds['low_green'] = new_threshold
        
    def change_high_green(self, new_threshold):
        self.thresholds['high_green'] = new_threshold

    def change_low_blue(self, new_threshold):
        self.thresholds['low_blue'] = new_threshold

    def change_high_blue(self, new_threshold):
        self.thresholds['high_blue'] = new_threshold

    def change_low_hue(self, new_threshold):
        self.thresholds['low_hue'] = new_threshold

    def change_high_hue(self, new_threshold):
        self.thresholds['high_hue'] = new_threshold

    def change_low_sat(self, new_threshold):
        self.thresholds['low_sat'] = new_threshold

    def change_high_sat(self, new_threshold):
        self.thresholds['high_sat'] = new_threshold

    def change_low_val(self, new_threshold):
        self.thresholds['low_val'] = new_threshold

    def change_high_val(self, new_threshold):
        self.thresholds['high_val'] = new_threshold


    # key-press handler
    def check_key_press(self, key_press):
        """ this method handles user key presses appropriately """
        self.last_key_pressed = key_press
        
        if key_press == ord('q') or key_press == 27: # if a 'q' or Esc was pressed
            print 'quitting'
            rospy.signal_shutdown( "Quit requested from keyboard" )
        elif key_press == ord('h'):
            print "Keyboard Commands Menu"
            print "======================"
            print "q    : quit"
            print "ESC  : quit"
            print "h    : help menu"
            print "t    : show total threshold image in threshold window"
            print "r    : show red image in threshold window"
            print "g    : show green image in threshold window"
            print "b    : show blue image in threshold window"
            print "y    : show thresholded red image in threshold window"
            print "u    : show thresholded green image in threshold window"
            print "i    : show thresholded blue image in threshold window"
            print "a    : show hue image in threshold window"
            print "s    : show saturation image in threshold window"
            print "d    : show value image in threshold window"
            print "z    : show thresholded hue image in threshold window"
            print "x    : show thresholded saturation image in threshold window"
            print "c    : show thresholded value image in threshold window"
        elif key_press == ord('m'):
            print "Now I See!"
        
            
            
    # this is the method that gets called with each Kinect image      
    def handle_next_image(self, data):
        """ this method processes the next image from the Kinect """
        
        # Grab incoming image from the Kinect
        self.image = self.bridge.imgmsg_to_cv(data, "bgr8")

        if self.created_images == False:
            # creates the additional images we need for processing
            # we only need to run this one time
            self.create_images()
            self.created_images = True

        # To get input from the keyboard, we use cv.WaitKey
        # only the lowest eight bits matter (so we get rid of the rest):
        key_press_raw = cv.WaitKey(5) # gets a raw key press
        key_press = key_press_raw & 255 # sets all but the low 8 bits to 0
        # call a method to handle key presses, if it's a real key (255 = "no key pressed")
        if key_press != 255: self.check_key_press(key_press)

        #Create the thresholded image, self.threshed_image
        self.threshold_image()
        
        #Find "blob" regions, which should eventually draw
        # a rectangle around the largest blob with a circle at its center.
        self.find_biggest_region()
        
        #Display the images
        cv.ShowImage('image', self.image)

        #Display different images depending on last_key_pressed
        if self.last_key_pressed == 255 or self.last_key_pressed == ord('t'):
            cv.ShowImage('threshold', self.threshed_image)
        elif self.last_key_pressed == ord('r'):
            cv.ShowImage('threshold', self.red)
        elif self.last_key_pressed == ord('g'):
            cv.ShowImage('threshold', self.green)
        elif self.last_key_pressed == ord('b'):
            cv.ShowImage('threshold', self.blue)
        elif self.last_key_pressed == ord('y'):
            cv.ShowImage('threshold', self.red_threshed)
        elif self.last_key_pressed == ord('u'):
            cv.ShowImage('threshold', self.green_threshed)
        elif self.last_key_pressed == ord('i'):
            cv.ShowImage('threshold', self.blue_threshed)
        elif self.last_key_pressed == ord('a'):
            cv.ShowImage('threshold', self.hue)
        elif self.last_key_pressed == ord('s'):
            cv.ShowImage('threshold', self.sat)
        elif self.last_key_pressed == ord('d'):
            cv.ShowImage('threshold', self.val)
        elif self.last_key_pressed == ord('z'):
            cv.ShowImage('threshold', self.hue_threshed)
        elif self.last_key_pressed == ord('x'):
            cv.ShowImage('threshold', self.sat_threshed)
        elif self.last_key_pressed == ord('c'):
            cv.ShowImage('threshold', self.val_threshed)
        

        
    # this creates self.threshed_image from self.image
    # by thresholding to the correct color
    def threshold_image(self):
        """ this method runs the image processing in order to
            create a black and white image out of self.image
        """
        # Use OpenCV to split the image up into channels,
        # saving them in their respective bw images
        cv.Split(self.image, self.blue, self.green, self.red, None)

        # This line creates a hue-saturation-value image
        cv.CvtColor(self.image, self.hsv, cv.CV_RGB2HSV)
        cv.Split(self.hsv, self.hue, self.sat, self.val, None)
        
        #Here is how OpenCV thresholds the images based on the slider values:
        cv.InRangeS(self.red, self.thresholds['low_red'],\
                    self.thresholds['high_red'], self.red_threshed)
        cv.InRangeS(self.green, self.thresholds['low_green'],\
                    self.thresholds['high_green'], self.green_threshed)
        cv.InRangeS(self.blue, self.thresholds['low_blue'],\
                    self.thresholds['high_blue'], self.blue_threshed)
        cv.InRangeS(self.hue, self.thresholds['low_hue'],\
                    self.thresholds['high_hue'], self.hue_threshed)
        cv.InRangeS(self.sat, self.thresholds['low_sat'],\
                    self.thresholds['high_sat'], self.sat_threshed)
        cv.InRangeS(self.val, self.thresholds['low_val'],\
                    self.thresholds['high_val'], self.val_threshed)

        #Multiply all the thresholded images into one "output" image,
        # named self.threshed_image
        cv.Mul(self.red_threshed, self.green_threshed, self.threshed_image)
        cv.Mul(self.threshed_image, self.blue_threshed, self.threshed_image)
        cv.Mul(self.threshed_image, self.hue_threshed, self.threshed_image)
        cv.Mul(self.threshed_image, self.sat_threshed, self.threshed_image)
        cv.Mul(self.threshed_image, self.val_threshed, self.threshed_image)
        
        #Erode and Dilate shave off and add edge pixels respectively
        cv.Erode(self.threshed_image, self.threshed_image, iterations = 1)
        cv.Dilate(self.threshed_image, self.threshed_image, iterations = 1)

        # no need to return: all of these are data members of self's object

        
    # one-time method to create "scratch space" images
    def create_images(self):
        """ one-time creation of all of the images we'll need... """
        #Find the size of the image
        self.size = cv.GetSize(self.image)

        #Create images for each color channel
        self.blue = cv.CreateImage(self.size, 8, 1) 
        self.red = cv.CreateImage(self.size, 8, 1)
        self.green = cv.CreateImage(self.size, 8, 1)
        self.hue = cv.CreateImage(self.size, 8, 1)
        self.sat = cv.CreateImage(self.size, 8, 1)
        self.val = cv.CreateImage(self.size, 8, 1)

        #Create images to save the thresholded images to
        self.red_threshed = cv.CreateImage(self.size, 8, 1)
        self.green_threshed = cv.CreateImage(self.size, 8, 1)
        self.blue_threshed = cv.CreateImage(self.size, 8, 1)
        self.hue_threshed = cv.CreateImage(self.size, 8, 1)
        self.sat_threshed = cv.CreateImage(self.size, 8, 1)
        self.val_threshed = cv.CreateImage(self.size, 8, 1)
        self.threshed_image = cv.CreateImage(self.size, 8, 1) # final result

        #Create the hsv image
        self.hsv = cv.CreateImage(self.size, 8, 3)


        # no need to return: all of these are data members of self's object

    

    def find_biggest_region(self):
        """ finds all of the contours in self.threshed_image
            and then finds the largest one
        """
    
        #Create a copy image of thresholds then find contours on that image
        storage = cv.CreateMemStorage(0) # create memory storage for contours
        copy = cv.CreateImage(self.size, 8, 1)
        cv.Copy( self.threshed_image, copy )  # copy self.threshed_image

        # this is OpenCV's call to find all of the contours:
        contours = cv.FindContours(copy, storage, cv.CV_RETR_EXTERNAL,\
                                   cv.CV_CHAIN_APPROX_SIMPLE)
                                   
        # Next we want to find the *largest* contour
        if len(contours)>0:
            biggest = contours
            biggestArea=cv.ContourArea(contours)
            while contours != None:
                nextArea=cv.ContourArea(contours)
                if biggestArea < nextArea:
                    biggest = contours
                    biggestArea = nextArea
                contours=contours.h_next()
            
            #Use OpenCV to get a bounding rectangle for the largest contour
            br = cv.BoundingRect(biggest,update=0)
            #print br

            #print "in find_regions, br is", br

            # you will want to change these so that they draw
            # a box around the largest contour and a circle at
            # its center:

            #Example of drawing a red box
            #cv.PolyLine(self.image,[[(42,42),(42,100),
            #                         (100,100),(100,42)]],
            #                            1, cv.RGB(255, 0, 0))

            cv.PolyLine(self.image,[[(br[0],br[1]),(br[0],br[1] + br[3]),
                                     (br[0]+br[2],br[1]+br[3]),(br[0]+br[2],br[1])]],
                                        1, cv.RGB(255, 0, 0))           
            #Example of drawing a yellow circle
            cv.Circle(self.image,(br[0] + int(br[2]/2),br[1] + int(br[3]/2)), 10, cv.RGB(255, 255, 0),\
                      thickness=1, lineType=8, shift=0)
            
            #Draw the contours in white with inner ones in green
            cv.DrawContours(self.image, biggest, cv.RGB(255,255,255),
                            cv.RGB(0, 255, 0), 1, thickness=2, lineType=8,
                            offset=(0, 0))  

            
            
                            
                            
if __name__ == "__main__":
    """Main routine: sets up stuff BlobFinder needs to run, then runs it"""
    
    #Initialize our node
    rospy.init_node('blobFinder')
    
    #Create an object of class BlobFinder, defined above
    #Remember that this will call the constructor, __init__
    bf = BlobFinder()
    
    #Subscribe to the image_color topic
    #Each time a new Kinect color image comes in, bf.find_blob will be called:
    rospy.Subscriber('/camera/rgb/image_color', sm.Image, bf.handle_next_image)
    
    #Run until soemthing stops us, such as a call to rospy.signal_shutdown
    rospy.spin()
