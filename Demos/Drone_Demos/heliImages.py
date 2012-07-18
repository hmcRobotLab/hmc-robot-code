#Application Specific Imports
#from ardrone_mudd.srv import *  #Might not need this.
#from ardrone_mudd.msg import *  #Might not need this.
import roslib; roslib.load_manifest('ardrone_mudd')

USE_DRONE = True
IMAGE_SOURCE = "droneImage"

#Generic Imports
from std_msgs.msg import String
from sensor_msgs.msg import *
import rospy
import time,sys,random,cv,cv_bridge,math,os
import threading

class ImageProcessor:

  def __init__(self,use_drone = USE_DRONE):
    #Start the open CV window thread (may not be necessary)
    #cv.StartWindowThread()

    #Setting up the publisher to broadcast the data.
    self.publisher = rospy.Publisher('imageData', String)
    self.publisher2 = rospy.Publisher('imageData2', String)

    #This is for when you might want to use static image sources, but it's
    # not implemented yet. 
    self.use_drone = use_drone

   ########### The threshold values below ###########

    self.thresholds = {'low_red': 0, 'high_red': 256,\
                       'low_green': 0, 'high_green': 256, \
                       'low_blue': 0, 'high_blue':256, \
                       'low_hue': 0, 'high_hue': 256,\
                       'low_sat': 0, 'high_sat': 256, \
                       'low_val': 0, 'high_val': 256}

    self.thresholds2 = {'low_red': 0, 'high_red': 256,\
                       'low_green': 0, 'high_green': 256, \
                       'low_blue': 0, 'high_blue':256, \
                       'low_hue': 0, 'high_hue': 256,\
                       'low_sat': 0, 'high_sat': 256, \
                       'low_val': 0, 'high_val': 256}

  ########### The threshold values end ###########

    
    self.load_thresholds()
    self.load_thresholds2()

    cv.NamedWindow('image')
    cv.MoveWindow('image', 0, 0)
   ########### The threshold windows below ###########
    cv.NamedWindow('threshold') # the first threshold window
    cv.MoveWindow('threshold',0,400)
    cv.NamedWindow('threshold2') # the second threshold window
    cv.MoveWindow('threshold2',400,400)
   ########### The threshold windows end ###########
    
    self.make_control_window()
    self.bridge = cv_bridge.CvBridge()
    self.color_image = None             # the image from the drone
    self.new_image = False              # did we just receive a new image?
    self.threshed_image = None          # thresholded image
    self.font = cv.InitFont(cv.CV_FONT_HERSHEY_PLAIN, 1, 1, 0, 1)
    self.contours_found = False

  def load_thresholds(self):
    """ loads the thresholds from data.txt into self.thresholds """
    try:
      f = open("data.txt","r")
      data = f.read()
      f.close()
      self.thresholds = eval(data)
      print "thresholds loaded from data.txt"
      self.make_control_window()
    except:
      print "An error occurred in loading data.txt"
      print "Check if it's there and if it contains"
      print " a dictionary of thresholds..."


  def load_thresholds2(self):
    """ loads the thresholds from data.txt into self.thresholds """
    try:
      f = open("data2.txt","r")
      data = f.read()
      f.close()
      self.thresholds2 = eval(data)
      print "thresholds loaded from data2.txt"
      self.make_control_window2()
    except:
      print "An error occurred in loading data.txt"
      print "Check if it's there and if it contains"
      print " a dictionary of thresholds..."


      
  def make_control_window(self):
    """ a method to make a window full of sliders """
    ########### Sliders for both thresholds below ###########
    #Create slider window
    cv.NamedWindow('sliders')
    cv.MoveWindow('sliders', 742, 100)

    
    # Create sliders
    # Sliders for the main side camera below #
    cv.CreateTrackbar('low_red', 'sliders', self.thresholds['low_red'], 256, self.change_low_red)
    cv.CreateTrackbar('high_red', 'sliders', self.thresholds['high_red'], 256, self.change_high_red)
    cv.CreateTrackbar('low_green', 'sliders', self.thresholds['low_green'], 256, self.change_low_green)
    cv.CreateTrackbar('high_green', 'sliders', self.thresholds['high_green'], 256, self.change_high_green)
    cv.CreateTrackbar('low_blue', 'sliders', self.thresholds['low_blue'], 256, self.change_low_blue)
    cv.CreateTrackbar('high_blue', 'sliders', self.thresholds['high_blue'], 256, self.change_high_blue)
    cv.CreateTrackbar('low_hue', 'sliders', self.thresholds['low_hue'], 256, self.change_low_hue)
    cv.CreateTrackbar('high_hue', 'sliders', self.thresholds['high_hue'], 256, self.change_high_hue)
    cv.CreateTrackbar('low_sat', 'sliders', self.thresholds['low_sat'], 256, self.change_low_sat)
    cv.CreateTrackbar('high_sat', 'sliders', self.thresholds['high_sat'], 256, self.change_high_sat)
    cv.CreateTrackbar('low_val', 'sliders', self.thresholds['low_val'], 256, self.change_low_val)
    cv.CreateTrackbar('high_val', 'sliders', self.thresholds['high_val'], 256, self.change_high_val)
    # Sliders for the main side camera end #


  def make_control_window2(self):
    """ a method to make a window full of sliders """
    # Create sliders for the downward camera
    cv.NamedWindow('sliders2')
    cv.MoveWindow('sliders2', 742, 100)
    
    #Create sliders
    # Sliders for the downward camera below #
    cv.CreateTrackbar('low_red', 'sliders2', self.thresholds2['low_red'], 256, self.change_low_red2)
    cv.CreateTrackbar('high_red', 'sliders2', self.thresholds2['high_red'], 256, self.change_high_red2)
    cv.CreateTrackbar('low_green', 'sliders2', self.thresholds2['low_green'], 256, self.change_low_green2)
    cv.CreateTrackbar('high_green', 'sliders2', self.thresholds2['high_green'], 256, self.change_high_green2)
    cv.CreateTrackbar('low_blue', 'sliders2', self.thresholds2['low_blue'], 256, self.change_low_blue2)
    cv.CreateTrackbar('high_blue', 'sliders2', self.thresholds2['high_blue'], 256, self.change_high_blue2)
    cv.CreateTrackbar('low_hue', 'sliders2', self.thresholds2['low_hue'], 256, self.change_low_hue2)
    cv.CreateTrackbar('high_hue', 'sliders2', self.thresholds2['high_hue'], 256, self.change_high_hue2)
    cv.CreateTrackbar('low_sat', 'sliders2', self.thresholds2['low_sat'], 256, self.change_low_sat2)
    cv.CreateTrackbar('high_sat', 'sliders2', self.thresholds2['high_sat'], 256, self.change_high_sat2)
    cv.CreateTrackbar('low_val', 'sliders2', self.thresholds2['low_val'], 256, self.change_low_val2)
    cv.CreateTrackbar('high_val', 'sliders2', self.thresholds2['high_val'], 256, self.change_high_val2)


  #Functions for changing the slider values for the main camera
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

  #Functions for changing the slider values for the downward camera
  def change_low_red2(self, new_threshold):
    self.thresholds2['low_red'] = new_threshold
  def change_high_red2(self, new_threshold):
    self.thresholds2['high_red'] = new_threshold
  def change_low_green2(self, new_threshold):
    self.thresholds2['low_green'] = new_threshold
  def change_high_green2(self, new_threshold):
    self.thresholds2['high_green'] = new_threshold
  def change_low_blue2(self, new_threshold):
    self.thresholds2['low_blue'] = new_threshold
  def change_high_blue2(self, new_threshold):
    self.thresholds2['high_blue'] = new_threshold
  def change_low_hue2(self, new_threshold):
    self.thresholds2['low_hue'] = new_threshold
  def change_high_hue2(self, new_threshold):
    self.thresholds2['high_hue'] = new_threshold
  def change_low_sat2(self, new_threshold):
    self.thresholds2['low_sat'] = new_threshold
  def change_high_sat2(self, new_threshold):
    self.thresholds2['high_sat'] = new_threshold
  def change_low_val2(self, new_threshold):
    self.thresholds2['low_val'] = new_threshold
  def change_high_val2(self, new_threshold):
    self.thresholds2['high_val'] = new_threshold



  def create_all_images(self):
    """ a one-time method that creates all of the images needed
        for the image processing...
        
        This should be called only if self.color_image is NOT None
        but self.threshed_image IS None
    """
    #Find the size of the images
    self.size = cv.GetSize(self.color_image)

    self.red = cv.CreateImage(self.size, 8, 1)     # color components
    self.green = cv.CreateImage(self.size, 8, 1)
    self.blue = cv.CreateImage(self.size, 8, 1)

    self.hsv = cv.CreateImage(self.size, 8, 3)     # HSV image
    self.hue = cv.CreateImage(self.size, 8, 1)     # and its components
    self.sat = cv.CreateImage(self.size, 8, 1)
    self.val = cv.CreateImage(self.size, 8, 1)

    
    # For the downward camera
    self.red2 = cv.CreateImage(self.size, 8, 1)     # color components
    self.green2 = cv.CreateImage(self.size, 8, 1)
    self.blue2 = cv.CreateImage(self.size, 8, 1)

    self.hsv2 = cv.CreateImage(self.size, 8, 3)     # HSV image
    self.hue2 = cv.CreateImage(self.size, 8, 1)     # and its components
    self.sat2 = cv.CreateImage(self.size, 8, 1)
    self.val2 = cv.CreateImage(self.size, 8, 1)



    self.threshed_image = cv.CreateImage(self.size, 8, 1)  # output image
    self.threshed_image2 = cv.CreateImage(self.size, 8, 1)  # output image
    self.copy = cv.CreateImage(self.size, 8, 1)
    self.copy2 = cv.CreateImage(self.size, 8, 1)

    self.storage = cv.CreateMemStorage(0) # create memory storage for contours
    self.storage2 = cv.CreateMemStorage(0) # create memory storage for contours 2

  def save_thresholds(self):
    """ saves the current thresholds to data.txt """
    f = open("data.txt","w")
    print >> f, self.thresholds
    f.close()
    print "thresholds saved to data.txt"

    
    # Saves the thresholds for the downward camera
  def save_thresholds2(self):
    """ saves the current thresholds to data2.txt """
    f = open("data2.txt","w")
    print >> f, self.thresholds2
    f.close()
    print "thresholds saved to data2.txt"

  def videoUpdate(self, data):
    """Displays the image, calls find_info"""
    # get the image from the Kinect, if self.use_drone == True
    # kinect images: self.image = self.bridge.imgmsg_to_cv(data, "32FC1")
    # drone images:
    if self.use_drone:
      self.color_image = self.bridge.imgmsg_to_cv(data, "bgr8")
      self.new_image = True
    # otherwise, get an image from file every so often
    else:
      cur_time = time.time()
      if cur_time - self.last_image_time > 0.25: # number of seconds per update
        self.last_image_time = cur_time # reset the last image time
        folder = self.folder_names[self.location]
        fn = folder + "/" + str(self.image_hour) + ".png"
        #print "filename", fn
        self.color_image=cv.LoadImageM(fn)
        self.new_image = True
      # otherwise, we don't get a new image
      
      # in case we want randomness:
      #image_hour_number = random.randint(0,23)
      #self.image_hour = random.randint(3,3)

  # do all of the image processing
  def process_Image(self):
    """ here is where the image should be processed to get the bounding box """
    # check if we've created the supporting images yet
    if self.threshed_image == None:
      if self.color_image != None:
        self.create_all_images()

    # from the old method call def threshold_image(self):
    cv.Split(self.color_image, self.blue, self.green, self.red, None)
    cv.Split(self.color_image, self.blue2, self.green2, self.red2, None)
    cv.CvtColor(self.color_image, self.hsv, cv.CV_RGB2HSV)
    cv.Split(self.hsv, self.hue, self.sat, self.val, None)
    cv.Split(self.hsv, self.hue2, self.sat2, self.val2, None)

    # replace each channel with its thresholded version
    cv.InRangeS(self.red, self.thresholds['low_red'],\
                self.thresholds['high_red'], self.red)
    cv.InRangeS(self.green, self.thresholds['low_green'],\
                self.thresholds['high_green'], self.green)
    cv.InRangeS(self.blue, self.thresholds['low_blue'],\
                self.thresholds['high_blue'], self.blue)
    cv.InRangeS(self.hue, self.thresholds['low_hue'],\
                self.thresholds['high_hue'], self.hue)
    cv.InRangeS(self.sat, self.thresholds['low_sat'],\
                self.thresholds['high_sat'], self.sat)
    cv.InRangeS(self.val, self.thresholds['low_val'],\
                self.thresholds['high_val'], self.val)

    # In range for the second thresholds
    cv.InRangeS(self.red2, self.thresholds2['low_red'],\
                self.thresholds2['high_red'], self.red2)
    cv.InRangeS(self.green2, self.thresholds2['low_green'],\
                self.thresholds2['high_green'], self.green2)
    cv.InRangeS(self.blue2, self.thresholds2['low_blue'],\
                self.thresholds2['high_blue'], self.blue2)
    cv.InRangeS(self.hue2, self.thresholds2['low_hue'],\
                self.thresholds2['high_hue'], self.hue2)
    cv.InRangeS(self.sat2, self.thresholds2['low_sat'],\
                self.thresholds2['high_sat'], self.sat2)
    cv.InRangeS(self.val2, self.thresholds2['low_val'],\
                self.thresholds2['high_val'], self.val2)

    # AND (multiply) all the thresholded images into one "output" image,
    # named self.copy
    cv.Mul(self.red, self.green, self.copy)
    cv.Mul(self.copy, self.blue, self.copy)
    cv.Mul(self.copy, self.hue, self.copy)
    cv.Mul(self.copy, self.sat, self.copy)
    cv.Mul(self.copy, self.val, self.copy)


    #Repeat but with Downward Camera

    cv.Mul(self.red2, self.green2, self.copy2)
    cv.Mul(self.copy2, self.blue2, self.copy2)
    cv.Mul(self.copy2, self.hue2, self.copy2)
    cv.Mul(self.copy2, self.sat2, self.copy2)
    cv.Mul(self.copy2, self.val2, self.copy2)
    
    # erode and dilate shave off and add edge pixels respectively
    cv.Erode(self.copy, self.copy, iterations = 1)
    cv.Dilate(self.copy, self.copy, iterations = 1)
    cv.Copy(self.copy,self.threshed_image)

    cv.Erode(self.copy2, self.copy2, iterations = 1)
    cv.Dilate(self.copy2, self.copy2, iterations = 1)
    cv.Copy(self.copy2,self.threshed_image2)

    self.find_biggest_region()
    self.find_biggest_region2()
    

  def find_biggest_region(self):
    """ this code should find the biggest region and
        then determine some of its characteristics, which
        will help direct the drone
    """
    # copy the thresholded image
    cv.Copy( self.threshed_image, self.copy )  # copy self.threshed_image
    # this is OpenCV's call to find all of the contours:
    contours = cv.FindContours(self.copy, self.storage, cv.CV_RETR_EXTERNAL,
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

      #Extract the characteristics of the bounding box.
      xl=br[0]
      xr=xl + br[2]
      yt=br[1]
      yb=yt + br[3]
      xcenter = (xl + xr)/2
      ycenter = (yt+yb)/2

      #Draw a contour around the bounding box.
      cv.PolyLine(self.color_image,[[(xl,yt),(xl,yb),(xr,yb),(xr,yt)]],10, cv.RGB(0, 0, 255))

      #Publish the bounding box.
      self.publisher.publish("%i %i %i %i" % (xl,xr,yt,yb))
      #self.publisher.publish("%i %i" % (xcenter,ycenter))

    else:
      xl = float("inf")
      xr = float("inf")
      yt = float("inf")
      yb = float("inf")
      #xcenter = float((xl + xr)/2)
      #ycenter =  float((yt+yb)/2)
      self.publisher.publish("%f %f %f %f" % (xl,xr,yt,yb))
      #self.publisher.publish("%f %f" % (xcenter,ycenter))


  def find_biggest_region2(self):
    """ this code should find the biggest region of the downward camera and
        then determine some of its characteristics, which
        will help direct the drone
    """
    # copy the thresholded image

    cv.Copy( self.threshed_image2, self.copy2 )  # copy self.threshed_image
    # this is OpenCV's call to find all of the contours:
    contours = cv.FindContours(self.copy2, self.storage2, cv.CV_RETR_EXTERNAL,
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

      #Extract the characteristics of the bounding box.
      xl=br[0]
      xr=xl + br[2]
      yt=br[1]
      yb=yt + br[3]

      #Draw a contour around the bounding box.
      cv.PolyLine(self.color_image,[[(xl,yt),(xl,yb),(xr,yb),(xr,yt)]],10, cv.RGB(255, 0, 255))

      self.publisher2.publish("%i %i %i %i" % (xl,xr,yt,yb))
      #Publish the bounding box.
      #self.publisher2.publish("%i %i" % (xcenter,ycenter))
    else:
      xl = float("inf")
      xr = float("inf")
      yt = float("inf")
      yb = float("inf")
      #xcenter = float((xl + xr)/2)
      #ycenter =  float((yt+yb)/2)
      self.publisher2.publish("%f %f %f %f" % (xl,xr,yt,yb))
      #self.publisher2.publish("%f %f" % (xcenter,ycenter))


  # the keyboard thread is the "main" thread for this program
  def keyboardLoop(self):
    """ the main keypress-handling thread to control the drone """

    # this is the main loop for the keyboard thread
    #
    # don't use 'Q', 'R', 'S', or 'T'  (they're the arrow keys)
    #
    count = 0
    while True:
      count += 1
      # handle the image processing if we have a new Kinect image
      if (self.new_image):# and (count % 50 == 0)):
        self.new_image = False # until we get a new one...
        new_image_size = cv.GetSize(self.color_image)
        self.process_Image()
        cv.ShowImage('image', self.color_image)
        cv.ShowImage('threshold', self.threshed_image)
        cv.ShowImage('threshold2', self.threshed_image2)

      # get the next keypress
      c = cv.WaitKey(25)&255

      # handle the keypress to quit...
      if c == ord('q') or c == 27: # the Esc key is 27
        rospy.sleep(1.42) # wait a bit for everything to finish...
        # it's important that the FSM not re-schedule itself further than
        # 1 second in the future, so that a thread won't be left running
        # after this Python program quits right here:
        return

      # saving and loading the thresholds from data.txt
      if c == ord('!'):  # the ! point saves
        self.save_thresholds()

      if c == ord('@'):  # atloads thresholds from data2.txt
        self.load_thresholds()

      if c == ord('#'):  # the # point saves
        self.save_thresholds2()

      if c == ord('$'):  # $ loads thresholds from data2.txt
        self.load_thresholds2()

      if c == ord('m'):
        print self.size

def main():
  rospy.init_node("image_processing")
  iP = ImageProcessor()

  #Subscribing to the video source
  print "\r Connecting to video service"
  rospy.Subscriber(IMAGE_SOURCE, Image, iP.videoUpdate, queue_size = 1)
  print "\r Connected\n"
  
  rospy.sleep(1)

  #the main loop
  iP.keyboardLoop()

  print "Exiting Program"

if __name__ == "__main__":
  main()
