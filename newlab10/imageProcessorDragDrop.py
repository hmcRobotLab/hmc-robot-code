#Generic Imports
import roslib; roslib.load_manifest('ardrone_mudd')
from std_msgs.msg import String
from sensor_msgs.msg import *
import rospy
import time,sys,random,cv,cv_bridge,math,os
import threading

#Application Specific Imports
# You'll likely use "droneImage" for the drone.
IMAGE_SOURCE = "/ardrone2/camera/image"

class ImageProcessor:

  def __init__(self):
    #Setting up the publisher to broadcast the data.
    self.publisher = rospy.Publisher('imageData', String)

    self.thresholds = {'low_red': 0, 'high_red': 256,\
                       'low_green': 0, 'high_green': 256, \
                       'low_blue': 0, 'high_blue':256, \
                       'low_hue': 0, 'high_hue': 256,\
                       'low_sat': 0, 'high_sat': 256, \
                       'low_val': 0, 'high_val': 256}
    self.load_thresholds()

    cv.NamedWindow('image')
    cv.MoveWindow('image', 0, 0)
    cv.SetMouseCallback('image', self.onMouse2, None)
    cv.NamedWindow('threshold')
    cv.MoveWindow('threshold',0,480)
    cv.SetMouseCallback('threshold', self.onMouse2, None)
    self.make_control_window()
    self.bridge = cv_bridge.CvBridge()
    self.image = None             # the image from the drone
    self.new_image = False              # did we just receive a new image?
    self.threshed_image = None          # thresholded image
    self.contours_found = False
    
    #If you want to use text on the image, uncomment the line below.
    # it sets the font to something readable. 
    #self.font = cv.InitFont(cv.CV_FONT_HERSHEY_PLAIN, 1, 1, 0, 1)
    
    # Drag And Drop data members.
    self.mouse_down = False
    self.down_coord = (-1, -1)
    self.up_coord = (-1, -1)
    self.sections = []
    self.mode = "add"

  def load_thresholds(self):
    """ loads the thresholds from data.txt into self.thresholds """
    try:
      f = open("data.txt","r")
      data = f.read()
      f.close()
      self.thresholds = eval(data)
      print "Thresholds loaded from data.txt"
      self.make_control_window()
    except:
      print "An error occurred in loading data.txt"
      print "Check if it's there and if it contains"
      print " a dictionary of thresholds..."

  def make_control_window(self):
    """ a method to make a window full of sliders """
    
    #Create slider window
    cv.NamedWindow('sliders')
    cv.MoveWindow('sliders', 742, 100)
    
    #Create sliders
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

  def create_all_images(self):
    """ a one-time method that creates all of the images needed
        for the image processing...
        
        This should be called only if self.image is NOT None
        but self.threshed_image IS None
    """
    #Find the size of the images
    self.size = cv.GetSize(self.image)

    self.red = cv.CreateImage(self.size, 8, 1)     # color components
    self.green = cv.CreateImage(self.size, 8, 1)
    self.blue = cv.CreateImage(self.size, 8, 1)

    self.hsv = cv.CreateImage(self.size, 8, 3)     # HSV image
    self.hue = cv.CreateImage(self.size, 8, 1)     # and its components
    self.sat = cv.CreateImage(self.size, 8, 1)
    self.val = cv.CreateImage(self.size, 8, 1)

    self.threshed_image = cv.CreateImage(self.size, 8, 1)  # output image
    self.copy = cv.CreateImage(self.size, 8, 1)

    self.storage = cv.CreateMemStorage(0) # create memory storage for contours

  def save_thresholds(self):
    """ saves the current thresholds to data.txt """
    f = open("data.txt","w")
    print >> f, self.thresholds
    f.close()
    print "thresholds saved to data.txt"

  def onMouse(self,event,x,y,flags,param):
    """ the method called when the mouse is clicked """
    width = 40
    if flags == cv.CV_EVENT_FLAG_CTRLKEY:
      width -= 20
    elif flags == cv.CV_EVENT_FLAG_SHIFTKEY:
      width += 20
    # if the left button was clicked
    if event == cv.CV_EVENT_RBUTTONDOWN:
      bgrTuple = tuple(map(lambda(v) :(max(int(v)-width,0),min(int(v)+width,256)),self.image[y,x]))
      hsvTuple = tuple(map(lambda(v) :(max(int(v)-width,0),min(int(v)+width,256)),self.hsv[y,x]))
      print "Setting Filters to:"
      print "b: %s, g: %s,  r: %s" % bgrTuple
      print "h: %s, s: %s, v: %s" % hsvTuple
      self.change_low_blue(bgrTuple[0][0])
      self.change_high_blue(bgrTuple[0][1])
      self.change_low_green(bgrTuple[1][0])
      self.change_high_green(bgrTuple[1][1])
      self.change_low_red(bgrTuple[2][0])
      self.change_high_red(bgrTuple[2][1])
      self.change_low_hue(hsvTuple[0][0])
      self.change_high_hue(hsvTuple[0][1])
      self.change_low_sat(hsvTuple[1][0])
      self.change_high_sat(hsvTuple[1][1])
      self.change_low_val(hsvTuple[2][0])
      self.change_high_val(hsvTuple[2][1])
      self.make_control_window()
      print "Type '$' to save."
    elif event == cv.CV_EVENT_LBUTTONDOWN:
      print "r: %s, g: %s, b: %s" % self.image[y,x]
      print "h: %s, s: %s, v: %s" % self.hsv[y,x]
      print "Right click to set filters"

  def process_section(self):
    """ calculates the min/max slider values for a given section and sets the
        slider values to them
    """
    print "sections:", self.sections

    if len(self.sections) > 0:
      # pull out all sections with 'a' in the front
      adds = [[x[1], x[2]] for x in self.sections if x[0] == 'a']

      # pull out all sections with a 's' in the front
      subs = [[x[1], x[2]] for x in self.sections if x[0] == 's']

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
              (b,g,r) = self.image[y,x]
              (h,s,v) = self.hsv[y,x]
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
        self.thresholds[names[i][j]] = minmax[i][j]
        cv.SetTrackbarPos(names[i][j], 'sliders', minmax[i][j])
            
  def onMouse2(self, event, x, y, flags, param):
    """ the method called when the mouse is clicked """
    if event==cv.CV_EVENT_LBUTTONDOWN: # clicked the left button
      print "x, y are", x, y
      (b,g,r) = self.image[y,x]
      print "r,g,b is", int(r), int(g), int(b)
      (h,s,v) = self.hsv[y,x]
      print "h,s,v is", int(h), int(s), int(v)
      self.down_coord = (x,y)
      self.mouse_down = True
    elif event==cv.CV_EVENT_LBUTTONUP: # let go of the left button
      print "x, y are", x, y
      (b,g,r) = self.image[y,x]
      print "r,g,b is", int(r), int(g), int(b)
      (h,s,v) = self.hsv[y,x]
      print "h,s,v is", int(h), int(s), int(v)
      self.up_coord = (x,y)
      self.mouse_down = False

      if self.mode == "start":
        self.sections = [ ['a', self.up_coord, self.down_coord] ]
        self.process_section()
      elif self.mode == "add":
        self.sections.append(['a', self.up_coord, self.down_coord])
        self.process_section()
      elif self.mode == "subtract":
        # put lower coordinates first
        x0 = self.down_coord[0]
        y0 = self.down_coord[1]
        x1 = self.up_coord[0]
        y1 = self.up_coord[1]

        if x0 > x1:
            x0, x1 = x1, x0
        if y0 > y1:
            y0, y1 = y1, y0
        
        self.sections.append(['s', (x0, y0), (x1, y1)])
        self.process_section()
      elif self.mode == "clear":
        self.sections = []

    elif self.mouse_down and event==cv.CV_EVENT_MOUSEMOVE: # mouse just moved
      self.up_coord = (x,y)
  
  
  def videoUpdate(self, data):
    """Displays the image, calls find_info"""
    # kinect images: self.image = self.bridge.imgmsg_to_cv(data, "32FC1")
    # drone images: self.image = self.bridge.imgmsg_to_cv(data, "bgr8")
    self.image = self.bridge.imgmsg_to_cv(data, "bgr8")

    # Here we set a boolean so that the opencv calls don't happen in the callback thread.
    # We read and react to the boolean in the main thread, in keyboardLoop
    self.new_image = True

  # do all of the image processing
  def process_Image(self):
    """ here is where the image should be processed to get the bounding box """
    # check if we've created the supporting images yet
    if self.threshed_image == None:
      if self.image != None:
        self.create_all_images()

    # from the old method call def threshold_image(self):
    cv.Split(self.image, self.blue, self.green, self.red, None)
    cv.CvtColor(self.image, self.hsv, cv.CV_RGB2HSV)
    cv.Split(self.hsv, self.hue, self.sat, self.val, None)

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

    # AND (multiply) all the thresholded images into one "output" image,
    # named self.copy
    cv.Mul(self.red, self.green, self.copy)
    cv.Mul(self.copy, self.blue, self.copy)
    cv.Mul(self.copy, self.hue, self.copy)
    cv.Mul(self.copy, self.sat, self.copy)
    cv.Mul(self.copy, self.val, self.copy)
    # erode and dilate shave off and add edge pixels respectively
    cv.Erode(self.copy, self.copy, iterations = 1)
    cv.Dilate(self.copy, self.copy, iterations = 1)

    # Make self.threshed_image be self.copy
    cv.Copy(self.copy,self.threshed_image)

    self.find_biggest_region()

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
      self.br = cv.BoundingRect(biggest,update=0)
      
      #Publish the data.
      self.publishBoxData()

  def publishBoxData(self):
    #Extract the characteristics of the bounding box.
    xl = self.br[0]
    xr = xl + self.br[2]
    yt = self.br[1]
    yb = yt + self.br[3]

    #Form the data to send
    left    = float(xl)/self.size[0]
    top     = float(yt)/self.size[1]
    right   = float(xr)/self.size[0]
    bottom  = float(yb)/self.size[1]
    centerX = left + (right - left)/2
    centerY = bottom + (top - bottom)/2
    area    = 1.0*(xr-xl)*(yb-yt)/(self.size[0]*self.size[1])
    
    #Draw a contour around the bounding box.
    cv.PolyLine(self.image,[[(xl,yt),(xl,yb),(xr,yb),(xr,yt)]],10, cv.RGB(0, 0, 255))

    #Publish the bounding box.
    #Format is: "CenterX (in percent of box width) CenterY (in percent of box height)
    #            Area (in percent of box area) LeftEdge (in percent of box width)
    #            TopEdge (in percent of box height) RightEdge (in percent of box width)
    #            BottomEdge (in percent of box height)"

    self.publisher.publish("%f %f %f %f %f %f %f" % (centerX, centerY, area, left, top, right, bottom))

  # the keyboard thread is the "main" thread for this program
  def keyboardLoop(self):
    """ the main keypress-handling thread to control the drone """

    while True:
      # handle the image processing if we have a new Kinect image
      if (self.new_image):
        self.new_image = False # until we get a new one...
        #Do the image processing
        self.process_Image()
        
        #And show the images.
        cv.ShowImage('image', self.image)
        cv.ShowImage('threshold', self.threshed_image)

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
      if c == ord('$'):  # dollar sign to save (looks like an S)
        self.save_thresholds()

      if c == ord('&'):  # ampersand loads thresholds from data.txt
        self.load_thresholds()

def main():
  # Initializing the node
  rospy.init_node("image_processing")
  
  # Creating the image processor
  iP = ImageProcessor()

  # Subscribing to the video source
  print "Connecting to video service"
  rospy.Subscriber(IMAGE_SOURCE, Image, iP.videoUpdate, queue_size = 1)
  print "Connected"
  
  rospy.sleep(1)

  # The main loop
  iP.keyboardLoop()

  print "Exiting Program"

if __name__ == "__main__":
  main()
