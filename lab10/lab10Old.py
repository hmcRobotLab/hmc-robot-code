#!/usr/bin/env python
import roslib; roslib.load_manifest('Daneel')
import rospy

import sensor_msgs.msg as sm
import cv_bridge
import cv
import time
import threading
import math

import pydrone.srv
import pydrone.ARDrone as ARDrone
import pydrone.msg as msg

import os
import random

#
# OpenCV Python documentation:
#    http://opencv.willowgarage.com/documentation/python/


class DroneController:
    """
    The drone is keyboard-controlled for movement. Clicking in the image
    window will give the RGB values at that point.
    """

    def __init__(self, use_drone = False):
        """ constructor; setting up data """

        # member variables relating to the drone's control
        self.use_drone = use_drone  # are we using the drone?
        self.init_drone_parameters()
        self.airborne = False         # not airborne yet
        self.run_without_data_stream = False
        self.battery_level = "Not read from drone"
        self.boxHeight = 0
        self.boxX = 0
        self.tarHeight = 35
        self.tarX = 150

        # variables for the off-board images
        self.location = ('c',1) # the starting location (always a tuple)
        #self.location = ( random.choice(['c','n','s']), random.randint(0,6) )
        self.image_hour = 3 # heading at 3 o'clock == toward the markers
        #self.angle_deg = random.randint(180,360) # the angle it's facing
        self.last_image_time = time.time() # last time an image was grabbed
        self.folder_names = {('c',0):"./drone_images/c_0_Ginny",
                             ('c',4):"./drone_images/c_4_Bill",
                             ('n',3):"./drone_images/n_3_Albus",
                             ('s',0):"./drone_images/s_0_Draco",
                             ('s',4):"./drone_images/s_4_Ron",
                             ('c',1):"./drone_images/c_1_Fred",
                             ('c',5):"./drone_images/c_5_Molly",
                             ('n',0):"./drone_images/n_0_Neville",
                             ('n',4):"./drone_images/n_4_Rubeus",
                             ('s',1):"./drone_images/s_1_Sirius",
                             ('s',5):"./drone_images/s_5_Hermione",
                             ('c',2):"./drone_images/c_2_George",
                             ('c',6):"./drone_images/c_6_Arthur",
                             ('n',1):"./drone_images/n_1_Luna",
                             ('n',5):"./drone_images/n_5_Severus",
                             ('s',2):"./drone_images/s_2_Remus",
                             ('s',6):"./drone_images/s_6_Harry",
                             ('c',3):"./drone_images/c_3_Charlie",
                             ('n',2):"./drone_images/n_2_Minerva",
                             ('n',6):"./drone_images/n_6_Dobby",
                             ('s',3):"./drone_images/s_3_Fawkes"}

        # thresholds for image processing
        self.thresholds = {'low_red': 0, 'high_red': 256,\
                           'low_green': 0, 'high_green': 256, \
                           'low_blue': 0, 'high_blue':256, \
                           'low_hue': 0, 'high_hue': 256,\
                           'low_sat': 0, 'high_sat': 256, \
                           'low_val': 0, 'high_val': 256}
        
        # windows for image processing
        cv.NamedWindow('image')
        cv.MoveWindow('image', 100, 0)
        cv.SetMouseCallback('image', self.onMouse, None)
        cv.NamedWindow('threshold')
        cv.MoveWindow('threshold',100,400)
        cv.SetMouseCallback('threshold', self.onMouse, None)
        self.make_slider_window()

        # images and other data for the images/windows
        self.bridge = cv_bridge.CvBridge()  # the interface to OpenCV
        self.color_image = None             # the image from the drone
        self.new_image = False              # did we just receive a new image?
        self.threshed_image = None          # thresholded image
        self.font = cv.InitFont(cv.CV_FONT_HERSHEY_PLAIN, 1, 1, 0, 1)
        self.contours_found = False
        self.saw_capital_c = False

        # variables for state machine
        self.state = "Keyboard"
        self.lastHover = 0
        
        self.load_thresholds()

    def init_drone_parameters(self):
        """ drone-specific parameters: speed, camera, etc. """
        print "Constructing the drone software object..."
        if self.use_drone == True:
            self.drone = ARDrone.ARDrone()
            # Drone's motion speed
            self.drone.speed = 0.15
            self.drone.frequency = 8 # Drone's ultrasound frequency - 7 for 22.22 Hz, 8 for 25 Hz
            self.drone.altmax_value = 10000 # Drone's maximum altitude between 500
                                       # and 5000 or 10000 for unlimited

            # print "Trying to connect to the drone"
            self.drone.connect()
            self.drone.freq(self.drone.frequency)
            self.drone.altmax(self.drone.altmax_value)

            # set the video stream
            self.drone.cam = 0 # 0 for forward, 1 for floor camera    def find_biggest_region(self):
            self.drone.feed(self.drone.cam)
            rospy.sleep(1)
            print "Drone initialized."

        else:
            print "Not using drone."



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
        
        self.copy = cv.CreateImage(self.size, 8, 1)    # extra image
        self.threshed_image = cv.CreateImage(self.size, 8, 1)  # output image

        self.storage = cv.CreateMemStorage(0) # create memory storage for contours




    def save_thresholds(self):
        """ saves the current thresholds to data.txt """
        f = open("data.txt","w")
        print >> f, self.thresholds
        f.close()
        print "thresholds saved to data.txt"



    def load_thresholds(self):
        """ loads the thresholds from data.txt into self.thresholds """
        try:
            f = open("data.txt","r")
            data = f.read()
            f.close()
            self.thresholds = eval(data)
            print "thresholds loaded from data.txt"
            self.make_slider_window()
        except:
            print "An error occurred in loading data.txt"
            print "Check if it's there and if it contains"
            print "  a dictionary of thresholds..."




    def change_location(self,direction):
        """ changes the location from which an image is taken in one of six ways """
        # get the state
        current_ns_character = self.location[0]
        current_ew_number = self.location[1]
        current_image_hour = self.image_hour

        # make appropriate changes
        if direction == 'west':
            current_ew_number += 1       
            if current_ew_number > 6: current_ew_number = 6
        elif direction == 'east':
            current_ew_number -= 1
            if current_ew_number < 0: current_ew_number = 0
        elif direction == 'north':
            if current_ns_character == 'n': pass
            elif current_ns_character == 's': current_ns_character = 'c'
            elif current_ns_character == 'c': current_ns_character = 'n'
        elif direction == 'south':
            if current_ns_character == 'n': current_ns_character = 'c'
            elif current_ns_character == 's': pass
            elif current_ns_character == 'c': current_ns_character = 's'
        elif direction == 'counterclockwise':
            current_image_hour -= 1
            if current_image_hour < 0: current_image_hour = 23
        elif direction == 'clockwise':
            current_image_hour += 1
            if current_image_hour > 23: current_image_hour = 0
        else:
            print "a direction of", direction,
            print "was not recognized in change_location"
            
        # reset the state
        self.location = (current_ns_character,current_ew_number)
        self.image_hour = current_image_hour
        print "location and image_hour are now", self.location, self.image_hour



    def make_slider_window(self):
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
    def change_low_red(self, new_threshold):       self.thresholds['low_red'] = new_threshold
    def change_high_red(self, new_threshold):       self.thresholds['high_red'] = new_threshold
    def change_low_green(self, new_threshold):       self.thresholds['low_green'] = new_threshold
    def change_high_green(self, new_threshold):       self.thresholds['high_green'] = new_threshold
    def change_low_blue(self, new_threshold):       self.thresholds['low_blue'] = new_threshold
    def change_high_blue(self, new_threshold):       self.thresholds['high_blue'] = new_threshold
    def change_low_hue(self, new_threshold):       self.thresholds['low_hue'] = new_threshold
    def change_high_hue(self, new_threshold):       self.thresholds['high_hue'] = new_threshold
    def change_low_sat(self, new_threshold):       self.thresholds['low_sat'] = new_threshold
    def change_high_sat(self, new_threshold):       self.thresholds['high_sat'] = new_threshold
    def change_low_val(self, new_threshold):       self.thresholds['low_val'] = new_threshold
    def change_high_val(self, new_threshold):       self.thresholds['high_val'] = new_threshold



    def handle_next_image(self, data):
        """Displays the image, calls find_info"""        
        # get the image from the Kinect, if self.use_drone == True
        # kinect images: self.image = self.bridge.imgmsg_to_cv(data, "32FC1")
        # drone images:
        if self.use_drone == True:
            self.color_image = self.bridge.imgmsg_to_cv(data, "bgr8")
            # tell the keyboard thread that we have a new image
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


    # handles the drone's data stream (separate from the video and control)
    def getData(self, data):
        """ the drone's navigation and status data """
        if self.use_drone == True and self.run_without_data_stream == False:
            try:
                data_dictionary = eval(data.data)
                self.battery_level = data_dictionary['battery']
                # also available:
                # 'phi', 'psi', 'num_frames', 'altitude', 'ctrl_state',
                # 'vx', 'vy', 'vz', 'theta'
            except:
                print "\n\n  You're not connected to the drone's data stream.\n\n"
                self.run_without_data_stream = True
        else:
            pass # do nothing if no drone


    # defines commands to control the drone
    def control_drone(self, command_string):
        """ this method provides the various drone controls
            it allows simulation when self.use_drone == False
            and actual control when self.use_drone == True
        """
        if self.use_drone == True: # real commands!
            if command_string == "halt":
                self.drone.land()    
                self.airborne = False
                self.drone.halt()
            elif command_string == "land":
                self.drone.land()
                rospy.sleep(0.1)
                self.airborne = False
            elif command_string == "takeoff":
                self.drone.takeoff()
                rospy.sleep(0.1)
                self.airborne = True
            elif command_string == "hover":
                self.drone.hover()
            elif command_string == "left":
                self.drone.moveRight(-self.drone.speed)
            elif command_string == "right":
                self.drone.moveRight(self.drone.speed)
            elif command_string == "up":
                self.drone.moveUp(self.drone.speed)
            elif command_string == "down":
                self.drone.moveUp(-self.drone.speed)
            elif command_string == "forward":
                self.drone.moveForward(self.drone.speed)
            elif command_string == "backward":
                self.drone.moveForward(-self.drone.speed)
            elif command_string == "turn_right":
                self.drone.moveRotate(2*self.drone.speed)
            elif command_string == "turn_left":
                self.drone.moveRotate(-2*self.drone.speed)
            # no else!
                
            print "control_drone command (real): ", command_string

        else:  # simulated commands
            print "control_drone command (sim): ", command_string


    # handle mouse events
    def onMouse(self,event,x,y,flags,param):
        """ mouse-click handler from OpenCV """
        if event == cv.CV_EVENT_LBUTTONDOWN and self.color_image != None:
            b, g, r = self.color_image[y,x]
            if self.hsv != None: h, s, v = self.hsv[y,x]
            print "Pixel has (r,g,b) =", (r,g,b),
            print "and (h,s,v) =", (h,s,v)


    # do all of the image processing
    def find_landmark(self):
        """ here is where the image should be processed to get the bounding box """
        # check if we've created the supporting images yet
        if self.threshed_image == None:
            if self.color_image != None:
                self.create_all_images()

        # from the old method call def threshold_image(self):
        cv.Split(self.color_image, self.blue, self.green, self.red, None)
        cv.CvtColor(self.color_image, self.hsv, cv.CV_RGB2HSV)
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
        # named self.threshed_image
        cv.Mul(self.red, self.green, self.threshed_image)
        cv.Mul(self.threshed_image, self.blue, self.threshed_image)
        cv.Mul(self.threshed_image, self.hue, self.threshed_image)
        cv.Mul(self.threshed_image, self.sat, self.threshed_image)
        cv.Mul(self.threshed_image, self.val, self.threshed_image)
        # erode and dilate shave off and add edge pixels respectively
        cv.Erode(self.threshed_image, self.threshed_image, iterations = 1)
        cv.Dilate(self.threshed_image, self.threshed_image, iterations = 1)

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
            br = cv.BoundingRect(biggest,update=0)

            #print "in find_regions, br is", br

            # you will want to change these so that they draw
            # a box around the largest contour and a circle at
            # its center:


            xl=br[0]
            xr=br[0] + br[2]
            yt=br[1]
            yb = yt +br[3]
            self.boxX = (xl + xr)/2
            self.boxHeight = (yb - yt)

            #print "boxx " + str(self.boxX)
            #print "boxy " + str(self.boxHeight)


            #Example of drawing a red box
            cv.PolyLine(self.color_image,[[(xl,yt),(xl,yb),(xr,yb),(xr,yt)]],1, cv.RGB(0, 0, 255))
                        
            #Draw the contours in white with inner ones in green
            #cv.DrawContours(self.color_image, biggest, cv.RGB(255,255,255),\
            #                cv.RGB(0, 255, 0), 1, thickness=2, lineType=8,\
            #                offset=(0, 0))
            self.biggest = biggestArea
            if biggestArea < 200 and self.state != "Keyboard" and self.state != "takeoff" \
                    and self.state != "landing":
                self.state = "searching"

        elif len(contours) == 0 and self.state != "Keyboard" and self.state != "takeoff" \
                    and self.state != "landing":
            self.state = "searching"


    # put text on the image...
    def text_to_image(self):
        """ write various things on the image ... """
        # the image is 320 pixels wide and 240 pixels high
        # clear a rectangle
        cv.Rectangle(self.color_image, (3,3), (242,30),
                     cv.RGB(255,255,255), cv.CV_FILLED )
                    
        # set up some text           
        llx = 7
        lly = 22
        s = "State: " + str(self.state)
        textllpoint = (llx,lly)
        cv.PutText(self.color_image, s, textllpoint, self.font, cv.RGB(0,0,255))


    # our drone's state machine
    def FSM1(self, timer_event=None):
        """ the finite-state machine """
        print "Now in state", self.state

        if self.state == "Keyboard": # user stopped our state machine!
            print "State machine has been stopped!"
            print "Sending hover command"
            if self.airborne == True: # only hover if airborne!
                self.control_drone( "hover" )
            return # officially ends the state machine

        elif self.state == "start":
            print "Starting the state machine!"
            self.state = "takeoff"
            rospy.Timer( rospy.Duration(0.1), self.FSM1, oneshot=True )
            
        elif self.state == "takeoff":
            print "taking off in the FSM"
            self.control_drone( "takeoff" )
            rospy.sleep(5.0)  # wait for takeoff...
            self.state = "hover"
            # reschedule the next state-machine callback
            rospy.Timer( rospy.Duration(0.1), self.FSM1, oneshot=True )
            return

        elif self.state == "landing":
            print "landing"
            self.control_drone( "land" )
            rospy.sleep(5.0)
            self.state = "Keyboard"
            rospy.Timer( rospy.Duration(0.1), self.FSM1, oneshot=True )
            return

        elif self.state == "wait_to_c":
            rospy.sleep(0.2)
            if self.saw_capital_c:
                self.state = "searching"
            rospy.Timer( rospy.Duration(0.1), self.FSM1, oneshot=True )
            return

        elif self.state == "searching":
            if abs(self.boxX - self.tarX) < 40:
                self.state = "approaching"
            elif self.boxX < self.tarX:
                self.control_drone( "turn_left" )
            else:
                self.control_drone( "turn_right" )
            rospy.Timer( rospy.Duration(0.1), self.FSM1, oneshot=True )
            return
        
        elif self.state == "approaching":
            if (time.time() - self.lastHover) > .6:
                self.state = "hover2"
            elif abs(self.boxX - self.tarX) > 40:
                self.state = "searching"
            elif self.tarHeight - self.boxHeight > 5:
                self.control_drone( "forward" )
                rospy.sleep(.025)
            elif self.boxHeight - self.tarHeight > 5:
                self.control_drone( "backward" )
                rospy.sleep(.025)
            else:
                self.state = "landing"
            rospy.Timer( rospy.Duration(0.1), self.FSM1, oneshot=True )
            return

        elif self.state == "hover2":
            self.lastHover = time.time()
            self.control_drone( "hover" )
            rospy.sleep(0.1)
            self.state = "searching"
            self.saw_capital_c = False
            rospy.Timer( rospy.Duration(0.1), self.FSM1, oneshot=True )
            return

        elif self.state == "hover":
            self.lastHover = time.time()
            self.control_drone( "hover" )
            rospy.sleep(0.1)
            self.state = "wait_to_c"
            self.saw_capital_c = False
            rospy.Timer( rospy.Duration(0.1), self.FSM1, oneshot=True )
            return

        else:  # state not recognized
            print "the state", self.state, "was not recognized"
            print "Changing to Keyboard state"
            self.state = "Keyboard"
            rospy.Timer( rospy.Duration(0.1), self.FSM1, oneshot=True )
            return
        
                
    # the keyboard thread is the "main" thread for this program
    def keyboardThread(self):
        """ the main keypress-handling thread to control the drone """
        self.airborne = False

        # this is the main loop for the keyboard thread
        #
        # don't use 'Q', 'R', 'S', or 'T'  (they're the arrow keys)
        #
        while True:

            if self.use_drone == False: # if no drone, we get our own images
                self.handle_next_image(None)

            # handle the image processing if we have a new Kinect image
            if self.new_image == True:
                self.new_image = False # until we get a new one...
                # now, do the image processing
                self.find_landmark()
                # now, put text information on the image
                self.text_to_image()
                # show the image
                cv.ShowImage('image', self.color_image)
                cv.ShowImage('threshold', self.threshed_image)
            
            # get the next keypress
            c = cv.WaitKey(25)&255

            # handle the keypress to quit...
            if c == ord('q') or c == 27: # the Esc key is 27
                self.state = "Keyboard" # stop the FSM
                self.control_drone( "halt" ) # lands and halts
                rospy.sleep(1.42) # wait a bit for everything to finish...
                # it's important that the FSM not re-schedule itself further than
                # 1 second in the future, so that a thread won't be left running
                # after this Python program quits right here:
                return
            

            if c == ord('F'):  # capital-F starts and stops the finite-state machine
                # only run it when it's not already running!
                if self.state == "Keyboard":  # it's not yet running, so
                    print "Starting state machine..."
                    self.state = "takeoff"  # these two lines start FSM1()
                    self.FSM1()
                # if it's already running, stop it!
                else:  # if it's in any other state, we bring it back to "Keyboard"
                    self.state = "Keyboard" # should stop the state machine
                    print "Stopping state machine..."


            # saving and loading the thresholds from data.txt
            if c == ord('$'):  # dollar sign to save (looks like an S)
                self.save_thresholds()

            if c == ord('&'):  # ampersand loads thresholds from data.txt
                self.load_thresholds()

            if c == ord('C'):
                self.saw_capital_c = True


            # here are the keyboard commands for the drone...
            #
            # ** NEW THIS WEEK **   you must hit the space bar to stop a motion
            #
            # that is, the space bar hovers; it no longer automatically hovers...

            if c == ord('\n'):
                self.control_drone( "land" )   #self.drone.land()

            elif c == ord('!'): 
                self.control_drone( "takeoff" )   #self.drone.takeoff()

            elif c == ord(' '):  # the space bar will hover
                self.control_drone( "hover" )   #self.drone.hover()
                
            elif c == ord('e'):   # 'e' switches the video feed
                print 'switch video feed'
                self.drone.cam = (1 + self.drone.cam) % 4
                self.drone.feed(self.drone.cam)

            elif c == ord('b'):   # 'b' for battery level
                print "Battery: ", self.battery_level
                

            # this *should* be an if, not an elif - it should always run
            if self.airborne:   # the different direction controls
                if c == ord(' '):
                    self.control_drone( "hover" )      #self.drone.hover()
                elif c == ord('a'):
                    self.control_drone( "left" )       #self.drone.moveRight(-self.drone.speed)
                elif c == ord('d'):
                    self.control_drone( "right" )      #self.drone.moveRight(self.drone.speed)
                elif c == ord('w'):
                    self.control_drone( "forward" )    #self.drone.moveForward(self.drone.speed)
                elif c == ord('s'):
                    self.control_drone( "backward" )   #self.drone.moveForward(-self.drone.speed)
                elif c == ord('j'):
                    self.control_drone( "turn_left" )  #self.drone.moveRotate(-2*self.drone.speed)
                elif c == ord('l'):
                    self.control_drone( "turn_right" ) #self.drone.moveRotate(2*self.drone.speed)
                elif c == ord('i'):
                    self.control_drone( "up" )         #self.drone.moveUp(self.drone.speed)
                elif c == ord('k'):
                    self.control_drone( "down" )       #self.drone.moveUp(-self.drone.speed)

            # if self.use_drone == False (simulated mode)
            # the arrow keys allow you to change the images you see...
            if self.use_drone == False:
                if c == 82:  # up arrow is the same as ord('R')
                    self.change_location( 'west' )
                if c == 84:  # down arrow is the same as ord('T')
                    self.change_location( 'east' )
                if c == 81:  # left arrow is the same as ord('Q')
                    self.change_location( 'north' )
                if c == 83:  # right arrow is the same as ord('S')
                    self.change_location( 'south' )
                if c == ord('-'):  # '-' decreases the image_hour
                    self.change_location( 'counterclockwise' )
                if c == ord('=') or c == ord('+'):  # '=' or '+' increases the image_hour
                    self.change_location( 'clockwise' )



    
if __name__ == '__main__':
    '''
    Main driver function for the drone.
    '''
    # Setup stuff
    rospy.init_node('dronekinect')  # names this ROS node

    use_drone = False
    controller = DroneController(use_drone)

    if use_drone == True:
        rospy.Subscriber('arimage',sm.Image,controller.handle_next_image)
        rospy.Subscriber('arnavdata', msg.NavData, controller.getData)

    # Display initial message
    print 'Keyboard controls:\n'
    print '! (exclamation point) takes off'
    print '\\n (return/enter) lands the drone'
    print 'space bar hovers (stops the drone\'s motion)\n'
    print 'The wasd keys translate at a fixed height and orientation'
    print 'i/k move the drone up/down'
    print 'j/l turn the drone left/right\n'
    print 'q or esc lands the drone and quits the program.'

    rospy.sleep(1)
    
    # start the main loop, the keyboard thread:
    controller.keyboardThread()
    
    print "the Python program is quitting..."


