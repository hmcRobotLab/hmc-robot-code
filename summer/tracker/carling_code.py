#!/usr/bin/env python
import roslib; roslib.load_manifest('Daneel')
import rospy

import sensor_msgs.msg as sm
import cv_bridge
import cv
import datetime
import threading

import pydrone.srv
import pydrone.ARDrone as ARDrone
import pydrone.msg as msg


class DroneController:
    """
    The drone is keyboard-controlled for movement. Clicking in the image
    window will give the RGB values at that point.
    """

    def __init__(self, use_drone = False):
        """ constructor; setting up data """
        self.use_drone = use_drone  # are we using the drone?
        
        self.init_drone_parameters()
        self.bridge = cv_bridge.CvBridge()
        self.counter = 0

        cv.NamedWindow('dist')
        cv.MoveWindow('dist', 100, 0)
        cv.SetMouseCallback('dist', self.onMouse, None)

        self.scale = 1
        self.left_x = 100
        self.make_slider_window()

        self.font = cv.InitFont(cv.CV_FONT_HERSHEY_SIMPLEX, .5, .5, 0, 2)
        self.show_on_image = True

        self.image = None
        self.color_image = None
        self.new_image = False

        self.gesture_found = False
        self.gesture_keypress = 255   # the default "no keypress" value
        self.airborne = False         # not airborne yet
        


    def init_drone_parameters(self):
        """ drone-specific parameters: speed, camera, etc. """
        print "Constructing the drone software object..."
        if self.use_drone == True:
            self.drone = ARDrone.ARDrone()
            # Drone's motion speed
            self.drone.speed = 0.4
            self.drone.cam = 1 # 0 for forward, 1 for floor camera (can be changed
                          # in flight)
            self.drone.frequency = 8 # Drone's ultrasound frequency - 7 for 22.22
                                # Hz, 8 for 25 Hz
            self.drone.altmax_value = 10000 # Drone's maximum altitude between 500
                                       # and 5000 or 10000 for unlimited

            # print "Trying to connect to the drone"
            self.drone.connect()
            self.drone.freq(self.drone.frequency)
            self.drone.altmax(self.drone.altmax_value)
            self.drone.feed(1)
            rospy.sleep(1)
            print "Drone initialized."
        else:
            print "Not using drone."

    def handle_next_image(self, data):
        """Displays the image, calls find_info"""
        #print " handling next image!"
        # get the image from the Kinect
        self.image = self.bridge.imgmsg_to_cv(data, "32FC1")
        cv.Flip( self.image, self.image, 1 )  # flip it!

        # change the scale
        cv.ConvertScale( self.image, self.image, scale=1.0/self.scale, shift=0.0 )

        # create a new color image, if it does not yet exist
        if self.color_image == None:
            self.color_image = cv.CreateImage(cv.GetSize(self.image),
                                              cv.IPL_DEPTH_32F, 3)

        # create *this frame's* color image, now that we know one exists
        cv.Merge( self.image, self.image, self.image, None, self.color_image )

        # tell the keyboard thread that we have a new image
        self.new_image = True


    def make_slider_window(self):
        """ a method to make a window full of sliders """
        #Create slider window
        cv.NamedWindow('sliders')
        cv.MoveWindow('sliders', 800, 0)
        
        #Create sliders
        cv.CreateTrackbar('left_x', 'sliders', self.left_x,\
                          320, self.change_left_x)
        cv.CreateTrackbar('scale', 'sliders', self.scale,\
                          10, self.change_scale)

    #Functions for changing the slider values  
    def change_left_x(self, newval):
        """ change the left-hand x-coordinate of the "pressure points" """
        self.left_x = newval
        
    def change_scale(self, newval):
        self.scale = newval
        if self.scale < 1: self.scale = 1

            
    def getData(self, data):
        """ the drone's navigation and status data """
        if self.use_drone == True and False:
            try:
                data_dictionary = eval(data.data)
                self.battery_level = data_dictionary['battery']
                # also available:
                # 'phi', 'psi', 'num_frames', 'altitude', 'ctrl_state',
                # 'vx', 'vy', 'vz', 'theta'
            except:
                print "You're not connected to all of the drone's data streams."
                print "Try disconnecting completely and reconnecting."
        else:
            pass # do nothing if no drone

    def onMouse(self,event,x,y,flags,param):
        """ mouse-click handler from OpenCV """
        if event == cv.CV_EVENT_LBUTTONDOWN and self.image != None:
            d = self.image[y,x]
            print "Scaled distance value is", d

    def check_for_gestures(self):
        """ determines if a gesture is recognized... """
        top_x = 320
        top_y = 50
        radius = 16
        top_circle_selected = None # None, yet!
        max_depth_allowed = 0.42  # a value too small to happen accidentally
        # the image will need to be scaled down (via the slider)
        cv.Circle(self.color_image, (top_x,top_y), radius,
                  cv.RGB(not self.airborne, self.airborne, 0), 2)
        top_depth = self.image[top_y,top_x]

        
        x = self.left_x  # from the slider's value
        y_max = 272
        
        # make a list of left-hand circles (indices, really)
        left_circle_list = [0,1,2,3,4]
        left_circle_selected = None # None, yet!
        left_circle_min = 42000 # some large value
        # loop through our list...
        for left_circle_index in left_circle_list:
            # compute the y-coordinate
            y = y_max - left_circle_index*2*radius # diameter!
            # draw the default circle
            cv.Circle( self.color_image, (x,y), radius, cv.RGB(0,0,1), 2 )
            # get the kinect's depth...
            depth = self.image[y,x]
            # NOTE: you should make this depth-finding more robust
            #       by looking through the _entire_ diameter and find the min!
            if depth < max_depth_allowed and depth < left_circle_min:
                left_circle_selected = left_circle_index
                left_circle_min = depth
                left_circle_selected_y_coordinate = y
                
        # display the red one...
        if left_circle_selected != None:
            cv.Circle( self.color_image,
                       (x,left_circle_selected_y_coordinate),
                       radius, cv.RGB(1,0,0), 2 )

        # make a list of right-hand circles (indices, really)
        right_circle_list = [0,1,2,3,4]
        right_circle_selected = None # None, yet!
        right_circle_min = 42000 # some large value
        # loop through our list...
        for right_circle_index in right_circle_list:
            # compute the y-coordinate
            y = y_max - right_circle_index*2*radius # diameter!
            # draw the default circle
            cv.Circle( self.color_image, (640-x,y), radius, cv.RGB(0,0,1), 2 )
            # get the kinect's depth...
            depth = self.image[y,640-x]
            # NOTE: you should make this depth-finding more robust
            #       by looking through the _entire_ diameter and find the min!
            if depth < max_depth_allowed and depth < right_circle_min:
                right_circle_selected = right_circle_index
                right_circle_min = depth
                right_circle_selected_y_coordinate = y
                
        # display the red one...
        if right_circle_selected != None:
            cv.Circle( self.color_image,
                       (640-x,right_circle_selected_y_coordinate),
                       radius, cv.RGB(1,0,0), 2 )

        print "The left circle pressed is", left_circle_selected
        print "The right circle pressed is", right_circle_selected

        if top_depth < max_depth_allowed and self.airborne == False:
            self.gesture_found = True
            self.gesture_keypress = ord('T')
        elif top_depth < max_depth_allowed and self.airborne == True:
            self.gesture_found = True
            self.gesture_keypress = ord('\n')
        elif left_circle_selected != None and right_circle_selected != None:
            if left_circle_selected - 2 == -(right_circle_selected - 2):
                self.gesture_found = True
                self.drone.speed = 0.2*(left_circle_selected - 2)
                self.gesture_keypress = ord('d')
        else:
            self.drone.speed = 0.4
            self.gesture_found = False

        '''if left_circle_selected == 2: # our gesture!
            self.gesture_found = True # signals the keyboard thread
            self.gesture_keypress = ord('b') # simple battery check
            # but any gesture couple be used here
            # put some text on the image here --
            # be sure it's the self.color_image!
        else:
            self.gesture_found = False  # signal that there's no gesture
            # don't need a keypress here'''
            

    def keyboardThread(self):
        """ the main keypress-handling thread to control the drone """
        self.airborne = False
        moving = False

        # this is the main loop for the keyboard thread
        while True:
            # handle the image processing if we have a new Kinect image
            if self.new_image == True:
                self.new_image = False # until the Kinect's image thread resets it
                # check for gestures
                self.check_for_gestures()
                # display the image
                cv.ShowImage('dist', self.color_image)
            
            # get the next keypress
            c = cv.WaitKey(50)&255

            # handle the keypress to quit...
            if c == ord('q') or c == 27: # the Esc key is 27
                print 'quit'
                if self.use_drone == True:
                    self.drone.land()
                    self.drone.halt()
                # otherwise, we just quit...
                return

            # if there's no keypress (c == 255), and there was a
            # successfully recognized gesture, then
            # *assign the appropriate keypress from that gesture*
            if c == 255:   # if there was no keypress, this is the value
                if self.gesture_found == True:
                    c = self.gesture_keypress
                    print "Using key #", c, "from a recognized gesture"

            # these are keyboard commands for the drone:
            if self.use_drone != True:
                continue # this stops the current iteration of the while loop
                # and continues back at the top of the next while loop iteration
                
            elif c == ord('T'):   # newline for takeoff/landing
                if not self.airborne:
                    print 'take off'
                    c = 255  # reset c
                    self.airborne = True
                    self.drone.takeoff()
                    rospy.sleep(2.0)

            elif c == ord('\n'):
                if self.airborne:
                    print 'land'
                    c = 255  # reset c
                    self.airborne = False
                    self.drone.land()
                    rospy.sleep(2.0)
                
            elif c == ord('e'):   # 'e' switches the video feed
                print 'switch video feed'
                self.drone.cam = (1 + self.drone.cam) % 4
                self.drone.feed(self.drone.cam)

            elif c == ord('b'):
                print "Battery: ", self.battery_level

            # this should be an if, not an elif - it should always run
            if self.airborne:   # the different direction controls
                if not moving:  # only move if it's not already moving
                    if c == ord('a'):
                        print 'left'
                        moving = True
                        self.drone.moveRight(-self.drone.speed)
                    elif c == ord('d'):
                        print 'right'
                        moving = True
                        self.drone.moveRight(self.drone.speed)
                    elif c == ord('w'):
                        print 'forward'
                        moving = True
                        self.drone.moveForward(self.drone.speed)
                    elif c == ord('s'):
                        print 'backward'
                        moving = True
                        self.drone.moveForward(-self.drone.speed)
                    elif c == ord('j'):
                        print 'turn left'
                        moving = True
                        self.drone.moveRotate(-2*self.drone.speed)
                    elif c == ord('l'):
                        print 'turn right'
                        moving = True
                        self.drone.moveRotate(2*self.drone.speed)
                    elif c == ord('i'):
                        print 'up'
                        moving = True
                        self.drone.moveUp(self.drone.speed)
                    elif c == ord('k'):
                        print 'down'
                        moving = True
                        self.drone.moveUp(-self.drone.speed)
                else:
                    moving = False
                    self.drone.hover()





    
if __name__ == '__main__':
    '''
    Main driver function for the drone.
    '''
    # Setup stuff
    rospy.init_node('dronekinect')  # names this ROS node

    use_drone = True
    controller = DroneController(use_drone)

    rospy.Subscriber('/camera/depth/image',sm.Image,controller.handle_next_image)
    if use_drone == True:
        rospy.Subscriber('arnavdata', msg.NavData, controller.getData)

    # Display initial message
    print 'Controls (all lower case):\n'
    print 'capital-T takes off and return (enter) lands the drone.'
    print 'The wasd keys translate at a fixed height and orientation.'
    print 'i/k move the drone up/down.'
    print 'j/l turn the drone left/right.'
    print 'q or esc lands the drone and quits the program.'
    print 'The drone returns to hover when no key (or another key) is pressed.'

    rospy.sleep(1)
    
    # start the main loop, the keyboard thread:
    controller.keyboardThread()
