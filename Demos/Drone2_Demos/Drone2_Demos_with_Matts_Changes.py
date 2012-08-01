USE_DRONE = True
USE_ROOMBA = False
USE_FIRST_DRONE = False
USE_720P = True

## For ROOMBA
# sudo rm /dev/rfcomm0
# sudo rfcomm connect 0 ADDRESS 1
# rosparam set /irobot_mudd/port /dev/rfcomm0  -OR- rosparam set /irobot_mudd /dev/rfcomm0
# rosrun irobot_mudd driver.py 

#########################################
#########  Things to Import if  #########
#########   Using the Drones    #########
#########################################
if(USE_DRONE):
  import roslib; roslib.load_manifest('ardrone2_mudd')
  from ardrone2_mudd.srv import *
  from ardrone2_mudd.msg import *
  from sensor_msgs.msg import *
  import rospy
  
#########################################
#########  Things to Import if  #########
#########   Using the Roomba    #########
######################################### 
if(USE_ROOMBA):
  roslib.load_manifest('irobot_mudd')
  from irobot_mudd.srv import *
  from irobot_mudd.msg import *

#########################################
#########  Imports other things #########
#########################################
import time,sys,random,cv,cv_bridge,math


#########################################
#########    Variable Setting   #########
#########################################
BOUNDING_DATA_SOURCE = "imageData"
BOUNDING_DATA_SOURCE2 = "imageData2"
NAV_DATA_SOURCE = "ardrone2/navData"
DRONE_CONTROL_SOURCE = "ardrone2/heli"

#########################################
#########   Outside Functions   #########
#########################################   
if (USE_ROOMBA):
  def get_services():
      """ returns an object (tank) that allows you
         to set the velocities of the robot's wheels
      """
      # obtain the tank service
      rospy.wait_for_service('tank') # won't continue until the "tank" service is on
      tank = rospy.ServiceProxy('tank', Tank) # tank permits requests, e.g., tank(0,50,50)
      # obtain the song service
      rospy.wait_for_service('song') #
      song = rospy.ServiceProxy('song', Song)
      # returning two things is easy with Python:
      return tank, song

#########################################
#########   External Callbacks  #########
######################################### 

if (USE_ROOMBA):
  def handle_sensor_data( data ):
    """handle_sensor_data is called every time the robot gets a new sensorPacket"""

    #uncomment this to see all of the fields of the sensorPacket
    #print dir( data )
    if data.bumpLeft == True or data.bumpRight == True:
      controller.roomba_bumped = True

#########################################
#########  Drone Controll Class #########
#########################################

class DroneController:
    """
    The drone is keyboard-controlled for movement. Clicking in the image
    window will give the RGB values at that point.
    """
    def __init__(self, use_drone = False):
        """ constructor; setting up data """

        # Setting which camera to use.
        self.camera_number = 0
        # To check if we're using the drone.
        self.use_drone = use_drone
        
        if self.use_drone:
            self.navDataSource = NAV_DATA_SOURCE
            self.boundingDataSource = BOUNDING_DATA_SOURCE
            self.boundingDataSource2 = BOUNDING_DATA_SOURCE2
            self.droneControlSource = DRONE_CONTROL_SOURCE
            self.configSource = 'ardrone2/config'
            self.init_drone_parameters()
        else:
            print "Not Using Drone"


        # Initial Values!
        self.run_without_data_stream = False
        self.battery_level = "Not read from drone"

        # Data for the Roomba
        self.roomba_bumped = False
	self.tank = 0
	self.speed = 80

	self.centerX = 0
        self.centerY = 0
        self.area = 0
        self.height = 0
        self.width = 0
        self.nodata = False
        self.speed_factor
        
        self.last_image_time = time.time()
        # last time an image was grabbed
        # images and other data for the images/windows
        self.bridge = cv_bridge.CvBridge()
        # the interface to OpenCV
        # variables for state machine
        self.state = "Keyboard"


        
    #########################################
    ########      Subscriptions +   #########
    ########      Control Window    #########
    #########################################
    def init_drone_parameters(self):
        """ drone-specific parameters: speed, camera, etc. """
        print "Constructing the drone software object..."

        # Building the control window
        cv.NamedWindow('control')
        cv.MoveWindow('control', 200, 500)
        
        print "Connecting to droneControl service"
        rospy.wait_for_service(self.droneControlSource)
        self.heli = rospy.ServiceProxy(self.droneControlSource, Control, persistent=True)
        
        print "\r Connecting to navData service"
        rospy.Subscriber(self.navDataSource,navData, self.navDataUpdate, queue_size=1)

        print "\r Connecting to bounding box data"
        rospy.Subscriber(self.boundingDataSource,std_msgs.msg.String, self.bBoxUpdate, queue_size=1)


        self.send(4,0,0,0,0)
        print "\r Connecting to config\n"
        rospy.wait_for_service('ardrone2/config')
        self.config = rospy.ServiceProxy(self.configSource, Config)

        print "\r Connected to services\n"
        self.send(4,0,0,0,0)
        print "Drone initialized."



    #########################################
    #########  Internal Callbacks   #########
    #########################################
        
    def navDataUpdate(self, data):
        """The navData gets sent here to then be send to prettyWrite
          so that it is nicely formated"""
        self.battery_level = data.batLevel

    #########################################
    #########  Internal Functions   #########
    #########################################
      
    def send(self,flag,phi,theta,gaz,yaw):
      self.lastsent = flag,phi,theta,gaz,yaw
      self.heli(flag,phi,theta,gaz,yaw)    
      rospy.sleep(.05)
      
    #########################################
    #########      Box Updates      #########
    #########################################
        
    def bBoxUpdate(self, data):
      """bBoxUpdate takes the information that the first publisher
        of heliimage returns. It calculats the X and Y centers of
        the biggest region. It also adjust for something that is far
        or if there is no biggest region."""
      # Splits data.
      br = data.data.split()
      
      centerX = int(br[0])
      centerY = int(br[1])
      area = int(br[2])
      left = int(br[3])
      top = int(br[4])
      right = int(br[5])
      bottom = int(br[6])

      self.centerX = centerX
      self.centerY = centerY
      self.area = area
      self.height = bottom - top
      self.width = right - left

      if self.area <= 0.1 or self.area >1:
          # TEST ALERT -- 10 percent of the image or less means insignificant.
          self.nodata = True

      if self.height > 1 or self.height <= 0:
        self.speed_factor = 1
      else:
        self.speed_factor = 1 - self.height
        

    #########################################
    #########         FSM           #########
    #########################################

    def FSM1(self, timer_event=None):
      """ the finite-state machine that looks around and then centers"""
      print "Now in state (FSM1)", self.state

      if self.state == "Keyboard":
        # user stopped our state machine!
        self.send(2,0,0,0,0)
        rospy.sleep(1.0)
        return
      elif self.state == "takeoff":
        rospy.sleep(1.0)
        self.send(3,0,0,0,0)
        rospy.sleep(4.0)
        self.state = "hover"
      elif self.state == "hover":
        self.send(0,0,0,0,0)
        rospy.sleep(1.0)
        self.state = "looking"
      elif self.state == "looking":
        if not self.nodata and abs(self.centerX -.50)< 0.05:
          self.state = "approach"
        elif self.centerX < .50:
          self.send(0,0,0,0,-.15)
        else:
          self.send (0,0,0,0,.15)
      elif self.state == "approach":
        if self.nodata or abs(self.centerX -.50) > 0.05:
          self.state = "looking"
        elif self.height < .7:
          if abs(self.centerY- .50)<0.05:
            self.send(1,0,-.2*self.speed_factor,0,0)
            #rospy.sleep(.02)
            self.state = "looking"
          else:
	    print 'STRAIT AND UP'
            self.send(1,0,-.2*self.speed_factor,.1*self.speed_factor,0)
            #rospy.sleep(.02)
            self.state = "looking"
        else:
          print self.area
          self.send(0,0,0,0,0)
          rospy.sleep(.25)
          self.state = "Keyboard"
      rospy.Timer( rospy.Duration(0.05), self.FSM1, oneshot=True )

    def FSM3(self, timer_event=None):
      """ the finite-state machine that centers using the downward camera and then lands"""
      print self.state
      if self.state == "Keyboard":
        # user stopped our state machine!
        self.send(2,0,0,0,0)
        self.state = "Stopped"
        return
      elif self.state == "takeoff":
        rospy.sleep(1.0)
        self.send(3,0,0,0,0)
        rospy.sleep(4.0)
        self.state = "hover"
      elif self.state == "hover":
        self.send(0,0,0,0,0)
        self.state = "scanning"
      elif self.state == "scanning":
        # Revmoved that boxX = 0 from the below.
        if self.nodata == True:
          print "Going Vertically UP"
          self.send(0,0,0,.15,0)
        else:
          self.state = "Ycenter"
      elif self.state == "Ycenter":
        if abs(self.tarY-self.boxY) < 50:
          self.state = "Xcenter"
          # Y is centered, now do X
        elif self.boxY < self.tarY:
          print "Going forward"
          self.send(1,0,-.15,0,0)
          self.state = "scanning"
        else:
          print "Going backwards"
          self.send(1,0,.15,0,0)
          self.state = "scanning"
      elif self.state == "Xcenter":
        if abs(self.tarX-self.boxX)< 50:
          self.state = "Centered"
        elif self.boxX < self.tarX:
          print "Going left"
          self.send(1,-.15,0,0,0)
          self.state = "scanning"
        else:
          print "Going right"
          self.send(1,.15,0,0,0)
          self.state = "scanning"
      elif self.state == "Centered":
        if self.area > 60000:
          print self.area
          self.send(0,0,0,0,0)
          self.state = "Keyboard"
        else:
          print "Going Down"
          self.send(0,0,0,-.15,0)
          rospy.sleep(0.2)
          self.state = "scanning"
      rospy.Timer( rospy.Duration(0.1), self.FSM3, oneshot=True )

          
    # the keyboard thread is the "main" thread for this program
    def keyboardLoop(self):
        """ the main keypress-handling thread t5o control the drone """
        maxpower = .15

        # this is the main loop for the keyboard thread
        # don't use 'Q', 'R', 'S', or 'T'  (they're the arrow keys)
        
        while True:
          c = chr(cv.WaitKey()&255)

          #########################################
          #########    Drone KeyPresses   #########
          #########################################


          # handle the keypress to quit...
          if c == 'q' or c == chr(27): # the Esc key is 27
            self.state = "Keyboard" # stop the FSM
            self.send(2,0,0,0,0) # lands and halts
            rospy.sleep(1.42) # wait a bit for everything to finish...
            # it's important that the FSM not re-schedule itself further than
            # 1 second in the future, so that a thread won't be left running
            # after this Python program quits right here:
            return

          drone_dictionary = {'1': self.FSM1,\
                  '3': self.FSM3}

          if c in drone_dictionary.keys():
            if self.state == "Keyboard":
              print "Starting state machine FSM", c
              self.state = "takeoff"
              drone_dictionary[c]()
            else: 
              self.state = "Keyboard" 
              print "Stopping state machine..."
              

          helistr= ""
          if c == '\n':
            self.state = "Keyboard"
            helistr = 2,0,0,0,0
            
          elif c == 't':
            print "takeoff"
            helistr = 3,0,0,0,0

          elif c == 'v':
            self.camera_number = (self.camera_number + 1) % 4
            print "self.camera_n umber is", self.camera_number
            self.config("camera " + str(self.camera_number))

          elif c == chr(81):
            print "turning left"
            helistr = 0,0,0,0,-.15

          elif c == chr(83):
            print "you pressed right"
            helistr = 0,0,0,0,.15

          elif c == chr(82):
            print "going forwards"
            helistr = 1,0,-0.3,0,0

          elif c == chr(84):
            print "going backwards"
            helistr = 1,0,0.3,0,0

          elif c == '=':
            print "going up"
            helistr = 0,0,0,.15,0

          elif c == '-':
            print "going down"
            helistr = 0,0,0,-.15,0

          elif c == '[':
            print "strafe left"
            helistr = 1,-.10,0,0,0

          elif c == ']':
            print "strafe right"
            helistr = 1,.10,0,0,0

          elif c == '9':
            print "hover w/o downward concern"
            helistr = 1,0,0,0,0

          elif c == '0':
            print "hover w/ downward concern"
            helistr = 0,0,0,0,0

          elif c == 'b':
            print self.battery_level

          elif c == 'f':
            self.config("anim 16")
            self.send(0,0,0,0,0)
          elif c == 'g':
            self.config("anim 17")
            self.send(0,0,0,0,0)
          elif c == 'h':
            self.config("anim 18")
            self.send(0,0,0,0,0)
          elif c == 'j':
            self.config("anim 19")
            self.send(0,0,0,0,0)            
          if not helistr == '':
            #print len(helistr)
            sflag, sphi, stheta, sgaz, syaw = helistr
            self.send(sflag,sphi,stheta,sgaz,syaw)


          #########################################
          #########   Roomba KeyPresses   #########
          #########################################
          if(USE_ROOMBA):
            if c == ' ':
              controller.tank(0,0)
            if c == 'a':
              controller.tank(-self.speed,self.speed)
            if c == 'd':
              controller.tank(self.speed,-self.speed)
            if c == 'w':
              controller.tank(self.speed,self.speed)
            if c == 's':
              controller.tank(-self.speed,-self.speed)
    
if __name__ == '__main__':
    '''
    Main driver function for the drone.
    '''
    # Setup stuff
    rospy.init_node('dronekinect')  # names this ROS node

    use_drone = USE_DRONE
    controller = DroneController(use_drone)
    
    if(USE_ROOMBA):
      controller.tank, controller.song = get_services()
      rospy.Subscriber('sensorPacket', SensorPacket, handle_sensor_data)
 
    # Display initial message
    print 'Keyboard controls:\n'
    print 't takes off'
    print '\\n (return/enter) lands the drone'
    print 'space bar hovers (stops the irobots motion)\n'
    print 'q or esc lands the drone and quits the program.'
    print '1 is the finite-state machine that looks around and then centers'
    print '2 is the finite-state machine that centers vertically'
    print '3 is the finite-state machine that centers using\
          the downward camera and then lands'
    print '4 is the finate state machine that works ok at following the Roomba'
    print 'Left arrow - rotate left'
    print 'Right arrow - rotate right'
    print 'Up arrow - go forward'
    print 'Down arrow - go backward'
    print '[ - strafe left'
    print '] - strafe right'
    print '= - goes up'
    print '- - goes down'
    print '9 - hover w/o downward concern'
    print '0 - hover w/  downward concern'
    if(USE_ROOMBA):
      print 'w - Roomba forward'
      print 's - Roomba backward'
      print 'a - Roomba rotate left'
      print 'd - Roomba rotate right'

    
    rospy.sleep(1)
    
    # start the main loop, the keyboard thread:
    controller.keyboardLoop()
    
    print "the Python program is quitting..."
       
