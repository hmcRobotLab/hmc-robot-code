USE_DRONE = True
USE_ROOMBA = False
USE_FIRST_DRONE = True
USE_720P = False

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
        


        # Initial Values!
        self.run_without_data_stream = False
        self.battery_level = "Not read from drone"

        # Data for the first threshold
        self.boxHeight = 0
        self.boxX = 0
        self.boxY = 0
        self.tarX = 320
        self.tarY = 180
        self.nodata = True
        self.area = 0

        # Data for the second threshold
        self.boxGHeight = 0
	self.boxGX = 0
        self.boxGY = 0
        self.tarGX = 320
        self.tarGY = 180
        self.nodata2 = True
        self.area2 = 0

        # Data goals for both thresholds
        self.tarHeight = 100
        self.image_size_thresh = 400
        self.buffer_range  = 40 

        # Data for the Roomba
        self.roomba_bumped = False
	self.tank = 0
	self.speed = 80
	
	if(USE_FIRST_DRONE):
          self.tarX = 160
          self.tarY = 120
          self.tarGX = 160
          self.tarGY = 120
          self.tarHeight = 45
          self.image_size_thresh = 200


        if(USE_720P):
          self.tarX = 640
          self.tarY = 360
          self.tarGX = 640
          self.tarGY = 360
          self.tarHeight = 235
          self.image_size_thresh = 600
	  self.buffer_range = 60
	  print '720 P Settings Set'
          
        
        self.last_image_time = time.time()
        # last time an image was grabbed
        # images and other data for the images/windows
        self.bridge = cv_bridge.CvBridge()
        # the interface to OpenCV
        # variables for state machine
        self.state = "Keyboard"

        if self.use_drone:
            self.navDataSource = NAV_DATA_SOURCE
            self.boundingDataSource = BOUNDING_DATA_SOURCE
            self.boundingDataSource2 = BOUNDING_DATA_SOURCE2
            self.droneControlSource = DRONE_CONTROL_SOURCE
            self.configSource = 'ardrone2/config'
            self.init_drone_parameters()
        else:
            print "Not Using Drone"

        
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

        print "\r Connecting to bounding box data2"
        rospy.Subscriber(self.boundingDataSource2,std_msgs.msg.String, self.bBoxUpdate2, queue_size=1) 

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
      xlf = float(br[0])
      if xlf == float("inf"):
        """Checks to see if the first data received is the float inf. If
          it is, it's because there was no data."""
        self.nodata = True
        self.boxX = float("inf")
        
      else:
        # Splits all four parts if there is proper data.
        xl = int(br[0])
        xr = int(br[1])
        yt = int(br[2])
        yb = int(br[3])

        if (xr-xl)*(yb-yt) < self.image_size_thresh:
            self.nodata = True
            self.boxX = float("inf")
            self.area = (xr-xl)*(yb-yt)
        else:
            self.nodata = False
            # The center of the box's X
            self.boxX = (xl + xr)/2
            # The center of the box's Y
            self.boxY = (yt+yb)/2
            # The height of the box
            self.boxHeight = yb - yt
            # The area of the box
            self.area = (xr-xl)*(yb-yt)

    def bBoxUpdate2(self, data):
      """bBoxUpdate takes the information that the second publisher
        of heliimage returns. It calculats the X and Y centers of
        the biggest region. It also adjust for something that is far
        or if there is no biggest region."""
      br = data.data.split()
      xlf = float(br[0])

      if xlf == float("inf"):
        """Checks to see if the first data received is the float inf. If
        it is, it's because there was no data."""
        self.nodata2 = True
        self.boxGX = float("inf")

      else:
        # Splits all four parts if there is proper data.
        xlg = int(br[0])
        xrg = int(br[1])
        ytg = int(br[2])
        ybg = int(br[3])

        if (xrg-xlg)*(ybg-ytg) < self.image_size_thresh:
          self.nodata2 = True
          self.boxGX = float("inf")
          self.area2 = (xrg-xlg)*(ybg-ytg)
        else:
          self.nodata2 = False
          # The center of the box's X
          self.boxGX = (xlg+xrg)/2
          # The center of the box's Y 
          self.boxGY = (ytg+ybg)/2
          # The height of the box
          self.boxGHeight = ybg - ytg
          # The area of the box
          self.area2 = (xrg-xlg)*(ybg-ytg)


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
        print self.boxX
        if abs(self.tarX-self.boxX) <self.buffer_range:
          self.state = "approach"
        elif self.boxX < self.tarX:
          self.send(0,0,0,0,-.15)
        else:
          self.send (0,0,0,0,.15)
      elif self.state == "approach":
        if abs(self.boxX - self.tarX) > self.buffer_range:
          self.state = "looking"
        elif self.tarHeight - self.boxHeight > 20:
          print self.tarHeight - self.boxHeight
          if abs(self.tarY-self.boxY) < self.buffer_range:
            self.send(1,0,-.2,0,0)
            #rospy.sleep(.02)
            self.state = "looking"
          else:
	    print 'STRAIT AND UP'
            self.send(1,0,-.2,.1,0)
            #rospy.sleep(.02)
            self.state = "looking"
        else:
          print self.tarHeight - self.boxHeight
          self.send(0,0,0,0,0)
          rospy.sleep(.25)
          self.state = "Keyboard"
      rospy.Timer( rospy.Duration(0.05), self.FSM1, oneshot=True )

    def FSM2(self, timer_event=None):
        """ the finite-state machine that centers vertically"""
        #print self.boxY
        print "Now in state", self.state
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
          self.state = "updown"
        elif self.state == "updown":
          print self.boxY
          if self.tarY-self.boxY < 0 :
            self.state = "Ycentered"
          elif self.boxY < self.tarY:
            self.send(0,0,0,.15,0)
            self.state = "hover"
          else:
            self.send (0,0,0,-.15,0)
            self.state = "hover"
        elif self.state == "Ycentered":
          self.send(0,0,0,0,0)
          rospy.sleep(4.0)
          self.state = "Keyboard"
        rospy.Timer( rospy.Duration(0.1), self.FSM2, oneshot=True )

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


    def FSM4(self, timer_event=None):
      """the finate state machine that works ok at following the Roomba"""
      if(USE_ROOMBA):
        print "the state is:", self.state
        print "data missing2", self.nodata2
        if self.state == "Keyboard":
          # user stopped our state machine!
          self.send(2,0,0,0,0)
          self.state = "Stopped"
          return
        elif self.roomba_bumped == True:
          print "I did bump!!! Yay me! Now I need to go use another FSM"
          self.state = "hover"
          self.FSM3()
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
          # Now that the scanning occurs -- follow the robot.
          if self.nodata2 == True:
            self.state = "hover"
          else:
            if abs(self.tarGY-self.boxGY) < 40:
              self.state = "scanning"
            elif self.boxGY < self.tarGY:
              print "Going forward"
              self.send(1,0,-.07,0,0)
              self.state = "scanning"
            elif self.boxGY > self.tarGY:
              print "Going backwards"
              self.send(1,0,.07,0,0)
              self.state = "scanning"
            if abs(self.tarGX-self.boxGX) < 40:
              self.state = "scanning"
            elif self.boxGX < self.tarGX:
              print "Going left"
              self.send(1,-.07,0,0,0)
              self.state = "scanning"
            elif self.boxGX > self.tarGX :
              print "Going right"
              self.send(1,.07,0,0,0)
              self.state = "scanning"
        rospy.Timer( rospy.Duration(0.05), self.FSM4, oneshot=True )
      else:
        self.state = "Keyboard"
        print "No Roomba Connected!"

    def FSM5(self, timer_event=None):
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
        print self.boxX
        if abs(self.tarX-self.boxX) <self.buffer_range:
          self.state = "approach"
        elif self.boxX < self.tarX:
          self.send(0,0,0,0,-.15)
        else:
          self.send (0,0,0,0,.15)
      elif self.state == "approach":
        if abs(self.boxX - self.tarX) > self.buffer_range:
          self.state = "looking"
        elif self.tarHeight - self.boxHeight > 20:
          print self.tarHeight - self.boxHeight
          if abs(self.tarY-self.boxY) < self.buffer_range:
            self.send(1,0,-.2,0,0)
            #rospy.sleep(.02)
            self.state = "looking"
          else:
	          print 'STRAIGHT AND UP'
            self.send(1,0,-.2,.1,0)
            #rospy.sleep(.02)
            self.state = "looking"
        else:
          print self.tarHeight - self.boxHeight
          self.send(0,0,0,0,0)
          rospy.sleep(.25)
          self.state = "Keyboard"
      rospy.Timer( rospy.Duration(0.05), self.FSM5, oneshot=True)

          
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
                  '2': self.FSM2,\
                  '3': self.FSM3,\
                  '4': self.FSM4}

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

          if USE_ROOMBA:
            if c == '0':
              print "hover w/ downward concern"
              helistr = 0,0,0,0,0
          else:
            if c == ' ':
              print "hover w/ downward concern"
              helistr = 0,0,0,0,0

          if c == 'b':
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
       
