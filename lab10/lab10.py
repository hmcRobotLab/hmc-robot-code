USE_DRONE = True

if(USE_DRONE):
  import roslib; roslib.load_manifest('ardrone_mudd')
  from ardrone_mudd.srv import *
  from ardrone_mudd.msg import *
  from sensor_msgs.msg import *
  import rospy

import time,sys,random,cv,cv_bridge,math
BOUNDING_DATA_SOURCE = "imageData"
NAV_DATA_SOURCE = "navData"
DRONE_CONTROL_SOURCE = "droneControl"
    
#class DroneData:
#  altitude = 542
#  vx = 3
#  vy = 4
#  vz = 0
#  phi = 100
#  theta = 4242
#  psi = 3064
#  state = 789

def makeHeliStr(flag, phi, theta,gaz,yaw):
    return "heli %i %.3f %.3f %.3f %.3f" \
        % (flag,phi,theta,gaz,yaw)

def prettyWrite(data,lastsent,boxHeight):
    sys.stdout.write("\r\t [alt: %i] \t [phi: %i psi: %i theta: %i] \t [vx: %i vy: %i vz: %i] \t [bat: %i  state: %i]              \n" \
      %
      (data.altitude,data.phi,data.psi,data.theta,data.vx,data.vy,data.vz,data.batLevel,data.ctrlState))
    sys.stdout.write("\rLast sent: %s      box: %s            \n" % (lastsent, boxHeight))
    sys.stdout.write("\033[A")
    sys.stdout.write("\033[A")
    sys.stdout.flush()


def folderNames():
    return { ('c',0):"./drone_images/c_0_Ginny",
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


class DroneController:
    """
    The drone is keyboard-controlled for movement. Clicking in the image
    window will give the RGB values at that point.
    """

    def __init__(self, use_drone = False):
        """ constructor; setting up data """

        # member variables relating to the drone's control
        self.use_drone = use_drone  # are we using the drone?
        if self.use_drone:
            self.navDataSource = NAV_DATA_SOURCE
            self.boundingDataSource = BOUNDING_DATA_SOURCE
            self.droneControlSource = DRONE_CONTROL_SOURCE
            self.init_drone_parameters()
        else:
            print "Not Using Drone"

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
        self.folder_names = folderNames()
        # images and other data for the images/windows
        self.bridge = cv_bridge.CvBridge()  # the interface to OpenCV

        # variables for state machine
        self.state = "Keyboard"
        self.lastHover = 0

    def init_drone_parameters(self):
        """ drone-specific parameters: speed, camera, etc. """
        print "Constructing the drone software object..."
        cv.NamedWindow('control')
        cv.MoveWindow('control', 200, 500)
        
        print "Connecting to droneControl service"
        rospy.wait_for_service("droneControl")
        self.heli = rospy.ServiceProxy("droneControl", Control, persistent=True)
        
        print "\r Connecting to navData service"
        rospy.Subscriber(self.navDataSource,navData, self.navDataUpdate, queue_size=1)

        print "\r Connecting to bounding box data"
        rospy.Subscriber(self.boundingDataSource,std_msgs.msg.String, self.bBoxUpdate, queue_size=1)

        print "\r Connected to services\n"
        self.send("reset")

        print "Drone initialized."


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

    def navDataUpdate(self, data):
        prettyWrite(data,self.lastsent,self.boxHeight)

    def bBoxUpdate(self, data):
        br = data.data.split()
        xl = int(br[0])
        xr = int(br[1])
        yt = int(br[2])
        yb = int(br[3])
        if (xr-xl)*(yb-yt) < 200:
            self.boxX = float("inf")
        else:
            self.boxX = (xl + xr)/2
            self.boxHeight = yb - yt #the y axis points down.

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
        	    self.send(makeHeliStr(0,0,0,0,0))
            return # officially ends the state machine

        elif self.state == "start":
            print "Starting the state machine!"
            self.state = "takeoff"

        elif self.state == "takeoff":
            print "taking off in the FSM"
            self.send( "takeoff" )
            rospy.sleep(5.0)  # wait for takeoff...
            self.state = "hover"
            # reschedule the next state-machine callback

        elif self.state == "approaching":
            if abs(self.boxX - self.tarX) > 40:
                self.state = "searching"
            elif self.tarHeight - self.boxHeight > 10:
                self.send(makeHeliStr(1,0,-.07,0,0))
                rospy.sleep(.025)
            else:
                self.send(makeHeliStr(0,0,0,0,0))
                rospy.sleep(.25)
                self.state = "landing"

        elif self.state == "landing":
            print "landing"
            self.send( "land" )
            rospy.sleep(5.0)
            self.state = "Keyboard"

        elif self.state == "wait_to_c":
            rospy.sleep(0.2)
            if self.saw_capital_c:
                self.state = "searching"

        elif self.state == "searching":
            if abs(self.boxX - self.tarX) < 40:
                self.state = "approaching"
            elif self.boxX < self.tarX:
                self.send(makeHeliStr(0,0,0,0,-.15))
            else:
                self.send(makeHeliStr(0,0,0,0,.15))

        elif self.state == "hover2":
            self.lastHover = time.time()
            self.send( "heli 0 0 0 0 0 " )
            rospy.sleep(0.1)
            self.state = "searching"
            self.saw_capital_c = False

        elif self.state == "hover":
            self.lastHover = time.time()
            self.send(makeHeliStr(0,0,0,0,0))
            rospy.sleep(0.1)
            self.state = "wait_to_c"
            self.saw_capital_c = False

        else:  # state not recognized
            print "the state", self.state, "was not recognized"
            print "Changing to Keyboard state"
            self.state = "Keyboard"

        rospy.Timer( rospy.Duration(0.1), self.FSM1, oneshot=True )
        return
                
    # the keyboard thread is the "main" thread for this program
    def keyboardLoop(self):
        """ the main keypress-handling thread to control the drone """
        maxpower = .15

        # this is the main loop for the keyboard thread
        #
        # don't use 'Q', 'R', 'S', or 'T'  (they're the arrow keys)
        #
        while True:

            #if self.use_drone == False: # if no drone, we get our own images
            #    #Add in non drone control.
            # get the next keypress
            c = chr(cv.WaitKey())

            # handle the keypress to quit...
            if c == 'q' or c == chr(27): # the Esc key is 27
                self.state = "Keyboard" # stop the FSM
                self.send( "land" ) # lands and halts
                rospy.sleep(1.42) # wait a bit for everything to finish...
                # it's important that the FSM not re-schedule itself further than
                # 1 second in the future, so that a thread won't be left running
                # after this Python program quits right here:
                return

            if c == 'f':  # capital-F starts and stops the finite-state machine
                # only run it when it's not already running!
                if self.state == "Keyboard":  # it's not yet running, so
                    print "Starting state machine..."
                    self.state = "takeoff"  # these two lines start FSM1()
                    self.FSM1()
                # if it's already running, stop it!
                else:  # if it's in any other state, we bring it back to "Keyboard"
                    self.state = "Keyboard" # should stop the state machine
                    print "Stopping state machine..."

            if c == ',':
                self.saw_capital_c = True


            # here are the keyboard commands for the drone...
            #
            # ** NEW THIS WEEK **   you must hit the space bar to stop a motion
            #
            # that is, the space bar hovers; it no longer automatically hovers...

            helistr= ""
            if c == '\n':
                self.state = "keyboard"
                helistr = "land"
            elif c == 't': 
                helistr = "takeoff"
            elif c == ' ':  # the space bar will hover
                helistr = "heli 0 0 0 0 0"
            elif self.airborne:
                if char == ord('w'):
                    sflag = 1
                    stheta = -maxpower
                elif char == ord('x'):
                    sflag = 1
                    stheta = maxpower
                elif char == ord('a'):
                    sflag = 1
                    sphi = -maxpower  
                elif char == ord('d'):
                    sflag = 1
                    sphi = maxpower  
                elif char == ord('e'):
                    sflag = 1
                    stheta = -math.sqrt(maxpower/2)
                    sphi = math.sqrt(maxpower/2)  
                elif char == ord('z'):
                    sflag = 1
                    stheta = math.sqrt(maxpower/2)
                    sphi = -math.sqrt(maxpower/2)  
                elif char == ord('q'):
                    sflag = 1
                    stheta = -math.sqrt(maxpower/2)
                    sphi = -math.sqrt(maxpower/2)  
                elif char == ord('c'):
                    sflag = 1
                    stheta = math.sqrt(maxpower/2)
                    sphi = math.sqrt(maxpower/2)  
                elif char == ord('s'):
                    self.send("heli 1 0 0 0 0")
                    rospy.sleep(.05)
                    sflag = 0
                else:
                    self.send(self.lastsent)
                
                helistr = makeHeliStr(sflag,sphi,stheta,sgaz,syaw)
            if not (helistr == ""):
                self.send(helistr)


            # if self.use_drone == False (simulated mode)
            # the arrow keys allow you to change the images you see...
            if not self.use_drone:
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

    def send(self,command):
        self.lastsent = command
        self.heli(command)    
    
if __name__ == '__main__':
    '''
    Main driver function for the drone.
    '''
    # Setup stuff
    rospy.init_node('dronekinect')  # names this ROS node

    use_drone = USE_DRONE
    controller = DroneController(use_drone)

    # Display initial message
    print 'Keyboard controls:\n'
    print 't takes off'
    print '\\n (return/enter) lands the drone'
    print 'space bar hovers (stops the drone\'s motion)\n'
    print 'The qweasdzxc keys translate at a fixed height and orientation'
    print 'i/k move the drone up/down'
    print 'j/l turn the drone left/right\n'
    print 'q or esc lands the drone and quits the program.'

    rospy.sleep(1)
    
    # start the main loop, the keyboard thread:
    controller.keyboardLoop()
    
    print "the Python program is quitting..."
       
if __name__ == "__main__":
  main()
