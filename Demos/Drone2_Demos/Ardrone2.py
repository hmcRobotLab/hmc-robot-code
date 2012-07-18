import roslib; roslib.load_manifest('ardrone2_mudd')
from ardrone2_mudd.srv import *
from ardrone2_mudd.msg import *
from std_msgs.msg import String
import rospy
import cv
import math
import sys

#############################################
####   Same as "ardrone.py", but with    ####
####   some tweaks to make it work well  ####
####   with Drone 2.0                    ####
#############################################

class Ardrone():
  def __init__(self,controlName="ardrone2/heli", navPubName="ardrone2/navData"):
    rospy.init_node("ArdroneBase")
    print "Connecting to droneControl service"
    rospy.wait_for_service(controlName)
    self.heli = rospy.ServiceProxy(controlName, Control, persistent=True)
    print "\r Connecting to navData service"
    rospy.Subscriber(navPubName, navData, self.navDataUpdate, queue_size=1)
    print "\r Connected to drone services"
    
    print """
    Keyboard Controls:
    h: quit
    t: takeoff              r: reset    enter/return(space): land
    1: use forward camera               2: use bottom camera

    q: forward left strafe  w: forward  e: forward right strafe
    a: strafe left          s: hover    d: strafe right
    z: backward left strafe x: backward e: backward right strafe

    f: spin left
    g: spin right
    v: up
    b: down
    """
    

    self.lastSent = "None"
    self.airborne = False

    # Navdata fields (partial)
    self.altitude  = 0 
    self.phi       = 0 
    self.psi       = 0 
    self.theta     = 0 
    self.vx        = 0 
    self.vy        = 0 
    self.vz        = 0 
    self.batLevel  = 0 
    self.ctrlState = 0

    # Base speed for drone.
    self.keyPower = .07

    # Reset the drone !!
    self.send(4,0,0,0,0)

    # The Opencv Windows
    cv.NamedWindow('control')
    cv.MoveWindow('control', 200, 500)

    print "Ready"

  def navDataUpdate(self,data):
    self.altitude  = data.altitude
    self.phi       = data.phi
    self.psi       = data.psi
    self.theta     = data.theta
    self.vx        = data.vx
    self.vy        = data.vy
    self.vz        = data.vz
    self.batLevel  = data.batLevel
    self.ctrlState = data.ctrlState

  def printStatus(self):
    print "\t [alt: %i] \t [phi: %i psi: %i theta: %i] \t [vx: %i vy: %i vz: %i] \t [bat: %i  state: %i]" \
      % (self.altitude,self.phi,self.psi,self.theta,self.vx,self.vy,self.vz,self.batLevel,self.ctrlState)
    print "\t Last sent: %s" % (self.lastsent)

  def send(self,flag,phi,theta,gaz,yaw):
    self.lastsent = flag,phi,theta,gaz,yaw
    self.heli(flag,phi,theta,gaz,yaw)    
    rospy.sleep(.05)

  def spinLeft(self,power):
    self.send(0,0,0,0,-power)

  def spinRight(self,power):
    self.send(0,0,0,0,power)

  def strafeRight(self,power):
    self.send(1,power,0,0,0)

  def strafeLeft(self,power):
    self.send(1,-power,0,0,0)

  def forward(self,power):
    self.send(1,0,-power,0,0)

  def backward(self,power):
    self.send(1,0,power,0,0)

  def up(self,power):
    self.send(0,0,0,power,0)

  def down(self,power):
    self.send(0,0,0,-power,0)

  def takeoff(self):
    if not airborne:
      self.airborne = True
      send("takeoff")

  def land(self):
    self.airborne = False
    send("land")
    
  def reset(self):
    self.airborne = False
    send("reset")

  def getKeyPress(self):
    char = chr(cv.WaitKey() % 255)
    self.keyCmd(char)
    return char

  def keyCmd(self,char):
    sflag = 0
    sphi = 0
    stheta = 0
    sgaz = 0
    syaw = 0
    sgaz = 0

    helistr = ''
    if char == ' ':
        self.airborne = False
        helistr = 2,0,0,0,0
        # Landing
    elif char == 'h':
      self.send(2,0,0,0,0)
      # Landing
      sys.exit(0)
    elif char == 'r':
        self.airborne = False
        helistr = 4,0,0,0,0
        # Resetting
    elif char == 't':
        self.airborne = True
        self.send(0,0,0,0,0)
        helistr = 3,0,0,0,0
        # Take Off
        # Why is the above there...why hover then take off?
    elif char == '1':
        helistr = "camera 0"
    elif char == '2':
        helistr = "camera 1"
    elif self.airborne:
        if char == 'w':
            sflag = 1
            stheta = -self.keyPower
        elif char == 'x':
            sflag = 1
            stheta = self.keyPower
        elif char == 'a':
            sflag = 1
            sphi = -self.keyPower  
        elif char == 'd':
            sflag = 1
            sphi = self.keyPower  
        elif char == 'e':
            sflag = 1
            stheta = -math.sqrt(self.keyPower/2)
            sphi = math.sqrt(self.keyPower/2)  
        elif char == 'z':
            sflag = 1
            stheta = math.sqrt(self.keyPower/2)
            sphi = -math.sqrt(self.keyPower/2)  
        elif char == 'q':
            sflag = 1
            stheta = -math.sqrt(self.keyPower/2)
            sphi = -math.sqrt(self.keyPower/2)  
        elif char == 'c':
            sflag = 1
            stheta = math.sqrt(self.keyPower/2)
            sphi = math.sqrt(self.keyPower/2)  
        elif char == 's': #Self-adjusting hover
            self.send(1,0,0,0,0)
            rospy.sleep(.05)
            sflag = 0
        elif char == '3': #Non-adjusting hover
            sflag = 1
        elif char == 'g':
            syaw = self.keyPower
        elif char == 'f':
            syaw = -self.keyPower
        elif char == 'v':
            sgaz = self.keyPower
        elif char == 'b':
            sgaz = -self.keyPower
        else:
            #self.send(self.lastsent)
            # The part above might now work because
            # The last sent could be made a tuple.
            return
          
        helistr = sflag,sphi,stheta,sgaz,syaw
        
    if not helistr == '':
      sflag, sphi, stheta, sgaz, syaw = helistr
      self.send(sflag,sphi,stheta,sgaz,syaw)

if __name__ == "__main__":
  controller = Ardrone()
  while True:
    controller.getKeyPress()

  
