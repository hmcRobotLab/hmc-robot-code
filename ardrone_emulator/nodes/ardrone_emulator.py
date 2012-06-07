import roslib; roslib.load_manifest('ardrone_emulator')

import rospy
import sys, time
import cv

from ardrone_emulator.msg import NavData
from ardrone_emulator.srv import DroneControl
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def unrecognizedCommand(heliStr):
    print "I didn't recognize the command:"
    print heliStr

def usage():
    print "Ardrone Emulator:"
    print "Please specify the image directory, the maximum x, y, and z \
coordinates, the distance scale between your images, and, optionally the \
number of angles at which you took images for each position in your grid, the starting x,y,or z, and the delay time"
    print "Example:"
    print "python <path>/ardrone_emulator.py ~/summer2012/data/droneImages 6 2 0 1 1 12 1 3 0 0 1"

class DroneEmulator:

    def __init__(self, imageDir, xMax, yMax, zMax, imScale, numHours = 12, locX=0, locY=0, locZ=0, image_hour = 0, delay_time = 1):

        #Right now this is totally implausible as we do everything discretely, so we say that a command to go forward,
        # regardless of speed, sends the robot forward at a rate of imScale/delayTime
        self.delayTime = delay_time

        self.xMax = int(xMax)
        self.yMax = int(yMax)
        self.zMax = int(zMax)
        self.imScale = float(imScale)
        self.numHours = float(numHours)

        #Setting the initial state:
        location = map(int, (locX, locY, locZ))
        self.location = location # the starting location Format: (x, y, z)
        self.initLocation = location
        self.imageHour = int(image_hour) # heading at 3 o'clock == toward the markers
        self.initHour = self.imageHour
        self.vel = (0,0,0,0) #Format: (y velocity, x velocity, z velocity, spin velocity)
        self.landed = True

        self.lastUpdateTime = time.time() # last time we updated our image
        self.baseImageDir = imageDir

        print "Broadcasting publishers 'navData' and 'droneImage'"
        self.navPublisher = rospy.Publisher('navData', NavData)
        self.imagePublisher = rospy.Publisher('droneImage', Image)

        self.bridge = CvBridge()

        self.publishImage()

        #Setting up the navdata message
        self.navData = NavData()

        #These parameters never change
        self.navData.batLevel = 100
        self.navData.tag = 0
        self.navData.size = 0
        self.navData.ctrlState = 0
        self.navData.numFrames = 0
        self.navData.theta = 0 #This part is not realistic. If we wanted too, we could record a fake theta based on speed.
        self.navData.phi = 0 #Likewise not realistic.

        #Now we get the variable ones and publish it. 
        self.publishNavData()

    def updateCommand(self, heliStr):
        commands = heliStr.split()
        if len(commands) != 1 or len(commands) != 6:
            unrecognizedCommand(heliStr)

        if commands[0] == 'reset':
            self.reset()
        elif commands[0] == 'land':
            self.land()

        if self.landed:
            if commands[0] == 'takeoff':
                self.landed = False
                return True
            else:
                unrecognizedCommand(heliStr)
                return False
        else:
            if commands[0] == 'heli':
                self.vel = tuple(map(lambda(x) : -copysign(1,float(x)), commands[2:]))
            else:
                unrecognizedCommand(heliStr)
                return False

    def land(self):
        self.landed = True
        self.vel = (0,0,0,0)

    def reset(self):
        self.land()
        self.location = self.initLocation
        self.imageHour = self.initHour

    def folderName(self):
        x = self.location[1]
        y = self.location[0]
        z = self.location[2]
        if self.landed:
            return 'landed.png'
        elif (x > self.xMax) or (y > self.yMax) or (z > self.zMax):
            return 'outOfRange.png'
        else:
            return '%s/%i-%i-%i/%i.png' %(self.baseImageDir, x, y, z,self.imageHour)

    def publishImage(self):
        self.image = cv.LoadImageM(self.folderName())
        self.imagePublisher.publish(self.bridge.cv_to_imgmsg(self.image,"bgr8"))

    def publishNavData(self):
        z = self.location[2]
        if self.imageHour > self.numHours/2.0:
            imHour = self.imageHour - self.numHours
        else:
            imHour = self.imageHour
        self.navData.psi = imHour/(self.numHours) * 360 * 1000 #The real navData reports in degrees times one thousand.
        self.navData.altitude = z*self.imScale
        self.navData.vx = self.vel[1]
        self.navData.vy = self.vel[0]
        self.navData.vz = self.vel[2]
        self.navPublisher.publish(self.navData)

    def hovering(self):
        return (self.vel == (0,0,0,0))

    def mainLoop(self):
        while True:
            if (self.hovering):
                self.lastUpdateTime = time.time()
            elif (time.time() - self.delayTime > self.lastUpdateTime):
                for i in xrange(3):
                    if self.vel[i] != 0:
                        self.location[i] += math.copysign(1, self.vel[i])
                if self.vel[3] != 0:
                    self.imageHour += (math.copysign(1,self.vel[3]) % self.numHours)
                self.lastUpdateTime = time.time()
            #Now to publish the data like real drone.
            self.publishImage()
            self.publishNavData()

def main(argTuple):
    print "Initializing node: ardrone_emulator"
    rospy.init_node('ardrone_emulator')
    print "Starting emulator"
    emulator = DroneEmulator(*argTuple)
    print "Registering service 'droneControl'"
    service = rospy.Service('droneControl', DroneControl, emulator.updateCommand)
    emulator.mainLoop()

if __name__ == "__main__":
    if (len(sys.argv) < 6) or (len(sys.argv) > 9):
        print usage()
        sys.exit(1)
    else:
        argTuple = tuple(sys.argv[1:])
    main(argTuple)
