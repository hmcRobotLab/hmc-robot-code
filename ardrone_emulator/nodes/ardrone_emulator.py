import roslib; roslib.load_manifest('ardrone_emulator')

import rospy
import sys, time
import cv
from math import copysign

from ardrone_emulator.msg import navData
from ardrone_emulator.srv import *
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

    def __init__(self, imageDir, xMax, yMax, zMax, imScale, numHours = 12, locX=0, locY=0, locZ=0, image_hour = 0, delay_time = 1.0):

        #Right now this is totally implausible as we do everything discretely, so we say that a command to go forward,
        # regardless of speed, sends the robot forward at a rate of imScale/delayTime
        self.delayTime = delay_time

        self.xMax = int(xMax)
        self.yMax = int(yMax)
        self.zMax = int(zMax)
        self.imScale = float(imScale)
        self.numHours = int(numHours)

        #Setting the initial state:
        location = map(int, [locX, locY, locZ])
        self.location = location # the starting location Format: [x, y, z]
        self.initLocation = location
        self.imageHour = int(image_hour)
        self.initHour = self.imageHour
        self.vel = [0,0,0,0] #Format: [y velocity, x velocity, z velocity, spin velocity]
        self.landed = True

        self.lastUpdateTime = time.time() # last time we updated our image
        self.baseImageDir = imageDir

        print "Broadcasting publishers 'navData' and 'droneImage'"
        self.navPublisher = rospy.Publisher('navData', navData)
        self.imagePublisher = rospy.Publisher('droneImage', Image)
        self.bridge = CvBridge()

        self.createSizedImage()
        self.publishImage()

        #Setting up the navdata message
        self.navData = navData()

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

    def updateCommand(self, data):
        heliStr = data.command
        commands = heliStr.split()
        command = commands[0]
        commands = map(float,commands[1:])
        toReturn = False

        if command == 'reset':
            self.reset()
            toReturn = True
        elif command == 'land':
            self.land()
            toReturn = True
        elif self.landed:
            if command == 'takeoff':
                self.landed = False
                toReturn = True
            else:
                unrecognizedCommand(heliStr)
        else:
            if command == 'heli':
                for i in xrange(4):
                    if commands[1+i] == 0:
                        self.vel[i] = 0
                    else:
                        self.vel[i] = -copysign(1,commands[1+i])
                print "Setting velocity to:",self.vel
                toReturn = True
            else:
                unrecognizedCommand(heliStr)

        return ControlResponse(toReturn)

    def land(self):
        self.landed = True
        self.vel = [0,0,0,0]

    def reset(self):
        self.land()
        self.location = self.initLocation
        self.imageHour = self.initHour

    def createSizedImage(self):
        baseIm = cv.LoadImageM('%s/0_0_0/0.png'%self.baseImageDir)
        size = cv.GetSize(baseIm)
        self.landedImage = cv.CreateImage(size, 8,3)
        self.rangeImage = self.landedImage
        #self.landedImage = cv.LoadImageM('landed.png')
        #cv.Resize(self.landedImage,baseIm)
        #self.rangeImage = cv.LoadImageM('outOfRange.png')
        #cv.Resize(self.rangeImage,baseIm)

    def publishImage(self):
        x = self.location[1]
        y = self.location[0]
        z = self.location[2]
        if self.landed:
            self.image = self.landedImage
        elif (x > self.xMax) or (x < 0) or (y < 0) or (y > self.yMax) or (z < 0) or (z > self.zMax):
            self.image = self.rangeImage
        else:
            folder = '%s/%i_%i_%i/%i.png' %(self.baseImageDir, x, y, z,self.imageHour)
            self.image = cv.LoadImageM(folder)

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
        return (self.vel == [0,0,0,0])

    def mainLoop(self):
        while True:
            if (self.hovering()):
                self.lastUpdateTime = time.time()
            elif ((time.time() - self.lastUpdateTime) > self.delayTime):
                for i in xrange(3):
                    if self.vel[i] != 0:
                        self.location[i] += copysign(1, self.vel[i])
                print self.location
                if self.vel[3] != 0:
                    self.imageHour = (copysign(1,self.vel[3]) + self.imageHour) % self.numHours
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
    service = rospy.Service('droneControl', Control, emulator.updateCommand)
    emulator.mainLoop()

if __name__ == "__main__":
    if (len(sys.argv) < 6) or (len(sys.argv) > 9):
        print usage()
        sys.exit(1)
    else:
        argTuple = tuple(sys.argv[1:])
    main(argTuple)
