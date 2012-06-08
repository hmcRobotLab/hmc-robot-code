import roslib; roslib.load_manifest('ardrone_emulator')

import rospy
import sys, time
import cv
from math import *

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

        #Some constants: 
        self.takeoffHeight = 1
        self.xMax = int(xMax)
        self.yMax = int(yMax)
        self.zMax = int(zMax)
        self.imScale = float(imScale)
        self.numHours = int(numHours)
        self.oldTime = time.time()

        #Setting the initial state:
        location = map(int, [locX, locY, locZ]).append(0)
        self.location = location # the starting location Format: [x, y, z, 0 rad]
        self.initLocation = location

        self.internalVel = [0,0,0,0] #Format: [y velocity, x velocity, z velocity, spin velocity]
        self.groundVel = [0,0,0,0] #Format: see above
        self.landed = True

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

    def formGroundVel():
        #First just set them equal to copy the z velocity and spin velocity
        self.groundVel = self.internalVel
        #Now modify x and y appropriately.
        xyVel = sqrt(self.internalVel[0]**2 + self.internalVel[1]**2)
        xyAng = atan(self.internalVel[0]/self.internalVel[1]) + self.location[3]
        self.groundVel[0] = xyVel*sin(xyAng)
        self.groundVel[1] = xyVel*cos(xyAng)
        print "Ground veolcity set to"+self.groundVel

    def updateCommand(self, data):
        heliStr = data.command
        commands = heliStr.split()
        command = commands[0]
        commands = map(float,commands[1:])
        toReturn = True

        if command == 'reset':
            self.reset()
        elif command == 'land':
            self.land()
        elif self.landed:
            if command == 'takeoff':
                self.takeoff()
            else:
                unrecognizedCommand(heliStr)
                toReturn = False
        else:
            if command == 'heli':
                for i in xrange(4):
                    self.internalVel[i] = -commands[1+i]
                print "Setting internal velocity to:", self.internalVel
                self.formGroundVel()
            else:
                unrecognizedCommand(heliStr)
                toReturn = False

        return ControlResponse(toReturn)

    def takeoff(self):
        self.landed = False
        self.internalVel = [0,0,0,0]
        self.groundVel = [0,0,0,0]
        self.location[2] = self.takeoffHeight

    def land(self):
        self.landed = True
        self.internalVel = [0,0,0,0]
        self.groundVel = [0,0,0,0]
        self.location[2] = 0

    def reset(self):
        self.land()
        self.location = self.initLocation

    def createSizedImage(self):
        baseIm           = cv.LoadImageM('%s/0_0_0/0.png'%self.baseImageDir)
        size             = cv.GetSize(baseIm)
        self.landedImage = cv.CreateImage(size, 8,3)
        self.rangeImage  = self.landedImage

    def publishImage(self):
        #First, here I need to convert x, y, z, and the robots rotation
        # into the appropriate images. 
        x      = int(round(self.location[1]/self.imScale))
        y      = int(round(self.location[0]/self.imScale))
        z      = int(round(self.location[2]/self.imScale))
        imHour = int(round(self.location[3]*(self.numHours/(2*pi))))
        if self.landed:
            self.image = self.landedImage
        elif (x > self.xMax) or (x < 0) or (y < 0) or (y > self.yMax) or (z < 0) or (z > self.zMax):
            self.image = self.rangeImage
        else:
            folder     = '%s/%i_%i_%i/%i.png' %(self.baseImageDir, x, y, z,imHour)
            self.image = cv.LoadImageM(folder)

        self.imagePublisher.publish(self.bridge.cv_to_imgmsg(self.image,"bgr8"))

    def publishNavData(self):
        z   = self.location[2]
        rot = self.location[3]
        if rot > pi:
            rot += -2*pi

        #The real navData reports in degrees times one thousand.
        self.navData.psi      = rot/(2*pi) * 360 * 1000
        self.navData.altitude = z
        self.navData.vx       = self.internalVel[1]
        self.navData.vy       = self.internalVel[0]
        self.navData.vz       = self.internalVel[2]
        self.navPublisher.publish(self.navData)

    def hovering(self):
        return (self.internalVel == [0,0,0,0])

    def mainLoop(self):
        while True:
            dt = time.time() - oldTime
            for i in xrange(4):
                self.location[i] += self.groundVel[i]*dt

            #Now we correct for potentially having an angle bigger than 2pi
            while self.location[3] > 2*pi:
                self.location[3] -= 2*pi

            #Now re-establish oldTime
            oldTime = time.time()

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
