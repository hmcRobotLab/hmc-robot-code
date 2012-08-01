import roslib; roslib.load_manifest('ardrone_emulator')

import rospy
import sys, time
import cv
from math import *
from copy import deepcopy
import vision_emulator

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
number of angles at which you took images for each position in your grid, \
the starting x,y,or z, and the name of the navData publisher."
    print "Example:"
    print "python <path>/ardrone_emulator.py\
    /home/robotics/summer2012/data/droneImages3by2 2 1 1 0 24 1 1 0 0 navData"

class DroneEmulator:

    def __init__(self, imageDir, xMax, yMax, zMax, imScale, numHours = 12,\
            locX=0, locY=0, locZ=0, image_hour = 0, navDataPubName = "navData", debug = False):

        #Some constants:
        self.speedConstants = [1,1,1,1] #Format: [y, x, z, spin]
        self.debug          = debug
        self.takeoffHeight  = 1
	self.visionEmulator = vision_emulator.visionEmulator.__init__(imageDir, xMax, yMax, zMax, imScale, numHours, debug) 

        #Setting the initial state:
        # the starting location Format: [y, x, z, 0 rad]
        self.location = map(float, [locY, locX, locZ])
        self.location.append(0.0)
        self.initLocation = deepcopy(self.location)

        self.internalVel = [0,0,0,0] #Format: [y velocity, x velocity, z velocity, spin velocity]
        self.groundVel = [0,0,0,0] #Format: see above
        self.landed = True

        print "Broadcasting publisher '%s'" % navDataPubName
        self.navPublisher = rospy.Publisher(navDataPubName, navData)

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

    def formGroundVel(self):
        #First just set them equal to copy the z velocity and spin velocity
        self.groundVel = deepcopy(self.internalVel)
        #Now modify x and y appropriately.
        self.groundVel[0] = self.internalVel[0]*cos(self.location[3])
        self.groundVel[1] = self.internalVel[1]*cos(self.location[3])
        self.groundVel[0] += self.internalVel[1]*sin(self.location[3])
        self.groundVel[1] += self.internalVel[1]*sin(self.location[3]) # should be 0???
        if self.debug:
            print "Ground veolcity set to",self.groundVel

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
                    self.internalVel[i] = -commands[1+i]*self.speedConstants[i]
                self.internalVel[2] = self.internalVel[2]*-1
                if self.debug:
                    print "Internal velocity set to:", self.internalVel
                self.formGroundVel()
            else:
                unrecognizedCommand(heliStr)
                toReturn = False
        if self.debug:
            print "My current location is",self.location,"\n"
        return ControlResponse(toReturn)

    def takeoff(self):
        self.landed = False
        self.internalVel = [0,0,0,0]
        self.groundVel = [0,0,0,0]
        self.location[2] = self.takeoffHeight
        print "Taking off to position",self.location

    def land(self):
        self.landed = True
        self.internalVel = [0,0,0,0]
        self.groundVel = [0,0,0,0]
        self.location[2] = 0
        print "Landing at position",self.location

    def reset(self):
        self.land()
        self.location = deepcopy(self.initLocation)
        print "Resetting to position",self.location

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

    def mainLoop(self):
        oldTime = time.time()
        while True:
            dt = time.time() - oldTime
            for i in xrange(4):
                self.location[i] += self.groundVel[i]*dt

            #Now we correct for potentially having an angle outside of the
            # desired range.
            self.location[3] %= (2*pi)

            #Now re-establish oldTime
            oldTime = time.time()

            #Now to publish the data like real drone.
            self.visionEmulator.publishImage(self.location, self.landed)
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
    if (len(sys.argv) < 6) or (len(sys.argv) > 13):
        print usage()
        sys.exit(1)
    else:
        argTuple = tuple(sys.argv[1:])
    main(argTuple)
