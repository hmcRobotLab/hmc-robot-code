import roslib; roslib.load_manifest('ardrone_emulator')

import rospy
import sys, time
import cv
from math import *
from copy import deepcopy

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
    print "python <path>/ardrone_emulator.py\
    /home/robotics/summer2012/data/droneImages 2 6 1 1 12 1 3 0 0 1"

class DroneEmulator:

    def __init__(self, imageDir, xMax, yMax, zMax, imScale, numHours = 12,\
            locX=0, locY=0, locZ=0, image_hour = 0, delay_time = 1.0, debug =\
            False):

        #Some constants:
        self.speedConstants = [1,1,1,1] #Format: [y, x, z, spin]
        self.debug = debug
        self.takeoffHeight = 1
        self.xMax = int(xMax)
        self.yMax = int(yMax)
        self.zMax = int(zMax)
        self.imScale = float(imScale)
        self.numHours = int(numHours)
	self.rect = (0,0,1,1)

        #Setting the initial state:

        # the starting location Format: [y, x, z, 0 rad]
        self.location = map(float, [locY, locX, locZ])
        self.location.append(0.0)
        self.initLocation = deepcopy(self.location)

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

    def formGroundVel(self):
        #First just set them equal to copy the z velocity and spin velocity
        self.groundVel = deepcopy(self.internalVel)
        #Now modify x and y appropriately.
        self.groundVel[0] = self.internalVel[0]*cos(self.location[3])
        self.groundVel[1] = self.internalVel[1]*cos(self.location[3])
        self.groundVel[0] += self.internalVel[1]*sin(self.location[3])
        self.groundVel[1] += self.internalVel[1]*sin(self.location[3])
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

    def createSizedImage(self):
        baseIm           = cv.LoadImageM('%s/0_0_1/0.png'%self.baseImageDir)
        self.baseSize    = cv.GetSize(baseIm)
	self.size        = map(lambda(x) : int(x+self.imScale*x), self.baseSize) 
	self.baseIm	 = cv.CreateImage(self.size, 8, 3)
        self.landedImage = cv.CreateImage(self.size, 8, 3)
        self.rangeImage  = cv.CreateImage(self.size, 8, 3)
	self.blackImage  = cv.CreateImage(self.baseSize, 8, 3)

    def publishImage(self):
        #First, here I need to convert x, y, z, and the robots rotation
        # into the appropriate images. The modding shouldn't be necessary, but 
        # it seems to be. I plan to investigate later. 
        base_x      = int(round(self.location[1]/self.imScale))
        base_y      = int(round(self.location[0]/self.imScale))
        base_z      = int(round(self.location[2]/self.imScale))
        imHour      = int(round(self.location[3]*(self.numHours/(2*pi)))) % self.numHours
        
	if self.landed:
            image  = self.landedImage
        elif (base_x > self.xMax) or (base_x < 0) or (base_y < 0) or (base_y > self.yMax) or (base_z < 1) or (base_z > self.zMax):
            image  = self.rangeImage
        else:
            folder = '%s/%i_%i_%i/%i.png' %(self.baseImageDir, base_x, base_y, base_z,imHour)
            image  = cv.LoadImageM(folder)
	
	im_width    = self.baseSize[0]
	im_height   = self.baseSize[1]
	pot_width_x = self.size[0] - self.baseSize[0]
	pot_width_z = self.size[1] - self.baseSize[1]
	# Z is easy
	im_z        = int(pot_width_z * (self.location[2]/self.imScale - (base_z - .5)))
	# X is complicated
	
	temp_x 	    = self.location[1]/self.imScale - base_x
	temp_y      = self.location[0]/self.imScale - base_y
	imAngle     = imHour*2*pi/self.numHours
	dx1         = temp_x * sin(imAngle)
	dx2 	    = temp_y * cos(imAngle)
	x	    = dx1 + dx2
	im_x        = int(pot_width_x * (x + .5))

	self.oldR   = self.rect
	self.rect   = (im_x,im_z,im_width,im_height)
	
	if self.debug:	
	    print "temp_x: %f, temp_y: %f"%(temp_x, temp_y)
	    print "imAngle: %f"%imAngle
	    print "dx1: %f, dx2: %f"%(dx1,dx2)
	    print "x: %f"%x
            print self.rect 

	#Delete the old image
	cv.SetImageROI(self.baseIm, self.oldR)
	cv.Resize(self.blackImage, self.baseIm)
	cv.ResetImageROI(self.baseIm)

	#Add in the new one
	cv.SetImageROI(self.baseIm, self.rect)
	cv.Resize(image, self.baseIm)
	cv.ResetImageROI(self.baseIm)

	#Publish
        self.imagePublisher.publish(self.bridge.cv_to_imgmsg(self.baseIm,"bgr8"))

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
        oldTime = time.time()
        while True:
            dt = time.time() - oldTime
            for i in xrange(4):
                self.location[i] += self.groundVel[i]*dt

            #Now we correct for potentially having an angle outside of the
            # desired range.
            self.location[3] = self.location[3] % (2*pi)

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
    if (len(sys.argv) < 6) or (len(sys.argv) > 11):
        print usage()
        sys.exit(1)
    else:
        argTuple = tuple(sys.argv[1:])
    main(argTuple)
