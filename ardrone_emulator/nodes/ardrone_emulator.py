import roslib; roslib.load_manifest('ardrone_emulator')

import rospy
import sys, time
import cv

from ardrone_emulator.msg import NavData
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class DroneEmulator:

    def __init__(self, imageDir, xMax, yMax, zMax, imScale, numHours = 12, location = (0,0), image_hour = 0, delay_time = 1):

        #Right now this is totally implausible as we do everything discretely, so we say that a command to go forward,
        # regardless of speed, sends the robot forward at a rate of imScale/delayTime
        self.delayTime = delay_time

        self.xMax = xMax
        self.yMax = yMax
        self.zMax = zMax,
        self.imScale = imScale
        self.numHours = numHours

        # variables for the off-board images
        self.location = location # the starting location (always a tuple)
        #self.location = ( random.choice([1,2,0]), random.randint(0,6) )

        self.image_hour = image_hour # heading at 3 o'clock == toward the markers

        self.vel = (0,0,0,0)

        self.lastUpdateTime = time.time() # last time we updated our image
        self.baseImageDir = imageDir

        self.navPublisher = rospy.Publisher('navData', NavData)
        self.imagePublisher = rospy.Publisher('droneImage', Image)

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

    def folderName(self):
        x = self.location[0]
        y = self.location[1]
        z = self.location[2]
        if (x > self.xMax) or (y > self.yMax) or (z > self.zMax):
            return 'outOfRange.png'
        else:
            return '%s/%i-%i-%i/%i.png' %(self.baseImageDir, x, y, z,self.imageHour)

    def publishImage(self):
        self.image = cv.LoadImageM(self.folderName())
        self.imagePublisher.publish(self.bridge.cv_to_imgmsm(self.image,"bgr8"))

    def publishNavData(self):
        z = self.location[2]
        if self.imageHour > self.numHours/2.0:
            imHour = self.imageHour - self.numHours
        else:
            imHour = self.imageHour
        self.navData.psi = imHour/(self.numHours) * 360 * 1000 #The real navData reports in degrees times one thousand.
        self.navData.altitude = z*self.imScale
        self.navData.vx = self.vel[0]
        self.navData.vy = self.vel[1]
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
    rospy.init_node('ardrone_emulator')

    emulator = DroneEmulator(*argTuple)

    service = rospy.Service('droneControl', DroneControl, emulator.updateCommand)

    emulator.mainLoop()

if __name__ == "__main__":
    if (len(sys.argv) < 6) or (len(sys.argv) > 9):
        print usage()
        sys.exit(1)
    else:
        argTuple = tuple(sys.argv[1:])
    main(argTuple)
