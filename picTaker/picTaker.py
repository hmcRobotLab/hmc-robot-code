import roslib; roslib.load_manifest('ardrone_mudd') # hmm..
roslib.load_manifest('irobot_mudd') # hmm..
import rospy
from sensor_msgs.msg import Image
from irobot_mudd.srv import *
from irobot_mudd.msg import *
import cv,cv_bridge,math,os

# base/x-y-z/hour.png
IMAGE_SOURCE = "droneImage"
SENSOR_DATA  = "sensorPacket"
TANK_SRV = 'tank'
HOURS=24
THETA_THRES = 1.5 
DIST_THRES = .03
X_INC = .55
Y_INC = .6

class picTaker():
    def __init__(self,basePath):
        self.x=0
        self.y=0
        self.z=1
        self.rx = 0.0
        self.ry = 0.0
        self.rth = 0.0
        self.hour=0  #up to 23?
        self.basePath = basePath
        self.image = 0
        self.bridge = cv_bridge.CvBridge()
        print "Connecting to irobot control"
        rospy.init_node("picTaker")
        rospy.wait_for_service(TANK_SRV)
        self.tank = rospy.ServiceProxy(TANK_SRV,Tank)
        print "Connecting to video service"
        rospy.wait_for_message(IMAGE_SOURCE, Image)
        rospy.Subscriber(IMAGE_SOURCE, Image, self.videoCb, queue_size = 1)
        rospy.sleep(5)
        print "Connecting to sensor data"
        rospy.Subscriber(SENSOR_DATA, SensorPacket, self.sensorCb)
        print "Connected"

    
    def takePic(self):
        path = "%s/%i_%i_%i/" % (self.basePath,self.x,self.y,self.z)
        try:
            os.makedirs(path)
        except:
            pass
        #cv.SaveImage("%s/%i-%i-%i/%-i.png" % \
        #        (self.basePath,self.x,self.y,self.z,self.hour), \
        #        self.image)
        cv.SaveImage("%s%i.png" % \
                (path,self.hour), \
                self.image)

    def videoCb(self,data):
        self.image = self.bridge.imgmsg_to_cv(data, "bgr8")

    def sensorCb(self,data):
        self.rx = data.x
        self.ry = data.y
        self.rth = math.degrees(data.theta) % 360

    def diffAng(self,target):
        target %= 360
        diffAng = target - self.rth
        if diffAng > 180:
            diffAng %= -360
        elif diffAng < -180:
            diffAng %= 360
        return diffAng

    def goToTheta(self,targetTheta):
        targetTheta %= 360
        diffAng = self.diffAng(targetTheta)

        # Turn right
        if diffAng < 0:
            while diffAng < -THETA_THRES:
                diffAng = targetTheta - self.rth
                if diffAng > 180:
                    diffAng %= -360
                elif diffAng < -180:
                    diffAng %= 360
                speed = 30 + (abs(diffAng) *2)
                self.tank(speed,-speed)

        # Turn left
        else:
            while diffAng > THETA_THRES:
                diffAng = targetTheta - self.rth
                if diffAng > 180:
                    diffAng %= -360
                elif diffAng < -180:
                    diffAng %= 360
                speed = 30 + (abs(diffAng)/2)
                self.tank(-speed,speed)


        self.tank(0,0)
        print "done " + str(self.rth)
        if abs(diffAng) > THETA_THRES:
	    self.goToTheta(targetTheta)

    def takeCirclePics(self):
        for x in range(HOURS):
            print "\t taking hour = " + str(x)
            self.goToTheta(x * 360/HOURS)
            if x == 0:
                raw_input("set to %i %i" % (self.x,self.y))
            self.hour = x
            self.takePic()

    def takePics(self,numX,numY):
        ascend = True
        for y in range(numY):
            if ascend:
                ascend = not ascend
                for x in range(numX):
                    print "Taking %i, %i" % (x,y)
                    self.x = x
                    self.y = y
                    self.goToXY(x * X_INC,y * Y_INC)
                    self.takeCirclePics()
            else:
                ascend = not ascend
                for x in reversed(range(numX)):
                    print "Taking %i, %i" % (x,y)
                    self.x = x
                    self.y = y
                    self.goToXY(x * X_INC,y * Y_INC)
                    self.takeCirclePics()

    def goToXY(self,tarX,tarY):
        dist = math.sqrt( (self.rx - tarX)**2 + (self.ry - tarY)**2 )
        targetAng = math.degrees(math.atan2(tarY - self.ry,tarX - self.rx))
        print "targetAng " + str(targetAng)
        self.goToTheta(math.degrees(targetAng))
        while dist > DIST_THRES:
            dist = math.sqrt( (self.rx - tarX)**2 + (self.ry - tarY)**2 )
            targetAng = math.degrees(math.atan2(tarY - self.ry,tarX - self.rx))
            diff = self.diffAng(targetAng)


            #print "targetAng " + str(targetAng)
            #print "SelfAng " + str(self.rth)
            #print "diff: %s"%diff
            #print "dist: %s, threshold: %s" % (dist, DIST_THRES)

            avgSpeed=100
            mult = 5
            self.tank(avgSpeed-(mult*diff),avgSpeed+(mult*diff))
            #print "%f %f" % (avgSpeed+(mult*diff),avgSpeed-(mult*diff))
        self.tank(0,0)

        #dist = math.sqrt( (self.rx - tarX)**2 + (self.ry - tarY)**2 )
        #while dist > DIST_THRES:
        #    dist = math.sqrt( (self.rx - tarX)**2 + (self.ry - tarY)**2 )
        #    targetAng = math.atan2(self.ry-tarY,self.rx-tarX)
        #    diffAng = targetAng - math.radians(self.rth)

        #    i = 1
        #    #if abs(diffAng) > math.pi/2:
        #    #    i=-1
        #    #    if diffAng > 0:
        #    #        diffAng = math.pi - diffAng
        #    #    else:
        #    #        diffAng = -math.pi - diffAng
        #    if dist > .75:
        #        avgSpeed = i * 300
        #        mult = i * 50
        #    elif dist > .25:
        #        avgSpeed = i * 100
        #        mult = i * 75
        #    else:
        #        avgSpeed = i*50
        #        mult = i*.90
        #    self.tank(avgSpeed-(mult*diffAng),avgSpeed+(mult*diffAng))
        #self.tank(0,0)
        #if dist > DIST_THRES:
        #    self.goToXY(tarX,tarY)

if __name__=="__main__":
    pt = picTaker("pics")
    pt.takePics(3,2)
