import roslib; roslib.load_manifest('ardrone_mudd') # hmm..
roslib.load_manifest('irobot_mudd') # hmm..
import rospy
from sensor_msgs.msg import Image
from irobot_mudd.srv import *
from irobot_mudd.msg import *
import cv,cv_bridge,math

# base/x-y-z/hour.png
IMAGE_SOURCE = "/camera/image"
SENSOR_DATA  = "sensorPacket"
TANK_SRV = 'tank'
HOURS=24
THETA_THRES = 3.0 
DIST_THRES = .03
TURN_SPEED = 100
X_INC = .4
Y_INC = .4

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
        rospy.Subscriber(IMAGE_SOURCE, Image, self.videoCb, queue_size = 1)
        print "Connecting to sensor data"
        rospy.Subscriber(SENSOR_DATA, SensorPacket, self.sensorCb)
        print "Connected"

    
    def takePic(self):
        #cv.SaveImage("%s/%i-%i-%i/%-i.png" % \
        #        (self.basePath,self.x,self.y,self.z,self.hour), \
        #        self.image)
        pass

    def videoCb(self,data):
        self.image = self.bridge.imgmsg_to_cv(data, "bgr8")

    def sensorCb(self,data):
        self.rx = data.x
        self.ry = data.y
        theta = math.degrees(data.theta) % 360
        if theta > 180:
            self.rth = theta - 360
        else:
            self.rth = theta

    def goToTheta(self,targetTheta):
        while abs(self.rth - targetTheta) > THETA_THRES:
            speed = (self.rth-targetTheta) * 2 + math.copysign(30,self.rth-targetTheta)
            self.tank(speed,-speed)
            #if self.rth > targetTheta:
            #    self.tank(TURN_SPEED,-TURN_SPEED)
            #else:
            #    self.tank(-TURN_SPEED,TURN_SPEED)
        self.tank(0,0)
        rospy.sleep(.25)
        if abs(self.rth - targetTheta) > THETA_THRES:
            self.goToTheta(targetTheta)

    def takeCirclePics(self):
        for x in range(HOURS):
            print "\t taking hour = " + str(x)
            self.goToTheta(x * 360/HOURS)
            self.hour = x
            self.takePic()

    def takePics(self,numX,numY):
        for y in range(numY):
            for x in range(numX):
                print "Taking %i, %i" % (x,y)
                self.x = x
                self.y = y
                self.goToXY(x * X_INC,y * Y_INC)
                #self.takeCirclePics()

    def goToXY(self,tarX,tarY):
        dist = math.sqrt( (self.rx - tarX)**2 + (self.ry - tarY)**2 )
        targetAng = math.atan2(tarY - self.ry,tarX - self.rx)
        self.goToTheta(math.degrees(targetAng))
        while dist > DIST_THRES:
            dist = math.sqrt( (self.rx - tarX)**2 + (self.ry - tarY)**2 )
            targetAng = math.atan2(tarY - self.ry,tarX - self.rx)
            diffAng = (targetAng - math.radians(self.rth))


            print "targetAng " + str(targetAng)
            print "SelfAng " + str(self.rth)
            print "diff: " + str(diffAng)
            print "dist: " + str(dist)

            avgSpeed=100
            mult = 200
            self.tank(avgSpeed-(mult*diffAng),avgSpeed+(mult*diffAng))
            print "%f %f" % (avgSpeed-(mult*diffAng),avgSpeed+(mult*diffAng))
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

