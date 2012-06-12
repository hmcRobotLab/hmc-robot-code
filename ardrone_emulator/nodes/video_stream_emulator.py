import roslib; roslib.load_manifest('ardrone_emulator')

import rospy
import sys, time
import cv
from math import *
from copy import deepcopy

from ardrone_emulator.srv import *
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class VisionEmulator:

    def __init__(self, imageDir, xMax, yMax, zMax, imScale, imagePubName, numHours = 12, debug = False):

        #Some constants:
        self.debug          = debug
        self.xMax           = int(xMax)
        self.yMax           = int(yMax)
        self.zMax           = int(zMax)
        self.imScale        = float(imScale)
        self.numHours       = int(numHours)
	self.rect           = (0,0,1,1)

	#Text constants
	self.textPos1       = (20,100)#(self.size[0]/4,self.size[1]/4)
	self.textPos2       = (20,140)
	self.textPos3       = (20,180)
	self.textColor      = cv.CV_RGB(255,255,255)
	self.font           = cv.InitFont(cv.CV_FONT_HERSHEY_COMPLEX, 1.0, 1.0, 0, 2)

        self.baseImageDir = imageDir

        print "Broadcasting publisher '%s'" % imagePubName
        self.imagePublisher = rospy.Publisher(imagePubName, Image)
        self.bridge = CvBridge()

        self.createSizedImage()
        self.publishImage()

    def createSizedImage(self):
        baseIm           = cv.LoadImageM('%s/0_0_1/0.png'%self.baseImageDir)
        self.baseSize    = cv.GetSize(baseIm)
	self.size        = map(lambda(x) : int(x+self.imScale*x), self.baseSize) 
	self.centerRect  = (int(self.imScale/2 * self.baseSize[0]), int(self.imScale/2 * self.baseSize[1]), self.baseSize[0],self.baseSize[1])
	self.baseIm	 = cv.CreateImage(self.size, 8, 3)
        self.landedImage = cv.CreateImage(self.size, 8, 3)
        self.rangeImage  = cv.CreateImage(self.size, 8, 3)
	self.blackImage  = cv.CreateImage(self.baseSize, 8, 3)
	self.finalImage  = cv.CreateImage(self.baseSize, 8, 3)

    def updateRangeImage(self,location):
	self.text1 = "I am out of range"
	self.text2 = "Pos: [%.2f, %.2f, %.2f]"%(self.location[1],self.location[0],self.location[2])
	self.text3 = "Max: [%i, %i, %i]"%(self.xMax,self.yMax,self.zMax)
	cv.PutText(self.rangeImage, self.text1, self.textPos1, self.font, self.textColor)
	cv.PutText(self.rangeImage, self.text2, self.textPos2, self.font, self.textColor)
	cv.PutText(self.rangeImage, self.text3, self.textPos3, self.font, self.textColor)

    def updateLandedImage(self,location):
	self.text1 = "I have landed"
	self.text2 = "Pos: [%.2f, %.2f, %.2f]"%(self.location[1],self.location[0],self.location[2])
	cv.PutText(self.landedImage, self.text1, self.textPos1, self.font, self.textColor)
	cv.PutText(self.landedImage, self.text2, self.textPos2, self.font, self.textColor)

    def publishImage(self,location):
        #First, here I need to convert x, y, z, and the robots rotation
        # into the appropriate images. The modding shouldn't be necessary, but 
        # it seems to be. I plan to investigate later. 
        base_x      = int(round(location[1]/self.imScale))
        base_y      = int(round(location[0]/self.imScale))
        base_z      = int(round(location[2]/self.imScale))
        imHour      = int(round(location[3]*(self.numHours/(2*pi)))) % self.numHours
        addTxt      = False

	if base_z == 0:
	    self.updateLandedImage()
            image   = self.landedImage
        elif (base_x > self.xMax) or (base_x < 0) or (base_y < 0) or (base_y > self.yMax) or (base_z < 1) or (base_z > self.zMax):
	    self.updateRangeImage()
            image   = self.rangeImage
        else:
            folder  = '%s/%i_%i_%i/%i.png' %(self.baseImageDir, base_x, base_y, base_z,imHour)
            image   = cv.LoadImageM(folder)
	
	im_width    = self.baseSize[0]
	im_height   = self.baseSize[1]
	pot_width_x = self.size[0] - self.baseSize[0]
	pot_width_z = self.size[1] - self.baseSize[1]
	# Z is easy
	im_z        = int(pot_width_z * (self.location[2]/self.imScale - (base_z - .5)))
	# X is complicated
	temp_x 	    = location[1]/self.imScale - base_x
	temp_y      = location[0]/self.imScale - base_y
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

	#Publish only the center rectangle.
	cv.SetImageROI(self.baseIm, self.centerRect)
	cv.Copy(self.baseIm, self.finalImage)
	cv.ResetImageROI(self.baseIm)
        self.imagePublisher.publish(self.bridge.cv_to_imgmsg(self.finalImage,"bgr8"))
