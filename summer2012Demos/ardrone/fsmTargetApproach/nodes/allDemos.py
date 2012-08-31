#!/usr/bin/env python

from roslib.packages import get_pkg_dir
import sys
from math import tan, radians

sys.path = [get_pkg_dir('ardrone2_mudd')+"/nodes"]+sys.path

import ardrone2
from ardrone2 import rospy
from std_msgs.msg import String

class myArdrone(ardrone2.Ardrone):
    def __init__(self):
      ardrone2.Ardrone.__init__(self)
      self.state           = "keyboard"
      self.xyApprData      = {"tarX": 0, "tolX": 40/320.0, \
                              "tarHeight": 0.23, "tolHeight": 0.03, \
                              "spinPower": 0.4, "forwardPower": 0.25 \
                             }
      self.xyzCenLandData  = {"tarX": 0, "tolX": 50/320.0, \
                              "tarY": 0, "tolY": 50/320.0, \
                              "landingArea": 0.6, \
                              "zPower": 0.4, "yPower": 0.09, \
                              "xPower": 0.09\
                             }
      self.boxX            = float("inf")
      self.boxY            = float("inf")
      self.boxHeight       = 0
      self.toleranceArea   = .001
      self.noBox           = True

    def loop(self):
      while not rospy.is_shutdown():
        char = self.getKeyPress(1000)
        if char == 'n':
          self.state = "start"
          self.xyAppr()
        elif char == 'm':
          self.state = "start"
          self.setCam(1)
          self.xyzCenLand()
        elif char == ' ':
          self.state = "keyboard"

    def xyzCenLand(self, timer_event=None):
      """ the finite-state machine that centers using the downward camera and then lands"""
      data = self.xyzCenLandData
      print self.state
      if self.state == "keyboard":
        # user stopped our state machine!
        self.land()
        return
      elif self.state == "start":
        self.takeoff()
        self.state = "scanning"
      elif self.state == "scanning":
        if self.noBox:
          print "ascending"
          self.up(data["zPower"])
        else:
          self.state = "yCenter"
      elif self.state == "yCenter":
        if abs(data["tarY"]-self.boxY) < data["tolY"]:
          # Y is centered, now do X
          self.state = "xCenter"
        elif self.boxY < data["tarY"]:
          print "Going forward"
          self.forward(data["yPower"])
          self.state = "scanning"
        else:
          print "Going backwards"
          self.backward(data["yPower"])
          self.state = "scanning"
      elif self.state == "xCenter":
        if abs(data["tarX"]-self.boxX)< data["tolX"]:
          self.state = "centered"
        elif self.boxX < data["tarX"]:
          print "Going left"
          self.strafeLeft(data["xPower"])
          self.state = "scanning"
        else:
          print "Going right"
          self.strafeRight(data["xPower"])
          self.state = "scanning"
      elif self.state == "centered":
        if self.area > data["landingArea"]:
          print self.area
          self.hover()
          self.land()
          self.state = "keyboard"
        else:
          print "Going Down, current area: %f" % self.area
          self.down(data["zPower"])
          rospy.sleep(0.2)
          self.state = "scanning"
      rospy.Timer( rospy.Duration(0.01), self.xyzCenLand, oneshot=True )

    def xyAppr(self,timer_event=None):
      """ the finite state machine that finds the landmark in the xy-plane
          and approaches it, then lands """
      data = self.xyApprData
      print self.state + " " + str(self.boxX) + " " + str(self.boxHeight)
      if self.state == "keyboard":
        return
      elif self.state == "start":
        #self.takeoff()
        self.state = "searching"
      elif self.state == "searching":
        if self.noBox:
          self.spinLeft(data["spinPower"])
        else:
          self.state = "closing_in"
      elif self.state == "closing_in":
        if abs(self.boxX - data["tarX"]) < data["tolX"]:
          self.state = "approaching"
        elif self.boxX < data["tarX"]:
          print "right"
          self.spinRight(data["spinPower"]/2)
        else:
          print "left"
          self.spinLeft(data["spinPower"]/2)
      elif self.state == "approaching":
        if self.noBox:
          self.state = "searching"
        elif abs(self.boxX - data["tarX"]) > data["tolX"]:
          self.state = "closing_in"
        elif data["tarHeight"] - self.boxHeight > data["tolHeight"]:
          self.forward(data["forwardPower"])
        else:
          self.state = "landing"
      elif self.state == "landing":
        self.hover()
        rospy.sleep(.25)
        self.land()
        self.state = "keyboard"

      rospy.Timer( rospy.Duration(0.01), self.xyAppr, oneshot=True )

    def bBoxUpdate(self, data):
        #Format is: "CenterX (in percent of box width) CenterY (in percent of box height)
        #            Area (in percent of box area) LeftEdge (in percent of box width)
        #            TopEdge (in percent of box height) RightEdge (in percent of box width)
        #            BottomEdge (in percent of box height)"
        br      = data.data.split()
        centerX = float(br[0])
        centerY = float(br[1])
        area    = float(br[2])
        height  = float(br[6])-float(br[4]) #the y axis points down.
        if area < self.toleranceArea:
          print "I see a too small box of area %f"%area
          self.noBox     = True
          self.boxX      = float("inf")
          self.boxY      = float("inf")
          self.boxHeight = 0
          self.area      = 0
        else:
          self.noBox     = False
          c              =  0.5 - centerX 
          d              =  0.5 - centerY 
          # The formulas below are designed to account for the fact that as the drone tilts, the image it 
          # recieves are not actually where the drone thinks they are. 
          if self.cameraNumber == 0:
            self.boxX    = c
          elif self.cameraNumber == 1:
            self.boxX    = (tan(-self.phi)*(1-2*c) + 2*c*tan(-self.phi - self.downCamLensAng/2.0))/(2*self.downCamLensAng/2.0)
          self.boxY      = (tan(-self.theta)*(1-2*d) + 2*d*tan(-self.theta - self.downCamLensAng/2.0))/(2*self.downCamLensAng/2.0)
          self.boxHeight = height
          self.area      = area

if __name__== "__main__":
  drone = myArdrone()
  print "\r Connecting to bounding box data"
  rospy.Subscriber("imageData",String,drone.bBoxUpdate,queue_size=1)
  print "Ready"
  drone.loop()
  print "Closing..."
