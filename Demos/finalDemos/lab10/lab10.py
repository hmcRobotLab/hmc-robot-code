import ardrone2
from std_msgs.msg import String
from ardrone2 import rospy

class myArdrone(ardrone2.Ardrone):
    def __init__(self):
      ardrone2.Ardrone.__init__(self)
      self.state           = "keyboard"
      self.boxX            = 0
      self.boxHeight       = 0
      self.tarHeight       = .20
      self.tarX            = .46
      self.toleranceX      = 40/320.0
      self.toleranceHeight = .03
      self.toleranceArea   = .001
      self.noBox           = True
      self.spinPower       = .4
      self.forwardPower    = .4

    def loop(self):
      while True:
        char = self.getKeyPress()
        if char == 'n':
          self.state = "start"
          self.fsm()

    def fsm(self,timer_event=None):
      print self.state + " " + str(self.boxX) + " " + str(self.boxHeight)
      if self.state == "keyboard":
        return
      elif self.state == "start":
        self.takeoff()
        self.state = "searching"
      elif self.state == "searching":
        if self.noBox:
          self.spinLeft(self.spinPower)
        else:
          self.state = "closing_in"
      elif self.state == "closing_in":
        if abs(self.boxX - self.tarX) < self.toleranceX:
          self.state = "approaching"
        elif self.boxX > self.tarX:
          print "right"
          self.spinRight(self.spinPower/2)
        else:
          print "left"
          self.spinLeft(self.spinPower/2)
      elif self.state == "approaching":
        if self.noBox:
          self.state = "searching"
        elif abs(self.boxX - self.tarX) > self.toleranceX:
          self.state = "closing_in"
        elif self.tarHeight - self.boxHeight > self.toleranceHeight:
          self.forward(self.forwardPower)
        else:
          self.state = "landing"
      elif self.state == "landing":
        self.hover()
        rospy.sleep(.25)
        self.land()
        self.state = "keyboard"

      rospy.Timer( rospy.Duration(0.01), self.fsm, oneshot=True )

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
          self.noBox = True
          self.boxX = float("inf")
          self.boxHeight = 0
        else:
          self.noBox = False
          self.boxX = centerX
          self.boxHeight = height

if __name__== "__main__":
  drone = myArdrone()
  print "\r Connecting to bounding box data"
  rospy.Subscriber("imageData",String,drone.bBoxUpdate,queue_size=1)
  print "Ready"
  drone.loop()
