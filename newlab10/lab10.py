import ardrone
from std_msgs.msg import String
from ardrone import rospy

class myArdrone(ardrone.Ardrone):
    def __init__(self):
        ardrone.Ardrone.__init__(self)
        self.state = "keyboard"
        self.boxX = 0
        self.boxHeight = 0
        self.tarHeight = 35
        self.tarX = 150

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
            if abs(self.boxX - self.tarX) < 40:
                self.state = "approaching"
            elif self.boxX < self.tarX:
                self.spinLeft(.3)
            else:
                self.spinRight(.3)
        elif self.state == "approaching":
            if abs(self.boxX - self.tarX) > 40:
                self.state = "searching"
            elif self.tarHeight - self.boxHeight > 10:
                self.forward(.07)
            else:
                self.land()
                self.state = "landing"
        elif self.state == "landing":
            self.hover()
            rospy.sleep(.25)
            self.land()
            self.state = "keyboard"

        rospy.Timer( rospy.Duration(0.01), self.fsm, oneshot=True )

    def bBoxUpdate(self, data):
        br = data.data.split()
        xl = int(br[0])
        xr = int(br[1])
        yt = int(br[2])
        yb = int(br[3])
        if (xr-xl)*(yb-yt) < 200:
            self.boxX = float("inf")
        else:
            self.boxX = (xl + xr)/2
            self.boxHeight = yb - yt #the y axis points down.

if __name__== "__main__":
  drone = myArdrone()
  print "\r Connecting to bounding box data"
  ardrone.rospy.Subscriber("imageData",String,drone.bBoxUpdate,queue_size=1)
  print "Ready"
  drone.loop()
