from std_msgs.msg import String
import rospy

class FSM:
    def __init__(self):
        self.frontImageData = [0, 0, 0, 0]
        self.bottomImageData = [0, 0, 0, 0]
        self.cam = "Front"
        self.center = [0, 0]
        self.state = "None"
        self.publisher = rospy.Publisher('state', String)
        self.area = 0
        self.search = False
        self.airborne = False
        rospy.Subscriber('imageData', String, self.handle_image_data)
        rospy.Subscriber('GUI', String, self.handle_GUI_data)
        
        # our drone's state machine
    
    def setState(self, state):
        self.state = state


    def FSM0(self, timer_event=None):
        print "center is:", self.center
        print "area is:", self.area
        print "state is:", self.state
        self.publisher.publish("ChangeCamera 1")
        if self.state == "Keyboard": # user stopped our state machine!
            print "State machine has been stopped!"
            print "Sending hover command"
            self.publisher.publish(self.state)
            return # officially ends the state machine

        elif self.state == "Landing":
            self.publisher.publish(self.state)
            self.airborne = False
            return

        elif self.state == "Start":
            print "Starting the state machine!"
            if self.airborne == True:
                self.state = "Hover"
            else: 
                self.airborne = True
                self.state = "Takeoff"
            self.publisher.publish(self.state)
            
        elif self.state == "Takeoff":
            print "taking off in the FSM"
            rospy.sleep(3.0)  # wait for takeoff...
            self.state = "Hover"
            self.publisher.publish(self.state)
            # reschedule the next state-machine callback
        
        elif self.state == "Search":
            if self.search == True:
                self.publisher.publish(self.state + " g")
            elif self.center[1] < 120 - 55:
                self.publisher.publish(self.state + " v")
            elif self.center[1] > 120 + 55:
                self.publisher.publish(self.state + " b")
            elif self.center[0] < 160 - 75:
                self.publisher.publish(self.state + " f")
            elif self.center[0] > 160 + 75:
                self.publisher.publish(self.state + " g")
            else:
                if self.area < 1800:
                    self.state = "Forward"
                else:
                    self.state = "Landing"
               
        elif self.state == "Forward":
            self.publisher.publish(self.state + " w")
            rospy.sleep(0.2)
            self.state = "Hover"

        elif self.state == "Backward":
            self.publisher.publish(self.state + " x")
            rospy.sleep(0.2)
            self.state = "Hover"

        elif self.state == "Hover":
            self.publisher.publish(self.state + " s")
            rospy.sleep(0.3)
            self.state = "Search"


        rospy.Timer( rospy.Duration(0.1), self.FSM0, oneshot=True )


    def FSM1(self, timer_event=None):
        print "center is:", self.center
        print "area is:", self.area
        print "state is:", self.state
        self.publisher.publish("ChangeCamera 1")
        if self.state == "Keyboard": # user stopped our state machine!
            print "State machine has been stopped!"
            print "Sending hover command"
            self.publisher.publish(self.state)
            return # officially ends the state machine

        elif self.state == "Landing":
            self.publisher.publish(self.state)
            self.airborne = False
            return

        elif self.state == "Start":
            print "Starting the state machine!"
            if self.airborne == True:
                self.state = "Hover"
            else: 
                self.airborne = True
                self.state = "Takeoff"
            self.publisher.publish(self.state)
            
        elif self.state == "Takeoff":
            print "taking off in the FSM"
            rospy.sleep(3.0)  # wait for takeoff...
            self.state = "Hover"
            self.publisher.publish(self.state)
            # reschedule the next state-machine callback
        
        elif self.state == "Search":
            if self.search == True:
                self.publisher.publish(self.state + " g")
            elif self.center[1] < 120 - 55:
                self.publisher.publish(self.state + " v")
            elif self.center[1] > 120 + 55:
                self.publisher.publish(self.state + " b")
            elif self.center[0] < 160 - 75:
                self.publisher.publish(self.state + " f")
            elif self.center[0] > 160 + 75:
                self.publisher.publish(self.state + " g")
            else:
                if self.area < 1000:
                    self.state = "Forward"
                elif self.area > 1800:
                    self.state = "Backward"
                else:
                    self.publisher.publish(self.state + " s")   
        
        elif self.state == "Forward":
            self.publisher.publish(self.state + " w")
            rospy.sleep(0.2)
            self.state = "Hover"

        elif self.state == "Backward":
            self.publisher.publish(self.state + " x")
            rospy.sleep(0.2)
            self.state = "Hover"

        elif self.state == "Hover":
            self.publisher.publish(self.state + " s")
            rospy.sleep(0.3)
            self.state = "Search"

        rospy.Timer( rospy.Duration(0.1), self.FSM1, oneshot=True )

    def FSM2(self, timer_event=None):
        print "center is:", self.center
        print "area is:", self.area
        print "state is:", self.state
        self.publisher.publish("ChangeCamera 2")
        if self.state == "Keyboard": # user stopped our state machine!
            print "State machine has been stopped!"
            print "Sending hover command"
            self.publisher.publish(self.state)
            return # officially ends the state machine

        elif self.state == "Landing":
            self.publisher.publish(self.state)
            self.airborne = False
            return

        elif self.state == "Start":
            print "Starting the state machine!"
            if self.airborne == True:
                self.state = "Hover"
            else: 
                self.airborne = True
                self.state = "Takeoff"
            self.publisher.publish(self.state)
            
        elif self.state == "Takeoff":
            print "taking off in the FSM"
            rospy.sleep(3.0)  # wait for takeoff...
            self.state = "Hover"
            self.publisher.publish(self.state)
            # reschedule the next state-machine callback
        
        elif self.state == "Search":
            if self.search == True:
                self.publisher.publish(self.state + " v")
            elif self.center[1] < 120 - 55:
                self.state = "Forward"
            elif self.center[1] > 120 + 55:
                self.state = "Backward"
            elif self.center[0] < 160 - 75:
                self.state = "Left"
            elif self.center[0] > 160 + 75:
                self.state = "Right"
            else:
                if self.area < 12000:
                    self.publisher.publish(self.state + " b")
                else:
                    self.state = "Landing" 
        
        elif self.state == "Forward":
            self.publisher.publish(self.state + " w")
            rospy.sleep(0.2)
            self.state = "Hover"

        elif self.state == "Backward":
            self.publisher.publish(self.state + " x")
            rospy.sleep(0.2)
            self.state = "Hover"

        elif self.state == "Left":
            self.publisher.publish(self.state + " a")
            rospy.sleep(0.2)
            self.state = "Hover"

        elif self.state == "Right":
            self.publisher.publish(self.state + " d")
            rospy.sleep(0.2)
            self.state = "Hover"

        elif self.state == "Hover":
            self.publisher.publish(self.state + " s")
            rospy.sleep(0.2)
            self.state = "Search"

        rospy.Timer( rospy.Duration(0.1), self.FSM2, oneshot=True )

    def handle_image_data(self, data):
        packet = data.data.split()
        if packet[0] == "ChangeCamera":
            if packet[1] == "1": self.cam == "Front"
            else: self.cam == "Bottom"
            self.publisher.publish("ChangeCamera " + packet[1])
        elif packet[0] == "None": self.search = True
        else:
            imageData = [int(x) for x in packet[0:4]]
            if self.cam == "Front": self.frontImageData = imageData
            else: self.bottomImageData = imageData

            self.center = [(imageData[0] + imageData[2])/2, \
                    (imageData[1] + imageData[3])/2]
            self.area = int(packet[4])
            if (imageData[3] - imageData[1]) < 10:
                self.search = True
            else: self.search = False
            
    def handle_GUI_data(self, data):
        packet = data.data.split()
        if packet[0] == "Airborne": self.airborne = True
        if packet[0] == "Landing":
            self.airborne = False
            self.state = packet[0]
        if packet[0] == "ChangeCamera":
            if packet[1] == "1": self.cam == "Front"
            else: self.cam == "Bottom"
        if packet[0] == "Keyboard":
            self.state = packet[0]
        
        
    






