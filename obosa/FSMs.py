from std_msgs.msg import String
import rospy

class FSM:
    def __init__(self):
        self.imageData = [0, 0, 0, 0]
        self.center = [0, 0]
        self.state = "None"
        self.publisher = rospy.Publisher('state', String)
        self.area = 0
        self.search = False
        rospy.Subscriber('imageData', String, self.handle_image_data)
        
        # our drone's state machine
    
    def setState(self, state):
        self.state = state


    def FSM0(self, timer_event=None):
        if self.state == "Keyboard": # user stopped our state machine!
            print "State machine has been stopped!"
            print "Sending hover command"
            self.publisher.publish(self.state)
            return # officially ends the state machine

        elif self.state == "Start":
            print "Starting the state machine!"
            self.state = "Takeoff"
            self.publisher.publish(self.state)
            rospy.Timer( rospy.Duration(0.1), self.FSM1, oneshot=True )
            
        elif self.state == "Takeoff":
            print "taking off in the FSM"
            rospy.sleep(3.0)  # wait for takeoff...
            self.state = "Hover"
            self.publisher.publish(self.state)
            # reschedule the next state-machine callback
            rospy.Timer( rospy.Duration(0.1), self.FSM1, oneshot=True )
            return
        
        elif self.state == "Search":
            print "center is:", self.center[0]
            if self.search == True:
                self.publisher.publish(self.state + " g")
	        #elif self.center[1] < 120 - 55:
            #    self.control_drone("up")
            #elif self.center[1] > 120 + 55:
            #    self.control_drone("down")
            elif self.center[0] < 160 - 75:
                self.publisher.publish(self.state + " f")
            elif self.center[0] > 160 + 75:
                self.publisher.publish(self.state + " g")
            else:
                if self.area < 1000:
                    self.state = "Forward"
                elif self.area > 1800:
                    self.state = "Land"
                else:
                    self.publisher.publish(self.state + " s")   
            rospy.Timer( rospy.Duration(0.1), self.FSM1, oneshot=True )
            return
        
        elif self.state == "Forward":
            self.publisher.publish(self.state + " w")
            rospy.sleep(0.2)
            self.state = "Hover"
            rospy.Timer( rospy.Duration(0.1), self.FSM1, oneshot=True )
            return

        elif self.state == "Backward":
            self.publisher.publish(self.state + " x")
            rospy.sleep(0.2)
            self.state = "Hover"
            rospy.Timer( rospy.Duration(0.1), self.FSM1, oneshot=True )
            return
        
        elif self.state == "Land":
            self.state = "Landing"
            self.publisher.publish(self.state)
            rospy.Timer( rospy.Duration(0.1), self.FSM1, oneshot=True )





    def FSM1(self, timer_event=None):
        """ the finite-state machine """

        if self.state == "Keyboard": # user stopped our state machine!
            print "State machine has been stopped!"
            print "Sending hover command"
            self.publisher.publish(self.state)
            return # officially ends the state machine

        elif self.state == "Start":
            print "Starting the state machine!"
            self.state = "Takeoff"
            self.publisher.publish(self.state)
            rospy.Timer( rospy.Duration(0.1), self.FSM1, oneshot=True )
            
        elif self.state == "Takeoff":
            print "taking off in the FSM"
            rospy.sleep(3.0)  # wait for takeoff...
            self.state = "Hover"
            self.publisher.publish(self.state)
            # reschedule the next state-machine callback
            rospy.Timer( rospy.Duration(0.1), self.FSM1, oneshot=True )
            return
        
        elif self.state == "Search":
            print "center is:", self.center[0]
            if self.search == True:
                self.publisher.publish(self.state + " g")
	        #elif self.center[1] < 120 - 55:
            #    self.control_drone("up")
            #elif self.center[1] > 120 + 55:
            #    self.control_drone("down")
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
            rospy.Timer( rospy.Duration(0.1), self.FSM1, oneshot=True )
            return
        
        elif self.state == "Forward":
            self.publisher.publish(self.state + " w")
            rospy.sleep(0.2)
            self.state = "Hover"
            rospy.Timer( rospy.Duration(0.1), self.FSM1, oneshot=True )
            return

        elif self.state == "Backward":
            self.publisher.publish(self.state + " x")
            rospy.sleep(0.2)
            self.state = "Hover"
            rospy.Timer( rospy.Duration(0.1), self.FSM1, oneshot=True )
            return
        
        elif self.state == "Land":
            self.state = "Landing"
            self.publisher.publish(self.state)
            rospy.Timer( rospy.Duration(0.1), self.FSM1, oneshot=True )
	
	
	#elif President in range and self.state == "assassinate":
	#   self.control_drone("gun")
        #   rospy.Timer(rospy.Duration("6 years"), self.FSM1, oneshot=ONEKILL)
           
                
        elif self.state == "Hover":
            self.publisher.publish(self.state + " s")
            rospy.sleep(0.3)
            self.state = "Search"
            rospy.Timer( rospy.Duration(0.1), self.FSM1, oneshot=True )
            return

        else:  # state not recognized
            print "the state", self.state, "was not recognized"
            print "Changing to Keyboard state"
            self.state = "Keyboard"
            self.publisher.publish(self.state)
            rospy.Timer( rospy.Duration(0.1), self.FSM1, oneshot=True )
            return

    def handle_image_data(self, data):
        packet = data.data.split()
        self.imageData = [int(x) for x in packet[0:4]]
        self.center = [(self.imageData[0] + self.imageData[2])/2, \
                (self.imageData[1] + self.imageData[3])/2]
        self.area = int(packet[4])
        contours = int(packet[5])
        if contours == 0: self.search = True
        if (self.imageData[3] - self.imageData[1]) < 8:
            self.search = True
        else: self.search = False
        
    






