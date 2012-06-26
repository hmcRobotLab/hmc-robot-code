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
        self.airborne = False
        self.end = False
        self.y_center = 120
        self.y_threshold = 35
        self.x_center = 160
        self.x_threshold = 55
        rospy.Subscriber('imageData', String, self.handle_image_data)
        rospy.Subscriber('GUI', String, self.handle_GUI_data)

    def YCoordinate_too_low(self):
        '''Returns whether or not the Y Coordinate is below the lower threshold'''
        return self.center[1] < self.y_center - self.y_threshold

    def YCoordinate_too_high(self):
        '''Returns whether or not the Y Coordinate is above the upper threshold'''
        return self.center[1] > self.y_center + self.y_threshold
    
    def XCoordinate_too_low(self):
        '''Returns whether or not the X Coordinate is below the lower threshold'''
        return self.center[0] < self.x_center - self.x_threshold

    def XCoordinate_too_high(self):
        '''Returns whether or not the X Coordinate is above the upper threshold'''
        return self.center[0] > self.x_center + self.x_threshold

    def FSMstart(self):
        '''Gets ready to start up a new FSM'''
        self.end = True #To end any currently running FSMs
        rospy.sleep(1)
        self.state = "Start"
        self.end = False

    def printData(self):
        '''Prints out the important FSM data'''
        print "Center coordinates:", self.center
        print "Biggest area:", self.area
        print "Current state:", self.state


    def runFSMBasics(self):
        '''Runs the basic checks that all FSMs need'''
        self.printData()

        if self.state == "Keyboard": # user stopped our state machine!
            print "State machine has been stopped!"
            print "Sending hover command"
            self.publisher.publish(self.state)
            self.end = True # signal to end the state machine

        elif self.state == "Landing":
            self.publisher.publish(self.state)
            self.airborne = False
            self.end = True # signal to end the state machine

        elif self.state == "Start":
            self.end = False
            print "Starting the state machine!"
            if self.airborne == True: #If already airborne, then hover
                self.state = "Hover"
            else: #If not airborne, then takeoff first
                self.airborne = True
                self.state = "Takeoff"
            self.publisher.publish(self.state)
            
        elif self.state == "Takeoff":
            print "taking off in the FSM"
            rospy.sleep(3.0)  # wait for takeoff...
            self.state = "Hover"
            self.publisher.publish(self.state)

        elif self.state == "Forward":
            self.publisher.publish(self.state + " w") #Publish the state and keypress
            rospy.sleep(0.01)
            self.state = "Hover"

        elif self.state == "Backward":
            self.publisher.publish(self.state + " x")
            rospy.sleep(0.01)
            self.state = "Hover"

        elif self.state == "Left":
            self.publisher.publish(self.state + " a")
            rospy.sleep(0.01)
            self.state = "Hover"

        elif self.state == "Right":
            self.publisher.publish(self.state + " d")
            rospy.sleep(0.01)
            self.state = "Hover"

        elif self.state == "Hover":
            self.publisher.publish(self.state + " s")
            rospy.sleep(0.5)
            self.state = "Search"


    def FSM0(self, timer_event=None):
        '''A finite state machine for landing in front of a target'''
        
        self.publisher.publish("ChangeCamera 1") #Use the forward camera

        self.runFSMBasics()
        if self.end == True:
            return # officially ends the state machine
        
        elif self.state == "Search":
            if self.search == True: # If nothing is found, rotate to the right
                self.publisher.publish(self.state + " g")
            elif self.YCoordinate_too_low(): #If the target is too high, move up
                self.publisher.publish(self.state + " v")
            elif self.YCoordinate_too_high(): #If the target is too low, move down
                self.publisher.publish(self.state + " b")
            elif self.XCoordinate_too_low(): #If the target is too far to the left, turn left
                self.publisher.publish(self.state + " f")
            elif self.XCoordinate_too_high(): #If the target is too far to the right, turn right
                self.publisher.publish(self.state + " g")
            else:
                if self.area < 2500: #If the object is centered but far away, move forward
                    self.state = "Forward"
                else: #If the object is centered and close, then land
                    self.state = "Landing"
        
        rospy.Timer( rospy.Duration(0.1), self.FSM0, oneshot=True )






    def FSM0_version2(self, timer_event=None):
        '''Same as FSM0, but uses strafes instead of turns for centering the target'''
        
        self.publisher.publish("ChangeCamera 1") #Use the forward camera

        self.runFSMBasics()
        if self.end == True:
            return # officially ends the state machine
        
        elif self.state == "Search":
            if self.search == True: #If nothing is found, rotate to the right
                self.publisher.publish(self.state + " g")
            elif self.YCoordinate_too_low(): #If the target is too high, move up
                self.publisher.publish(self.state + " v")
            elif self.YCoordinate_too_high(): #If the target is too low, move down
                self.publisher.publish(self.state + " b")
            elif self.XCoordinate_too_low(): #If the target is too far to the left, strafe left
                self.state = "Left"
            elif self.XCoordinate_too_high(): #If the target is too far to the right, strafe right
                self.state = "Right"
            else:
                if self.area < 2500: #If the object is centered but far away, move forward
                    self.state = "Forward"
                else: #If the object is centered and close, then land
                    self.state = "Landing"
               
        rospy.Timer( rospy.Duration(0.1), self.FSM0_version2, oneshot=True )






    def FSM1(self, timer_event=None):
        '''A finite state machine for following a set distance behind a target'''
        
        self.publisher.publish("ChangeCamera 1") #Use the forward camera

        self.runFSMBasics()
        if self.end == True:
            return # officially ends the state machine
        
        elif self.state == "Search":
            if self.search == True: # If nothing is found, rotate to the right
                self.publisher.publish(self.state + " g")
            elif self.YCoordinate_too_low(): #If the target is too high, move up
                self.publisher.publish(self.state + " v")
            elif self.YCoordinate_too_high(): #If the target is too low, move down
                self.publisher.publish(self.state + " b")
            elif self.XCoordinate_too_low(): #If the target is too far to the left, turn left
                self.publisher.publish(self.state + " f")
            elif self.XCoordinate_too_high(): #If the target is too far to the right, turn right
                self.publisher.publish(self.state + " g")
            else:
                if self.area < 1000: #If the target is centered but too far away, move forward
                    self.state = "Forward"
                elif self.area > 1800: #If the target is centered but too close, move backward
                    self.state = "Backward"
                else: #Otherwise, try to hover in place
                    self.state = "Hover"  

        rospy.Timer( rospy.Duration(0.1), self.FSM1, oneshot=True )





    def FSM2(self, timer_event=None):
        '''A finite state machine for landing on top of a target'''
        
        self.publisher.publish("ChangeCamera 2") #Use the bottom camera

        self.runFSMBasics()
        if self.end == True:
            return # officially ends the state machine
        
        elif self.state == "Search":
            if self.search == True: #If nothing is found, then move upward to increase the camera's range
                self.publisher.publish(self.state + " v")
            elif self.XCoordinate_too_low(): #If the target is too far to the left, then strafe left
                self.state = "Left"
            elif self.XCoordinate_too_high(): #If the target is too far to the right, then strafe right
                self.state = "Right"
            elif self.YCoordinate_too_low(): #If the target is too far ahead, then move forward
                self.state = "Forward"
            elif self.YCoordinate_too_high(): #If the target is too far behind, then move backward
                self.state = "Backward"
            else:
                if self.area < 20000: #If the target is centered but too far below the drone, then descend
                    self.publisher.publish(self.state + " b")
                else: #If the target is centered and close to the drone, then land
                    self.state = "Landing" 
        
        rospy.Timer( rospy.Duration(0.1), self.FSM2, oneshot=True )




    def FSM3(self, timer_event=None):
        '''A finite state machine for using the drone's self-adjusting hover to 
            follow an object moving below the drone (such as a roomba)
            until a target to land upon is found'''

        self.publisher.publish("ChangeCamera 2") #Use the bottom camera
        
        self.runFSMBasics()
        if self.end == True:
            return # officially ends the state machine
        
        elif self.state == "Search":
            self.publisher.publish(self.state)
            if self.search == False: 
                #If something is found, then enter the "landing on top of a target" FSM
                print "Landing on Box..."
                return self.FSM2()
            #If something isn't found, then we'll just keep looping back to this "Search" state,
            # so as to keep the image captured for the self-adjusting hover that the FSM caught
            # on its first loop through.

        rospy.Timer( rospy.Duration(0.1), self.FSM3, oneshot=True )


    def handle_image_data(self, data):
        '''Function for handling the data from the imageProcessor'''
        packet = data.data.split()
        if packet[0] == "ChangeCamera": #If told to change the camera, then send a command to the GUI
            self.publisher.publish("ChangeCamera " + packet[1])
        elif packet[0] == "None": self.search = True #If nothing is found, set self.search as True
        else:
            imageData = [int(x) for x in packet[0:4]] #Set the image data to be the data given by the imageProcessor
            self.center = [(imageData[0] + imageData[2])/2, \
                    (imageData[1] + imageData[3])/2] #Find the center of the bounding box
            self.area = int(packet[4]) #Get the area of the bounding box
            if self.area < 300: 
                #If the biggest area is negligble (i.e., probably noise), then set self.search as True
                self.search = True
            else: self.search = False
            



    def handle_GUI_data(self, data):
        '''Function for handling data from the GUI'''
        packet = data.data.split()
        if packet[0] == "Airborne": self.airborne = True
        if packet[0] == "Landing":
            self.airborne = False
            self.state = packet[0]
        if packet[0] == "Keyboard": self.state = packet[0]
        
        
    






