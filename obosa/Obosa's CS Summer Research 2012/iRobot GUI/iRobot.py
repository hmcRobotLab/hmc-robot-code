#!/usr/bin/env python

import roslib; roslib.load_manifest('irobot_mudd')
import rospy
import irobot_mudd
from std_msgs.msg import String
from irobot_mudd.srv import *
from irobot_mudd.msg import *
import odomClass
import sys
import math

rospy.wait_for_service('tank') # won't continue until the "tank" service is on
rospy.wait_for_service('song') # won't continue until the "song" service is on
tank = rospy.ServiceProxy('tank', Tank) # tank permits requests, e.g., tank(50,50)
song = rospy.ServiceProxy('song', Song) # song permits requests, e.g., song([69,70,71,72], [24,24,24,80])

class iRobot():
    def __init__(self):
        self.direction = "None"
        self.speed = 50
        self.state = "Keyboard"
        self.start_x = 0
        self.start_y = 0
        self.start_theta = 0
        self.action = "None"
        self.code = "None"
        self.odometry = odomClass.Odometry()
        
        rospy.Subscriber('sensorPacket', SensorPacket, self.handle_sensor_data)
        
        tank(0,0)

    def setStartCoordinates(self):
        '''Sets the current coordinates and heading as the starting coordinates
            and heading (useful for a variety of tasks that involve moving forward,
            stopping, turning, and continuing forward).'''
        self.start_x = self.odometry.current_x
        self.start_y = self.odometry.current_y
        self.start_theta = self.odometry.current_theta



    def distanceFromStart(self):
        '''Returns the distance from the current coordinates and the starting coordinates'''
        return math.hypot(self.odometry.current_x - self.start_x, self.odometry.current_y - self.start_y)



    def angleChange(self, theta):
        '''Returns True if the current heading has changed by theta degrees, and returns False otherwise'''
        return abs(self.odometry.current_theta) >= (abs(self.start_theta) + theta)

     
                
    def handle_sensor_data(self, sensorData ):
        '''This processes the sensor data that the robot keeps receiving, and publishes the
            updated odometry to the GUI.'''
        self.sensorData = sensorData
        self.odometry = self.odometry.updateOdometry(sensorData)

    def FSM_Square(self):
        '''Moves the robot in a square pattern'''
        distance = self.distanceFromStart()
        if distance < 200: #Until we've drawn a full side...
            tank(self.speed, self.speed) #Move the iRobot forward
        elif (distance >= 200) and not self.angleChange(90): #Until we've turned 90 degrees...
            tank(-self.speed, self.speed) #Rotate the iRobot
        else:
            self.setStartCoordinates()



    def lineFollow(self):
        '''Makes the robot follow lines based on sensor data'''

        if self.sensorData.cliffFrontRightSignal > self.thresholds["front right"]: #If the front-right sensor detects white, turn right
            tank(self.speed, -self.speed)

        elif self.sensorData.cliffRightSignal > self.thresholds["side right"]: #If the side-right sensor detects white, turn right
            tank(self.speed, -self.speed)

        elif self.sensorData.cliffLeftSignal > self.thresholds["side left"]: #If the side-left sensor detects white, turn left
            tank(-self.speed, self.speed)

        elif self.sensorData.cliffFrontLeftSignal > self.thresholds["front left"]: #If the front-left sensor detects white, turn left
            tank(-self.speed, self.speed)
            
        else:
            tank(self.speed, self.speed) # otherwise, go forward


    def keyboardMovement(self):
        '''Moves the robot based on what direction it was told to move from keypresses'''
        if self.action == 'w':
            tank(self.speed, self.speed)
        if self.action == "s":
            tank(-self.speed, -self.speed)
        if self.action == "d":
            tank(self.speed, -self.speed)
        if self.action == "a":
            tank(-self.speed, self.speed)
        if self.action == 'b':
            tank(0,0)
     
        
        
    def execute(self):
        '''This will tell the robot what to do based on what state it is in.
            This function must be placed in something repetitive  
             (such as a "while True:" loop or a timerEvent in a GUI) so that it
             is called over and over; otherwise, the finite state machines
             will not work.'''
        #Set the line-following thresholds based on the robot's speed (if it's moving fast, it should be more sensitive so that it doesn't miss the lines)
        if abs(self.speed) >= 90:
            self.thresholds = {"front right": 400, "side right": 300, "side left": 350, "front left": 400}
        
        else: self.thresholds = {"front right": 450, "side right": 350, "side left": 400, "front left": 450}


        #Run functions based on the robot's current state
        if self.state == "Square Movement":
            self.FSM_Square()

        elif self.state == "Keyboard":
            self.keyboardMovement()             

        elif self.state == "Code": #Runs the user-typed code if possible
            try: eval(self.code)
            except NameError: print "Invalid code."
            except SyntaxError: print "Invalid code."
            self.state = "done"
            
        elif self.state == "Button 1":
            #Insert a call to your function here (i.e., "self.MyFunction1()")
            return
            
        elif self.state == "Button 2":
            #Insert a call to your function here (i.e., "self.MyFunction2()")
            return

        elif self.state == "Button 3":
            #Insert a call to your function here (i.e., "self.MyFunction3()")
            return

        elif self.state == "done":
            pass

        else: tank(0, 0)
        
            
    
           
if __name__ == '__main__':
    import cv
    rospy.init_node('iRobotGUI')
    robot = iRobot()
    print """
                       iRobot control!
                      Keyboard Controls:
                            r: exit

    q: increase speed       b: brake       e: decrease speed


                            w: forward
    a: rotate left                         d: rotate right
                            s: backward 

    """           
    cv.NamedWindow('iRobot Keyboard Control')
    cv.MoveWindow('iRobot Keyboard Control', 200, 500)
    cv.WaitKey(2)
    while True: 
        robot.action = chr(cv.WaitKey() % 255)
        robot.execute()
        if robot.action == 'r': 
            tank(0,0)
            sys.exit(0)
        if robot.action == "q": 
            robot.speed += 10
            print "Speeding up. Speed is now", robot.speed
        if robot.action == "e": 
            robot.speed -= 10
            print "Slowing down. Speed is now", robot.speed

