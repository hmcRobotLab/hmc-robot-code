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
        self.speed = 50
        self.state = "Keyboard"
        self.start_x = 0
        self.start_y = 0
        self.start_theta = 0
        self.lastaction = "None"
        self.odometry = odomClass.Odometry()
        rospy.Subscriber('sensorPacket', SensorPacket, self.handle_sensor_data)
        rospy.Subscriber('GUI', String, self.handle_GUI_data)
        
        tank(0,0)

    def handle_GUI_data(self, GUIdata):
        '''This parses and processes the commands published by the GUI and its widgets'''
        info = GUIdata.data.split()
            
        if info[0] == "reset": #If the reset button was pressed, then reset the robot's information
            self.init()

        elif info[0] == "Button": 
        #If one of the user-defined buttons were pressed, then reset all the robot's 
        #information, set the current coordinates as starting coordinates for one of
        #the tasks that the buttons do, and tell the GUI to reset its values and
        #the turtle.
        
            setStartCoordinates()
            if info[1] == "1": 
                self.state = "Button 1"
            elif info[1] == "2": 
                self.state = "Button 2"
            else: 
            #if button 3 was pressed, then set up data that we'll use for button 3's
            #task
                self.state = "Button 3"
                
        else:
            print "Message not recognized."
            pass


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
            
                    
    def keyboardControl(self, action):
        '''Moves the robot based on what direction it was told to move from keypresses'''
        self.state = "Keyboard"
        if action != "SpeedUp" and action != "SlowDown":
            self.lastaction = action
        if action == "Forward":
            tank(self.speed, self.speed)
        elif action == "Reverse":
            tank(-self.speed, -self.speed)
        elif action == "Right":
            tank(self.speed, -self.speed)
        elif action == "Left":
            tank(-self.speed, self.speed)
        elif action == "SpeedUp":
            self.speed += 10
            self.keyboardControl(self.lastaction)
            "Speeding up. Speed is now:", self.speed
        elif action == "SlowDown":
            self.speed -= 10
            self.keyboardControl(self.lastaction)
            "Slowing down. Speed is now:", self.speed
        else:
            tank(0, 0)
        
        
        
        
    def execute(self):
        '''This will tell the robot what to do based on what state it is in.'''
        #Set the line-following thresholds based on the robot's speed (if it's moving fast, it should be more sensitive so that it doesn't miss the lines)
        if abs(self.speed) >= 90:
            self.thresholds = {"front right": 375, "side right": 400, "side left": 230, "front left": 200}
        else: self.thresholds = {"front right": 425, "side right": 410, "side left": 245, "front left": 225}



        #Run functions based on the robot's current state
        if self.state == "Square Movement":
            FSM_Square()

        elif self.state == "Keyboard":
            pass             

        elif self.state == "Code": #Runs the user-typed code if possible
            try: eval(self.code)
            except NameError: print "Invalid code."
            except SyntaxError: print "Invalid code."
            self.state = "done"
            
        elif self.state == "Button 1":
            runButton1()
            
        elif self.state == "Button 2":
            runButton2()
            
        elif self.state == "Button 3":
            runButton3()
            
        elif self.state == "done":
            pass

        else: tank(0, 0)
        
            
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
           
           

