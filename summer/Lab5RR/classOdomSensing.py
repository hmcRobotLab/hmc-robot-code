#!/usr/bin/env python
import roslib; roslib.load_manifest('Frizzle')
import rospy
import irobot_create_2_1
from std_msgs.msg import String
from irobot_create_2_1.srv import *
from irobot_create_2_1.msg import *
import odomClass
from nav_msgs.msg import Odometry
from geometry_msgs.msg import *
from tf.msg import *
import sys, select
from Tkinter import *
from turtle import *
import math
import time

shape("turtle")
color("green")

def handle_sensor_data( data ):
    """
        handle_sensor_data is called every time the robot gets a new sensorPacket
    """
    global odometry
    odometry = odometry.updateOdometry(data)

def handle_key_data(data):
    if data.data == 'q' or data.data == chr(27): # if a 'q' or Esc was pressed
        print 'quitting'
        rospy.signal_shutdown( "Quit requested from keyboard" )

    if data.data == 'r':
        print "Resetting"
        odometry = odomClass.Odometry()
        
        
def ros_services():
        """ sets data members self.tank and self.song, analogous to lab 1 """
        global tank
        global song
        # obtain the tank service
        rospy.wait_for_service('tank') # won't continue until the "tank" service is on
        tank = rospy.ServiceProxy('tank', Tank) # tank permits requests, e.g., tank(0,50,50)
        # obtain the song service
        rospy.wait_for_service('song') #
        song = rospy.ServiceProxy('song', Song)
        
def listener():
    global odometry
    odometry = odomClass.Odometry() #Initializes the odometer
    rospy.init_node('listener', anonymous=True) #initialize this node.
        #Since it only listens, we can have it anonymous.
    rospy.Subscriber('keyPress', String, handle_key_data)
    rospy.Subscriber('sensorPacket', SensorPacket, handle_sensor_data)

def odometryDraw():
    global odometry
    global tank
    square = True
    while True:
        
        if square == True: #If we're drawing squares...
            for num in range(4): #For each side of the square...
                distance = 0 #Set the side length at 
                start_x = odometry.current_x #Set the starting coordinates
                start_y = odometry.current_y #Set the starting coordinates
                start_theta = odometry.current_theta #Set the starting angle
                while distance < 7000: #Until we've drawn a full side...
                    distance += math.hypot(odometry.current_x- start_x, odometry.current_y - start_y) #Add to the length of the side
                    print odometry
                    setpos(odometry.current_x, odometry.current_y) #Move the turtle with the iRobot
                    setheading(odometry.current_theta) #Change the turtle's heading
                    tank(0, 50, 50) #Move the iRobot forward
                while odometry.current_theta < (start_theta + 90): #Until we've turned 90 degrees...
                    print odometry
                    setpos(odometry.current_x, odometry.current_y) #Move the turtle with the iRobot
                    setheading(odometry.current_theta) #Change the turtle's heading
                    tank(0, -50, 50) #Rotate the iRobot
        
        else: #If we're not drawing a square...
            print odometry
            setpos(odometry.current_x, odometry.current_y) #Move the turtle with the iRobot
            setheading(odometry.current_theta) #Change the turtle's heading


if __name__ == '__main__':
    """
    this simply runs listener
    """
    global odometry
    listener() # initializes code to listen for the dock
    ros_services()
    odometry = odomClass.Odometry()
    odometryDraw()
    print "Bye!"
    rospy.spin() #this stops the code from completing.  It will allow other 
        #processes, like listening, to happen, and the code won't end.
