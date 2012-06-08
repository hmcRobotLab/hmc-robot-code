#!/usr/bin/env python
import roslib; roslib.load_manifest('Frizzle')
import rospy
import irobot_create_2_1
from std_msgs.msg import String
from irobot_create_2_1.srv import *
from irobot_create_2_1.msg import *
import time


# global variables go here
# services are often global:
tank = 0
song = 0


def handle_sensor_data( data ):
   """
       handle_sensor_data is called every time the robot gets a new sensorPacket
   """
   #  uncomment this to see all of the fields of the sensorPacket
   #print dir( data )

   # print the front two IR sensor values, FrontLeft and FrontRight:
   print "FrLeft:", data.cliffFrontLeftSignal, "   ",
   print "FrRight:", data.cliffFrontRightSignal

   left = data.cliffFrontLeftSignal
   right = data.cliffFrontRightSignal

   farleft = data.cliffLeftSignal
   farright = data.cliffRightSignal
   
   # check for a bump
   if left > 350 and left > right:
       print 'turning right because we saw something on left'
       tank(0,-100,80)
   elif right > 200:
       print 'turning left because we saw something on right'
       tank(0,80,-100)
   elif data.bumpLeft == True:
       song( 0, [67,16,30,16,60,16] )
       print 'bumping something on the left! Quitting...'
       tank(0,0,0)
       rospy.signal_shutdown("left bump")
   elif data.wheeldropLeft == True:
       song( 0, [67,16,30,16,60,16] )
       print 'picked up! Quitting...'
       tank(0,0,0)
       rospy.signal_shutdown("wheel drop")
   else:
       tank(0,120,120) # otherwise, go forward


def subscribe():
   """
       subscribe tells ROS which topics it want to subscribe to
       and indicates the functions that will act as "callbacks"
       to handle the information from those topics
   """
   # this next line subscribes to sensorPacket
   # it tells ROS that the function handle_sensor_data
   # will handle each incoming sensorPacket
   rospy.Subscriber('sensorPacket', SensorPacket, handle_sensor_data)


def get_services():
   """ returns an object (tank) that allows you
       to set the velocit
ies of the robot's wheels
   """
   # obtain the tank service
   rospy.wait_for_service('tank') # won't continue until the "tank" service is on
   tank = rospy.ServiceProxy('tank', Tank) # tank permits requests, e.g., tank(0,50,50)
   # obtain the song service
   rospy.wait_for_service('song') #
   song = rospy.ServiceProxy('song', Song)
   # returning two things is easy with Python:
   return tank, song


def main():
   """ the main program that gives our node a name,
       sets up service objects, subscribes to topics (with a callback),
       and then lets interactions happen!
   """
   # we should give our program a ROS node name
   # the name is not important, so we use "lab1_node"
   rospy.init_node('lab1_node', anonymous=True)

   # set up services
   global tank, song
   tank, song = get_services()

   # subscribe to data
   subscribe()

   # run "forever"
   rospy.spin()
   print "Goodbye!"


# this is the "main" trick: it tells Python
# what code to run when you run this as a stand-alone script:
if __name__ == "__main__":
   main()
