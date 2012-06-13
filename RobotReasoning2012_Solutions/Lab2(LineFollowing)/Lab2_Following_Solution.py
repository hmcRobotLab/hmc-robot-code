
#!/usr/bin/env python
import roslib; roslib.load_manifest('irobot_mudd')
import rospy
import irobot_mudd
from std_msgs.msg import String
from irobot_mudd.srv import *
from irobot_mudd.msg import *
import time
import random


# global variables go here
# services are often global:
tank = 0
song = 0
handling_data = False
        
def handle_sensor_data( data ):
    """
       handle_sensor_data is called every time the robot gets a new sensorPacket
    """

    global handling_data
    if handling_data == True:
       return 

    handling_data = True
    #uncomment this to see all of the fields of the sensorPacket
    #print dir( data )

    #uncomment the four below to print the IR sensor data

    print "FrontLeft:", data.cliffFrontLeftSignal, "   ",
    print "FrontRight:", data.cliffFrontRightSignal, "   "
    #print "FarLeft:", data.cliffLeftSignal, "   ",
    #print "FarRight:", data.cliffRightSignal
    #print "Left Bump:", data.bumpLeft


    if data.play:
        """if the iRobots's play button is pressed,
          the robot comes to a halt and program shuts down"""
        tank( 0, 0)
        rospy.signal_shutdown("play button pressed")

    elif data.cliffFrontLeftSignal>600:
        print 'Line On My Left..Adjusting'
        tank(-30,50)


    elif data.cliffFrontRightSignal>600:
        print 'Line On My Right..Adjusting'
        tank(50,-30)
 
    else:
       tank(50,50) # otherwise, continue doing what you're doing

    handling_data = False


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
       to set the velocities of the robot's wheels
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
