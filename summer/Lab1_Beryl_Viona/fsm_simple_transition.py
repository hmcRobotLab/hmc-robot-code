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
data = 0
ready_to_start = False

####
# Helper function to check for stops during state machine
####

def transition( time, next_state, definitely_not_ending=False):
    """ time is a float, next_state is a function """
    if ready_to_start:
        # check if we should stop
        if data.play and not definitely_not_ending:
            tank( 0, 0, 0)
            rospy.signal_shutdown("play button pressed")
            return
    rospy.Timer( rospy.Duration(time), next_state, oneshot=True )

####
# State Machine Starts Here
####

def state_start( timer_event = None ):
    """ waits for start"""
    print "Waiting to start"
    if ready_to_start:
        if data.play:
            transition( 1.0, state_forward, True)
        else:
            transition( 0.1, state_start)
    else:
        transition( 0.1, state_start)
        
def state_0( timer_event = None ):
    """ goes forward """
    print "Now in state 0: Going forward"
    tank(0, 100, 100)
    transition( 1.0, state_1 )

def state_1( timer_event = None ):
    """ turns to the right """
    print "Now in state 1: Turning right"
    tank( 0, -200, 200)
    transition( 1.0, state_0 )
    
####
# State Machine Ends Here
####

def handle_sensor_data( new_data ):
    """
       handle_sensor_data is called every time the robot gets a new sensorPacket,
       updates global data with new data for everyone to look at
    """
    global data, ready_to_start
    data = new_data

    if not ready_to_start:
        ready_to_start = True
        
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
    
    # start finite state machine
    state_start()

    # run "forever"
    rospy.spin()
    print "Goodbye!"


# this is the "main" trick: it tells Python
# what code to run when you run this as a stand-alone script:
if __name__ == "__main__":
   main()
