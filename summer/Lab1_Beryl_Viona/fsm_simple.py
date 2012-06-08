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

def state_0( timer_event = None ):
    """ goes forward """
    print "Now in state 0: Going forward"

    tank(0, 100, 100)

    rospy.Timer( rospy.Duration(1.0), state_1, oneshot=True )

def state_1( timer_event = None ):
    """ turns to the right """
    print "Now in state 1: Turning right"

    tank( 0, -200, 200)

    rospy.Timer( rospy.Duration(1.0), state_2, oneshot=True )

def state_2( timer_event = None ):
    """ moving forwards """
    print "Now in state 2: Going forward"

    tank( 0, 100, 100)

    rospy.Timer( rospy.Duration(1.0), state_3, oneshot=True )
    
def state_3( timer_event = None ):
    """ stopping """
    print "Now in state 3: Stopping"
    tank(0, 0, 0)
    rospy.signal_shutdown("state 3")
    
    
def get_services():
    """ returns an object (tank) that allows you
       to set the velocities of the robot's wheels
    """
    # obtain the tank service
    rospy.wait_for_service('tank') # won't continue until the "tank" service is on
    tank = rospy.ServiceProxy('tank', Tank) # tank permits requests, e.g., tank(0,50,50)
    # obtain the song serviceAll through these three states, you should put in an if/else statement where pressing the play button will stop your robot and exit the code. (This will let you stop your robot easily when everyone else is also testing and there is about to be a huge robot pileup!)
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

    # start finite state machine
    state_0()

    # run "forever"
    rospy.spin()
    print "Goodbye!"


# this is the "main" trick: it tells Python
# what code to run when you run this as a stand-alone script:
if __name__ == "__main__":
   main()
