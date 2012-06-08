#!/usr/bin/env python

import roslib; roslib.load_manifest('Daneel')
import rospy
import tf
from tf.transformations import euler_from_quaternion
import sensor_msgs.msg as sm
import cv_bridge
import cv
import irobot_create_2_1
from std_msgs.msg import String
from irobot_create_2_1.srv import *
from irobot_create_2_1.msg import *
from math import *
import time, threading
import pydrone.srv
import pydrone.ARDrone as ARDrone
import pydrone.msg as msg
import os, random

D = {}

def init_globals(use_drone=False):
    """ setup """
    global D

    # create cv stuff

    print " Started making a window"
    cv.NamedWindow('skeleton')
    cv.MoveWindow('skeleton', 0, 0)
    cv.SetMouseCallback('skeleton', onMouse, None)
    print "finished making a window"
