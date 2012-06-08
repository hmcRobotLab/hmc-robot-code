# helper function for state machines, will make sure it stops
# when any key that controls the robot separately has been pressed
def transition( time, next_state ):
    """ time is a float, next_state is a funcion """
    if D["state"] == "keyboard":
        print "State machine has been stopped!"
        D["tank"](0,0,0)
        return
    rospy.Timer( rospy.Duration(time), next_state, oneshot=True )

# go forward state
def state_forward( timer_event=None ):
    global D
    print "Now in state forward!"
    D["tank"](0,50,50)
    transition(2.0, state_backwards)

# go backwards state
def state_backwards( timer_event=None ):
    global D
    print "Now in state backwards!"
    D["tank"](0, -50, -50)
    transition(2.0, state_stop)

# Stops the state machine by not having another transition
def state_stop( timer_event=None ):
    global D
    print "Now in state stop"
    D["tank"](0, 0, 0)
    D["state"] = "keyboard"
