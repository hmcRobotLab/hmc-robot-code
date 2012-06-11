#!/usr/bin/env python

import roslib; roslib.load_manifest('Frizzle')
import rospy
from std_msgs.msg import String
from irobot_create_2_1.srv import *
from irobot_create_2_1.msg import *
import odomClass
import sys
import math
       
def handle_GUI_data(GUIdata):
    '''This parses and processes the commands published by the GUI and its widgets'''
    global data
    global pub
    info = GUIdata.data.split()
    if info[0] == "keyPress:": #If a keyboard key was pressed, then switch to Keyboard state and move accordingly
        data["direction"] = info[1]
        print data["direction"]
        data["state"] = "Keyboard"
        
    elif info[0] == "changeState:": #If the checkbox was clicked, then change state accordingly
        if data["state"] == "Keyboard":
            data["state"] = "Square Movement"
            print "Square!"
            setStartCoordinates()
        elif data["state"] == "Square Movement":
            data["state"] = "Keyboard"
        
    elif info[0] == "speed:": #If the dial was turned, then change the speed to the new dial value
        data["speed"] = int(info[1])
        
    elif info[0] == "changeSpeed:": #If 'Q' or 'E' were pressed, then change the speed accordingly
        data["speed"] += int(info[1])
        if int(info[1]) > 0:
            print "Speeding up"
        else: print "Slowing down"
        
    elif info[0] == "reset": #If the reset button was pressed, then reset the robot's information
        resetValues()
        
    elif info[0] == "code:": #If code was sent to the robot, then save the code for later execution
        if len(info) > 1:
            print "Code sent to robot: " + info[1]
            data["state"] = "Code"
            data["code"] = info[1]
        else: 
            print "No code sent to robot."
            
    elif info[0] == "Button": 
    #If one of the user-defined buttons were pressed, then reset all the robot's 
    #information, set the current coordinates as starting coordinates for one of
    #the tasks that the buttons do, and tell the GUI to reset its values and
    #the turtle.
    
        resetValues()
        data["pub"].publish("reset")
        setStartCoordinates()
        if info[1] == "1": 
            data["state"] = "Button 1"
        elif info[1] == "2": 
            data["state"] = "Button 2"
        else: 
        #if button 3 was pressed, then set up data that we'll use for button 3's
        #task
            data["state"] = "Button 3"
            data["width"] = 0
            data["length"] = 0
            data["turn"] = False
            
    else:
        print "Message not recognized."
        pass



def runButton1():
    '''Button 1 moves the robot forward 20 cm, and turns the robot 120 degrees clockwise.'''
    moveAndTurn(20, 120)
    
    
    
def runButton2():
    '''Button 2 makes the robot follow a line until it has reached its starting point'''
    followLoopOnce()
    return



def runButton3():
    '''Button 3 makes the robot measure the area of a rectangular course'''
    measureRoom2()
    return



def FSM_Square():
    '''Moves the robot in a square pattern'''
    global data
    global tank
    global song
    distance = distanceFromStart()
    if distance < 500: #Until we've drawn a full side...
        tank(0, data["speed"], data["speed"]) #Move the iRobot forward
    elif (distance >= 500) and not angleChange(90): #Until we've turned 90 degrees...
        tank(0, -data["speed"], data["speed"]) #Rotate the iRobot
    else:
        setStartCoordinates()
        data["state"] = "done"




def moveAndTurn(k, d):
    '''Moves the robot k centimeters, and rotates the robot d degrees clockwise'''
    global tank
    global song
    global data
    distance = distanceFromStart()
    if distance < k * 10: #Until we've gone k cm...
        tank(0, data["speed"], data["speed"]) #Move the iRobot forward
    elif (distance >= k * 10) and not angleChange(d): #Until we've turned d degrees...
        tank(0, data["speed"], -data["speed"]) #Rotate the iRobot
    else:
        setStartCoordinates()
        data["state"] = "done"
        tank(0, 0, 0)



            
def followLoopOnce():
    '''Makes the robot follow lines until it has gone around the given course once'''
    global tank
    global song
    global data
    lineFollow()
    distance = distanceFromStart()
    if distance <= 100 and angleChange(90):
        print "End of course."
        song(0,[69,24,70,24,71,24,72,80])
        data["state"] = "done"
        tank(0, 0, 0)
    
    
    
    
def measureRoom():
    '''Makes the robot measure the length and width of a rectangular room, and print the length, width, and area.'''
    global data
    global tank
    if data["length"] == 0 and data["sensorData"].cliffRightSignal > data["thresholds"]["side right"]: #If the robot has not recorded the length yet, and it hits a wall, then set the distance traveled as the length, and make the robot turn
        data["length"] = distanceFromStart()
        setStartCoordinates()
        data["turn"] = True
        
    elif data["turn"] == True and not angleChange(90): #if the robot has not turned 90 degrees yet, then keep turning
        tank(0, data["speed"], -data["speed"])
        
    elif data["turn"] == True and angleChange(90):  #If the robot has turned 90 degrees, stop turning, and start the new side (width)
        data["turn"] = False
        setStartCoordinates()
        
    elif data["length"] != 0 and data["sensorData"].cliffRightSignal > data["thresholds"]["side right"]: #If the length has been recorded and the robot hits a wall, then stop and set the distance traveled as the width
        data["width"] = distanceFromStart()
        
    elif data["length"] != 0 and data["width"] != 0: #If the robot has finished measuring, then print the information
        length = str(data["length"])
        width = str(data["width"])
        area = str(data["length"] * data["width"])
        meterLength = str(data["length"] / 1000)
        meterWidth = str(data["width"] / 1000)
        meterArea = str((data["length"]/ 1000) * (data["width"] / 1000))
        tank(0,0,0)
        print "Room length is: " + length + " millimeters (" + meterLength + " meters).\nRoom width is: " + width + " millimeters (" + meterWidth + " meters).\nTotal room area is: " + area + " millimeters squared (" + meterArea + " meters squared)."
        data["state"] = "done"
        
    else: tank(0,data["speed"],data["speed"]) #Otherwise, move forward!
        
        
        
def measureRoom2():
    '''Makes the robot measure the length and width of a rectangular room, and print the length, width, and area.'''
    global data
    global tank
    if data["length"] == 0 and angleChange(45): #If the robot has not recorded the length yet, and it hits a wall, then set the distance traveled as the length, and make the robot turn
        data["length"] = distanceFromStart()
        setStartCoordinates()
        
    elif data["length"] != 0 and angleChange(45): #If the length has been recorded and the robot hits a wall, then stop and set the distance traveled as the width
        data["width"] = distanceFromStart()
        setStartCoordinates()
        
    elif data["length"] != 0 and data["width"] != 0: #If the robot has finished measuring, then print the information
        length = str(data["length"])
        width = str(data["width"])
        area = str(data["length"] * data["width"])
        meterLength = str(data["length"] / 1000)
        meterWidth = str(data["width"] / 1000)
        meterArea = str((data["length"]/ 1000) * (data["width"] / 1000))
        tank(0,0,0)
        print "Room length is: " + length + " millimeters (" + meterLength + " meters).\nRoom width is: " + width + " millimeters (" + meterWidth + " meters).\nTotal room area is: " + area + " millimeters squared (" + meterArea + " meters squared)."
        data["state"] = "done"
        
    else: lineFollow() #Otherwise, move forward!
              

def lineFollow():
    '''Makes the robot follow lines based on sensor data'''
    global tank
    global data

    if data["sensorData"].cliffFrontRightSignal > data["thresholds"]["front right"]: #If the front-right sensor detects white, turn right
        tank(0, data["speed"], -data["speed"])

    elif data["sensorData"].cliffRightSignal > data["thresholds"]["side right"]: #If the side-right sensor detects white, turn right
        tank(0, data["speed"], -data["speed"])

    elif data["sensorData"].cliffLeftSignal > data["thresholds"]["side left"]: #If the side-left sensor detects white, turn left
        tank(0, -data["speed"], data["speed"])

    elif data["sensorData"].cliffFrontLeftSignal > data["thresholds"]["front left"]: #If the front-left sensor detects white, turn left
        tank(0, -data["speed"], data["speed"])
        
    else:
        tank(0, data["speed"], data["speed"]) # otherwise, go forward
        
                
def keyboardMovement():
    '''Moves the robot based on what direction it was told to move from keypresses'''
    global data
    if data["direction"] == "Forward":
        tank(0, data["speed"], data["speed"])
    elif data["direction"] == "Reverse":
        tank(0, -data["speed"], -data["speed"])
    elif data["direction"] == "Right":
        tank(0, data["speed"], -data["speed"])
    elif data["direction"] == "Left":
        tank(0, -data["speed"], data["speed"])
    else:
        tank(0, 0, 0)
    
    
    
    
def execute():
    '''This will tell the robot what to do based on what state it is in.'''
    global tank
    global song
    global data
    while True: #Main loop that keeps running everything
        
        #Set the line-following thresholds based on the robot's speed (if it's moving fast, it should be more sensitive so that it doesn't miss the lines)
        if abs(data["speed"]) >= 90:
            data["thresholds"] = {"front right": 375, "side right": 400, "side left": 230, "front left": 200}
        else: data["thresholds"] = {"front right": 425, "side right": 410, "side left": 245, "front left": 225}



        #Run functions based on the robot's current state
        if data["state"] == "Square Movement":
            FSM_Square()
            
        elif data["state"] == "Keyboard":
            keyboardMovement()
                
        elif data["state"] == "Code": #Runs the user-typed code if possible
            try: eval(data["code"])
            except NameError: print "Invalid code."
            except SyntaxError: print "Invalid code."
            data["state"] = "done"
            
        elif data["state"] == "Button 1":
            runButton1()
            
        elif data["state"] == "Button 2":
            runButton2()
            
        elif data["state"] == "Button 3":
            runButton3()
            
        elif data["state"] == "done":
            pass
        
        else: tank(0, 0, 0)
        
        
def setStartCoordinates():
    '''Sets the current coordinates and heading as the starting coordinates
        and heading (useful for a variety of tasks that involve moving forward,
        stopping, turning, and continuing forward).'''
    global data
    data["start_x"] = data["odometry"].current_x
    data["start_y"] = data["odometry"].current_y
    data["start_theta"] = data["odometry"].current_theta



def distanceFromStart():
    '''Returns the distance from the current coordinates and the starting coordinates'''
    global data
    return math.hypot(data["odometry"].current_x - data["start_x"], data["odometry"].current_y - data["start_y"])




def angleChange(theta):
    '''Returns True if the current heading has changed by theta degrees, and returns False otherwise'''
    global data
    return abs(data["odometry"].current_theta) >= (abs(data["start_theta"]) + theta)


      
            
def handle_sensor_data( sensorData ):
    '''This processes the sensor data that the robot keeps receiving, and publishes the
        updated odometry to the GUI.'''
    global data
    data["sensorData"] = sensorData
    data["odometry"] = data["odometry"].updateOdometry(sensorData)
    data["pub"].publish(String("Odometry: " + str(data["odometry"].current_x) + " " + str(data["odometry"].current_y) + " " + str(data["odometry"].current_theta)))
       
       
       
        
def ros_services():
    """ Sets global data members tank and song"""
    global song
    global tank
    # obtain the tank service
    rospy.wait_for_service('tank') # won't continue until the "tank" service is on
    tank = rospy.ServiceProxy('tank', Tank) # tank permits requests, e.g., tank(0,50,50)
    
    # obtain the song service
    rospy.wait_for_service('song') # won't continue until the "song" service is on
    song = rospy.ServiceProxy('song', Song) # song permits requests, e.g., song(0,[69,24,70,24,71,24,72,80])



def resetValues():
    '''Resets the robot's information'''
    global data
    data["speed"] = 50
    data["state"] = "Keyboard"
    data["direction"] = "None"
    data["start_x"] = 0
    data["start_y"] = 0
    data["start_theta"] = 0
    data["odometry"] = odomClass.Odometry()
       
       
       
       
def listener():
    '''Sets up the stuff that the robot needs in order to publish and receive messages.'''
    global data
    global tank
    tank(0,0,0)
    data = {}
    resetValues()
    data["pub"] = rospy.Publisher('Odometry', String)
    rospy.init_node('publishOdometry')
    rospy.Subscriber('sensorPacket', SensorPacket, handle_sensor_data)
    rospy.Subscriber('GUI', String, handle_GUI_data)
    data["pub"].publish("reset")



    
if __name__ == '__main__':
    ros_services()
    try: listener()
    except rospy.ROSInterruptException:
        print "Connection Failed. Try Again."
        pass
    execute()
    rospy.spin()
    
