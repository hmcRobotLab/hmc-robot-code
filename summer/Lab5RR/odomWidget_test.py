#!/usr/bin/env python2.7

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
import random
import sys
from PyQt4 import QtGui, QtCore
setup(640, 640, 0, 0)

def handle_sensor_data( data ):
    """
        handle_sensor_data is called every time the robot gets a new sensorPacket
    """
    global odometry
    odometry = odometry.updateOdometry(data)

def handle_key_data(data):
    global state
    global odometry
    global quitter
    global tank
    if data.data == chr(27): # if a 'q' or Esc was pressed
        print 'quitting'
        tank(0, 0, 0)
        quitter = True

    if data.data == 'r':
        print "Resetting"
        odometry = odomClass.Odometry()

    if data.data != '':
        state = "Keyboard"
        
        
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
    '''This function will work on its own as the main loop function,
        but we can't use it with the PyQt stuff'''
    global odometry
    global tank
    square = False
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


class robotWidget(QtGui.QWidget):
    global quitter
    quitter = False
    ######################################
    #### Timer Event for updating     ####
    ####  things on screen            ####
    #### (for displaying Odometry)    ####
    ######################################
    def paintEvent(self, e):
        global odometry
        qp = QtGui.QPainter()
        qp.begin(self)
        pen = QtGui.QPen(QtCore.Qt.red, 5, QtCore.Qt.SolidLine, QtCore.Qt.RoundCap)
        qp.setPen(pen)
        qp.end()
        
    def timerEvent(self, event):
        '''This will constantly be called because we have a timer widget
            constantly running behind the scenes in our GUI.'''
        global odometry
        global state
        global tank
        global distance
        global start_x
        global start_y
        global start_theta
        global quitter
        if quitter == True:
            bye()
            self.close()
            return
        shape("turtle")
        color("green")
        self.lcd4.display(odometry.current_theta)
        self.lcd5.display(odometry.current_x)
        self.lcd6.display(odometry.current_y)
        self.lcd7.display(self.speed)
        self.lbl8.setText("<font style='color: black;background: white;'>" + state + "</font>")
        if state == "Square Movement":
                if distance < 7000: #Until we've drawn a full side...
                    distance += math.hypot(odometry.current_x- start_x, odometry.current_y - start_y) #Add to the length of the side
                    setpos(odometry.current_x, odometry.current_y) #Move the turtle with the iRobot
                    setheading(odometry.current_theta) #Change the turtle's heading
                    tank(0, self.speed, self.speed) #Move the iRobot forward
                    self.lbl8.setText("<font style='color: black;background: white;'>" + state + "</font>")
                if (distance >= 7000) and (odometry.current_theta < (start_theta + 90)): #Until we've turned 90 degrees...
                    setpos(odometry.current_x, odometry.current_y) #Move the turtle with the iRobot
                    setheading(odometry.current_theta) #Change the turtle's heading
                    tank(0, -self.speed, self.speed) #Rotate the iRobot
                    self.lbl8.setText("<font style='color: black;background: white;'>" + state + "</font>")
                if (distance >= 7000) and (odometry.current_theta >= (start_theta + 90)):
                    distance = 0
                    start_x = odometry.current_x
                    start_y = odometry.current_y
                    start_theta = odometry.current_theta
        else:
            setpos(odometry.current_x, odometry.current_y) #Move the turtle with the iRobot
            setheading(odometry.current_theta) #Change the turtle's heading
            self.cb.setChecked(False)

        
    def __init__(self):
        global state
        state = "Keyboard"
        self.speed = 50
        super(robotWidget, self).__init__()
        #######################################
        ##### Initializing Odometry LCDs,  ####
        ##### state text, and timer (which ####
        ##### we exploit in timerEvent     ####
        ##### as a secondary main loop)    ####
        #######################################
        
        self.lcd4 = QtGui.QLCDNumber(self)
        self.lcd5 = QtGui.QLCDNumber(self)
        self.lcd6 = QtGui.QLCDNumber(self)
        self.lbl8 = QtGui.QLabel(state, self)
        self.lcd7 = QtGui.QLCDNumber(self)
        self.showNums = QtCore.QBasicTimer()
        self.showNums.start(0, self)
        
        hbox = QtGui.QHBoxLayout(self) # This is the main box that will hold our GUI.

        ######################################
        ##### Setting up the top left box ####
        ##### (the box with the sliders)  ####
        ######################################
        
        sliderGrid = QtGui.QGridLayout()
        sliderGrid.setSpacing(0)
        sliderGrid.setColumnMinimumWidth(0, 50)
        sliderGrid.setColumnMinimumWidth(1, 400)
        
        lcd1 = QtGui.QLCDNumber(self)
        sld1 = QtGui.QSlider(QtCore.Qt.Horizontal, self)
        lbl1 = QtGui.QLabel('Red', self)
        lbl1.setText("<font style='color: red;background: white;'>Red</font>")
        sld1.setMaximum(255)
        sld1.valueChanged.connect(lcd1.display)
        def setValue1(rvalue):
            global r
            r = rvalue
        self.connect(sld1, QtCore.SIGNAL('valueChanged(int)'), setValue1)
        
        sliderGrid.addWidget(lcd1, 1, 2)
        sliderGrid.addWidget(sld1, 1, 1)
        sliderGrid.addWidget(lbl1, 1, 0)



        lcd2 = QtGui.QLCDNumber(self)
        sld2 = QtGui.QSlider(QtCore.Qt.Horizontal, self)
        lbl2 = QtGui.QLabel('Green', self)
        lbl2.setText("<font style='color: green;background: white;'>Green</font>")
        sld2.setMaximum(255)
        sld2.valueChanged.connect(lcd2.display)
        def setValue2(gvalue):
            global g
            g = gvalue
        self.connect(sld2, QtCore.SIGNAL('valueChanged(int)'), setValue2)
        
        sliderGrid.addWidget(lcd2, 2, 2)
        sliderGrid.addWidget(sld2, 2, 1)
        sliderGrid.addWidget(lbl2, 2, 0)



        lcd3 = QtGui.QLCDNumber(self)
        sld3 = QtGui.QSlider(QtCore.Qt.Horizontal, self)
        lbl3 = QtGui.QLabel("Blue", self)
        lbl3.setText("<font style='color: blue;background: white;'>Blue</font>")
        sld3.setMaximum(255)
        sld3.valueChanged.connect(lcd3.display)
        def setValue3(bvalue):
            global b
            b = bvalue
        self.connect(sld3, QtCore.SIGNAL('valueChanged(int)'), setValue3)

        sliderGrid.addWidget(lcd3, 3, 2)
        sliderGrid.addWidget(sld3, 3, 1)
        sliderGrid.addWidget(lbl3, 3, 0)


        
        topleft = QtGui.QFrame(self)
        topleft.setLayout(sliderGrid)


        ######################################
        #### Setting up the top right box ####
        #### (Displays iRobot Odometry)   ####
        ######################################

        odomGrid = QtGui.QGridLayout()
        odomGrid.setSpacing(30)


        lbl4 = QtGui.QLabel("Theta", self)
        lbl4.setText("<font style='color: black;background: white;'>Theta</font>")
        odomGrid.addWidget(lbl4, 1, 0)
        odomGrid.addWidget(self.lcd4, 1, 1)


        lbl5 = QtGui.QLabel("X Coordinate", self)
        lbl5.setText("<font style='color: black;background: white;'>X Coordinate</font>")
        odomGrid.addWidget(lbl5, 2, 0)
        odomGrid.addWidget(self.lcd5, 2, 1)


        lbl6 = QtGui.QLabel("Y Coordinate", self)
        lbl6.setText("<font style='color: black;background: white;'>Y Coordinate</font>")
        odomGrid.addWidget(lbl6, 3, 0)
        odomGrid.addWidget(self.lcd6, 3, 1)
        

        lbl7 = QtGui.QLabel("Current State:", self)
        lbl7.setText("<font style='color: black;background: white;'>Current State:</font>")
        
        
        odomGrid.addWidget(lbl7, 4, 0)
        odomGrid.addWidget(self.lbl8, 4, 1)
        
        
        topright = QtGui.QFrame(self)
        topright.setLayout(odomGrid)


        ######################################
        #### Setting up the bottom box    ####
        #### (Random widget examples!)    ####
        ######################################
        
        bottomGrid = QtGui.QGridLayout()
        bottomGrid.setSpacing(15)
        
        btn = QtGui.QPushButton('Push me to reset odometry!', self)
        btn.clicked.connect(self.buttonClicked)
        
        self.cb = QtGui.QCheckBox('Check me to change state!', self)
        self.cb.setPalette(QtGui.QPalette(QtGui.QColor("white")))

        self.cb.stateChanged.connect(self.changeState)

        lbl9 = QtGui.QLabel(self)
        lbl9.setText("<font style='color: black;background: white;'>Enter Text Here<br>(it appears on the right!):</font>")

        self.lbl10 = QtGui.QLabel(self)

        qle = QtGui.QLineEdit(self)
        qle.textChanged[str].connect(self.onChanged)

        combo = QtGui.QComboBox(self)
        combo.addItem("Default Background Color")
        combo.addItem("Red Background")
        combo.addItem("Orange Background")
        combo.addItem("Yellow Background")
        combo.addItem("Green Background")
        combo.addItem("Blue Background")
        combo.addItem("Indigo Background")
        combo.addItem("Purple Background")
        combo.addItem("Select Background from File")
        combo.activated[str].connect(self.onActivated)

        dial = QtGui.QDial(self)
        dial.setNotchesVisible(True)
        dial.setMaximum(200)
        dial.valueChanged.connect(self.speedChange)

        lbl11 = QtGui.QLabel(self)
        lbl11.setText("<font style='color: black;background: white;'>Current Speed<br>(Control with Dial!)</font>")

        lbl12 = QtGui.QLabel(self)
        lbl12.setText("<center><font style='color: black;background: white;'> Keyboard Controls:<br>WASD (forward, left, reverse, right)<br>Q (speed up)<br>E (slow down)<br>Esc (exit)<br>Press any other key to brake</font></center>")
        
        bottomGrid.addWidget(btn, 1, 0)
        bottomGrid.addWidget(self.cb, 2, 0)
        bottomGrid.addWidget(lbl9, 3, 0)
        bottomGrid.addWidget(qle, 3, 1)
        bottomGrid.addWidget(self.lbl10, 3, 2)
        bottomGrid.addWidget(combo, 4, 0)
        bottomGrid.addWidget(lbl11, 1, 3)
        bottomGrid.addWidget(self.lcd7, 1, 2)
        bottomGrid.addWidget(dial, 1, 1)
        bottomGrid.addWidget(lbl12, 2, 1)
        
        bottom = QtGui.QFrame(self)
        bottom.setLayout(bottomGrid)


        ######################################
        ####  Adding all the boxes and    ####
        ####  widgets together to the     ####
        ####     main GUI window...       ####
        ######################################
        
        splitter1 = QtGui.QSplitter(QtCore.Qt.Horizontal)
        splitter1.addWidget(topleft)
        splitter1.addWidget(topright)

        splitter2 = QtGui.QSplitter(QtCore.Qt.Vertical)
        splitter2.addWidget(splitter1)
        splitter2.addWidget(bottom)

        hbox.addWidget(splitter2)
        self.setLayout(hbox)
        QtGui.QApplication.setStyle(QtGui.QStyleFactory.create('Cleanlooks'))
        
        self.setGeometry(900, 0, 1000, 640)
        self.setWindowTitle('Robot Control Panel')
        self.show()

                      
    def buttonClicked(self):
        global odometry
        odometry = odomClass.Odometry()
        reset()
        shape("turtle")
        color("green")
        
    def keyPressEvent(self, e):
        global quitter
        global tank
        if e.key() == QtCore.Qt.Key_Escape:
            quitter = True
            tank(0, 0, 0)
            bye()
            self.close()
        if e.key() == 87:
            print "Forward"
            tank(0, self.speed, self.speed)
        elif e.key() == 83:
            print "Reverse"
            tank(0, -self.speed, -self.speed)
        elif e.key() == 68:
            print "Right"
            tank(0, self.speed, -self.speed)
        elif e.key() == 65:
            print "Left"
            tank(0, -self.speed, self.speed)
        elif e.key() == 81:
            print "Speeding Up"
            self.speed += 20
        elif e.key() == 69:
            print "Slowing Down"
            self.speed -= 20
        
        else:
            print "Braking"
            print e.key()
            tank(0, 0, 0)
        self.cb.setChecked(False)
        
        
    def speedChange(self, num):
            print num
            self.speed = num
            
    def changeState(self, check):
        global state
        global distance
        global start_x
        global start_y
        global start_theta
        global tank
        if check == QtCore.Qt.Checked:
            state = 'Square Movement'
            distance = 0
            print "GO"
            start_x = odometry.current_x #Set the starting coordinates
            start_y = odometry.current_y #Set the starting coordinates
            start_theta = odometry.current_theta
        else:
            state = 'Keyboard'
            self.cb.setChecked(False)
            tank(0, 0, 0)

    def onActivated(self, text):
        if text == "Red Background":
            col = QtGui.QColor(255, 0, 0)
        elif text == "Orange Background":
            col = QtGui.QColor(255, 127, 0)
        elif text == "Yellow Background":
            col = QtGui.QColor(255, 255, 0)
        elif text == "Green Background":
            col = QtGui.QColor(0, 255, 0)
        elif text == "Blue Background":
            col = QtGui.QColor(0, 0, 255)
        elif text == "Indigo Background":
            col = QtGui.QColor(111, 0, 255)
        elif text == "Purple Background":
            col = QtGui.QColor(143, 0, 255)
        elif text == "Default Background Color":
            col = QtGui.QColor(245, 245, 245)
        else:
            if text == "Select Background from File":
                fname = QtGui.QFileDialog.getOpenFileName(self, 'Open file', '/home')
                if fname == '': #If the user presses 'cancel', then do nothing
                    return
                self.setStyleSheet("width: 10%;background-image: url(" + fname + ");}")
                self.setStyle(QtGui.QStyleFactory.create('Cleanlooks'))
            return
        self.setStyleSheet("QFrame { background-color: %s }" % col.name())
        self.setStyle(QtGui.QStyleFactory.create('Cleanlooks'))
        
    def onChanged(self, text):
        self.lbl10.setText("<font style='color: black;background: white;'>" + text + "</font>")
        self.lbl10.adjustSize()


    
if __name__ == '__main__':
    listener()
    ros_services()
    app = QtGui.QApplication(sys.argv)
    widget = robotWidget()
    sys.exit(app.exec_())
    
