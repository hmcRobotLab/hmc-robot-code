#!/usr/bin/env python

import roslib; roslib.load_manifest('Frizzle')
import rospy
from std_msgs.msg import String
from iRobot import *
import sys
from mtTkinter import *
from turtle import *
import time
from PyQt4 import QtGui, QtCore

setup(640, 640, 0, 0) #Set the turtle window to be in the top left of the screen
shape("turtle")
color("green")

class robotWidget(QtGui.QWidget):
    ######################################
    #### Timer Event for updating     ####
    ####  things on screen            ####
    #### (for displaying Odometry)    ####
    ######################################
        
    def timerEvent(self, event):
        '''This will constantly be called because we have a timer widget
            constantly running behind the scenes in our GUI.'''
        self.lcd4.display(robot.odometry.current_theta)
        self.lcd5.display(robot.odometry.current_x)
        self.lcd6.display(robot.odometry.current_y)
        self.lcd7.display(robot.speed)
        self.lbl8.setText("<font style='color: black;background: white;'>" + robot.state + "</font>")
        robot.execute()
        setpos(robot.odometry.current_x*0.2, robot.odometry.current_y*0.2) #Move the turtle with the iRobot
        setheading(robot.odometry.current_theta) #Change the turtle's heading

        
    def __init__(self):
        self.col = QtGui.QColor(245, 245, 245)
        robot.state = "Keyboard"
        shape("turtle")
        color("green")
        super(robotWidget, self).__init__()
        
        hbox = QtGui.QHBoxLayout(self) # This is the main box that will hold our GUI.
        
        #######################################
        ##### Initializing Odometry LCDs,  ####
        ##### state text, and timer (which ####
        ##### we exploit in timerEvent     ####
        ##### as a secondary main loop)    ####
        #######################################
        
        self.lcd4 = QtGui.QLCDNumber(self)
        self.lcd5 = QtGui.QLCDNumber(self)
        self.lcd6 = QtGui.QLCDNumber(self)
        self.lbl8 = QtGui.QLabel(robot.state, self)
        self.lcd7 = QtGui.QLCDNumber(self)
        self.showNums = QtCore.QBasicTimer()
        self.showNums.start(0, self)
        
        

        ######################################
        ##### Setting up the top left box ####
        ##### (the box with the sliders)  ####
        ######################################
        
        sliderGrid = QtGui.QGridLayout()
        sliderGrid.setSpacing(0)
        sliderGrid.setColumnMinimumWidth(0, 50)
        sliderGrid.setColumnMinimumWidth(1, 255)
        
        title = QtGui.QLineEdit(self)
        title.setText("                                                              Background Color")
        title.setReadOnly(True)
        sliderGrid.addWidget(title, 1, 1)
       
       
        
        lcd1 = QtGui.QLCDNumber(self)
        self.sld1 = QtGui.QSlider(QtCore.Qt.Horizontal, self)
        lbl1 = QtGui.QLabel('Red', self)
        lbl1.setText("<font style='color: red;background: white;'>Red</font>")
        self.sld1.valueChanged.connect(lcd1.display)
        
        self.sld1.setMaximum(255)
        self.sld1.setValue(245)
        
        sliderGrid.addWidget(lcd1, 2, 2)
        sliderGrid.addWidget(self.sld1, 2, 1)
        sliderGrid.addWidget(lbl1, 2, 0)



        lcd2 = QtGui.QLCDNumber(self)
        self.sld2 = QtGui.QSlider(QtCore.Qt.Horizontal, self)
        lbl2 = QtGui.QLabel('Green', self)
        lbl2.setText("<font style='color: green;background: white;'>Green</font>")
        self.sld2.valueChanged.connect(lcd2.display)
        self.sld2.setMaximum(255)
        self.sld2.setValue(245)
        
        sliderGrid.addWidget(lcd2, 3, 2)
        sliderGrid.addWidget(self.sld2, 3, 1)
        sliderGrid.addWidget(lbl2, 3, 0)



        lcd3 = QtGui.QLCDNumber(self)
        self.sld3 = QtGui.QSlider(QtCore.Qt.Horizontal, self)
        lbl3 = QtGui.QLabel("Blue", self)
        lbl3.setText("<font style='color: blue;background: white;'>Blue</font>")
        self.sld3.valueChanged.connect(lcd3.display)
        self.sld3.setMaximum(255)
        self.sld3.setValue(245)
        
        sliderGrid.addWidget(lcd3, 4, 2)
        sliderGrid.addWidget(self.sld3, 4, 1)
        sliderGrid.addWidget(lbl3, 4, 0)



        lcd4 = QtGui.QLCDNumber(self)
        self.sld4 = QtGui.QSlider(QtCore.Qt.Horizontal, self)
        lbl4 = QtGui.QLabel("Hue", self)
        lbl4.setText("<font style='color: brown;background: white;'>Hue</font>")
        self.sld4.valueChanged.connect(lcd4.display)
        self.sld4.setMaximum(255)
        self.sld4.setValue(self.col.hue())
        
        sliderGrid.addWidget(lcd4, 5, 2)
        sliderGrid.addWidget(self.sld4, 5, 1)
        sliderGrid.addWidget(lbl4, 5, 0)
        
        
        
        lcd5 = QtGui.QLCDNumber(self)
        self.sld5 = QtGui.QSlider(QtCore.Qt.Horizontal, self)
        lbl5 = QtGui.QLabel("Saturation", self)
        lbl5.setText("<font style='color: grey;background: white;'>Saturation</font>")
        self.sld5.valueChanged.connect(lcd5.display)
        self.sld5.setMaximum(255)
        self.sld5.setValue(self.col.saturation())
        
        sliderGrid.addWidget(lcd5, 6, 2)
        sliderGrid.addWidget(self.sld5, 6, 1)
        sliderGrid.addWidget(lbl5, 6, 0)
        
        
        lcd6 = QtGui.QLCDNumber(self)
        self.sld6 = QtGui.QSlider(QtCore.Qt.Horizontal, self)
        lbl6 = QtGui.QLabel("Value", self)
        lbl6.setText("<font style='color: black;background: white;'>Value</font>")
        self.sld6.valueChanged.connect(lcd6.display)
        self.sld6.setMaximum(255)
        self.sld6.setValue(self.col.value())
        
        sliderGrid.addWidget(lcd6, 7, 2)
        sliderGrid.addWidget(self.sld6, 7, 1)
        sliderGrid.addWidget(lbl6, 7, 0)
        
        
        
        self.sld1.valueChanged.connect(self.sliderChange)
        self.sld2.valueChanged.connect(self.sliderChange)
        self.sld3.valueChanged.connect(self.sliderChange)
        self.sld4.valueChanged.connect(self.sliderChange)
        self.sld5.valueChanged.connect(self.sliderChange)
        self.sld6.valueChanged.connect(self.sliderChange)
        
        topleft = QtGui.QFrame(self)
        topleft.setLayout(sliderGrid)


        ###############################################
        #### Setting up the top right box          ####
        #### (Displays iRobot Odometry and state)  ####
        ###############################################

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
        
        
        ###### Row 1 Widgets  (Reset button, speed-controlling dial, and speed-displaying LCD) ######
        bottomGrid = QtGui.QGridLayout()
        bottomGrid.setVerticalSpacing(5)
        bottomGrid.setHorizontalSpacing(10)
        
        btn = QtGui.QPushButton('Push me to reset odometry!', self)
        btn.clicked.connect(self.buttonClicked)
        
        self.dial = QtGui.QDial(self)
        self.dial.setNotchesVisible(True)
        self.dial.setMaximum(200)
        self.dial.setValue(50)
        self.dial.valueChanged.connect(self.speedChange)
        
        lbl11 = QtGui.QLabel(self)
        lbl11.setText("<font style='color: black;background: white;'>Current Speed<br>(Control with Dial or 'Q' & 'E' keys!)</font>")
        
        
        
        ###### Row 2 Widgets (State-changing checkbox, display of keyboard controls, and background color-changing combo box) ######
        self.cb = QtGui.QCheckBox('Check me to change state!\nChecked: Square Movement state\n Unchecked: Keyboard state', self)
        self.cb.setPalette(QtGui.QPalette(QtGui.QColor("white")))
        self.cb.stateChanged.connect(self.changeState)
        
        lbl12 = QtGui.QLabel(self)
        lbl12.setText("<center><font style='color: black;background: white;'> Keyboard Controls:<br>WASD (forward, left, reverse, right)<br>Q (speed up), E (slow down), B (brake)<br>Esc (exit)</font></center>")

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
        
        
        
        ###### Row 3 Widgets (Code-input text line, code-sending button) ######
        lbl9 = QtGui.QLabel(self)
        lbl9.setText("<font style='color: black;background: white;'>Enter Code Here (Don't use any spaces!):<br><br>Below are examples of code you can enter<br>(Feel free to add your own!):</font>")
      
        self.qle = QtGui.QLineEdit(self)
        self.qle.returnPressed.connect(self.codeSend)
        
        codeSendBtn = QtGui.QPushButton('Send code to iRobot!', self)
        codeSendBtn.clicked.connect(self.codeSend)



        ###### Row 4 Widgets (Text box with code examples, and 3 user-defined buttons) ######
        ex1 = QtGui.QTextEdit(self)
        ex1.setText("tank(50,200)\n\ntank(data['speed'],data['speed'])\n\nZelda Item Get Tune:\nsong([69,70,71,72],[24,24,24,80])\n\nZelda Main Theme:\nsong([70,65,70,70,72,74,75,77,77,77,78,80,82,82,82,80,78,80,78,77],[32,48,16,8,8,8,8,64,16,12,12,12,64,16,12,12,12,32,16,64])\n\nZelda Skyward Sword Ballad of the Goddess Tune:\nsong([62,64,65,67,72,74,67,64,62,67,64,62,60,62,67,64],[80,16,16,64,28,64,28,64,80,28,64,16,16,64,28,64])")
        
        
        
        self.newBtn1 = QtGui.QPushButton('User-defined Button 1', self)
        self.newBtn2 = QtGui.QPushButton('User-defined Button 2', self)
        self.newBtn3 = QtGui.QPushButton('User-defined Button 3', self)
        
        self.newBtn1.clicked.connect(self.FSMButton)
        self.newBtn2.clicked.connect(self.FSMButton)
        self.newBtn3.clicked.connect(self.FSMButton)
        
        ###### Adding the Widgets ######
        bottomGrid.addWidget(btn, 1, 0)
        bottomGrid.addWidget(self.dial, 1, 1)
        bottomGrid.addWidget(self.lcd7, 1, 2)
        bottomGrid.addWidget(lbl11, 1, 3)
        
        bottomGrid.addWidget(self.cb, 2, 0)
        bottomGrid.addWidget(lbl12, 2, 1)
        bottomGrid.addWidget(combo, 2, 3)
        
        bottomGrid.addWidget(lbl9, 3, 0)
        bottomGrid.addWidget(self.qle, 3, 1, 1, 2)
        bottomGrid.addWidget(codeSendBtn, 3, 3)
        
        bottomGrid.addWidget(ex1, 4, 0)
        bottomGrid.addWidget(self.newBtn1, 4, 1)
        bottomGrid.addWidget(self.newBtn2, 4, 2)
        bottomGrid.addWidget(self.newBtn3, 4, 3)
        
        bottom = QtGui.QFrame(self)
        bottom.setLayout(bottomGrid)


        #########################################
        ####  Adding all the boxes and       ####
        ####  widgets together to the main   ####
        ####  GUI window via splitters...    ####
        #########################################
        
        splitter1 = QtGui.QSplitter(QtCore.Qt.Horizontal)
        splitter1.addWidget(topleft)
        splitter1.addWidget(topright)

        splitter2 = QtGui.QSplitter(QtCore.Qt.Vertical)
        splitter2.addWidget(splitter1)
        splitter2.addWidget(bottom)
        splitter2.setStretchFactor(0, 6)
        splitter2.setStretchFactor(1, 3)

        hbox.addWidget(splitter2)
        self.setLayout(hbox)
        QtGui.QApplication.setStyle(QtGui.QStyleFactory.create('Cleanlooks'))
        
        self.setGeometry(900, 0, 1024, 640) #Set the GUI window to be in the top right of the screen
        self.setWindowTitle('iRobot Control Panel')
        self.show()

    
    #################################
    ####  Functions that define  ####
    ####  widget behavior        ####
    #################################
    def sliderChange(self, newValue):
        '''If a slider is moved, then the background's RGB values will be changed accordingly'''
        if self.sender() == self.sld1:
            self.col.setRed(newValue)
            
        if self.sender() == self.sld2:
            self.col.setGreen(newValue)
            
        if self.sender() == self.sld3:
            self.col.setBlue(newValue)
            
        if self.sender() == self.sld4:
            self.col.setHsv(newValue, self.col.saturation(), self.col.value())
        
        if self.sender() == self.sld5:
            self.col.setHsv(self.col.hue(), newValue, self.col.value())
            
        if self.sender() == self.sld6:
            self.col.setHsv(self.col.hue(), self.col.saturation(), newValue)
            
        self.setStyleSheet("QFrame { background-color: %s }" % self.col.name())
        self.setStyle(QtGui.QStyleFactory.create('Cleanlooks'))
        
            
    def buttonClicked(self):
        '''If the reset button is clicked, then we reset stuff.'''
        self.showNums.stop()
        robot.__init__()
        self.dial.setValue(50)
        reset()
        shape("turtle")
        color("green")
        self.showNums.start(0, self)
        time.sleep(1)
        robot.__init__()
    
    def FSMButton(self):
        robot.setStartCoordinates()
        if self.sender() == self.newBtn1:
            robot.state = "Button 1"
        if self.sender() == self.newBtn2:
            robot.state = "Button 2"
        if self.sender() == self.newBtn3:
            robot.state = "Button 3"
            
    def codeSend(self):
        '''If the code-sending button is clicked, then send the code to the robot'''
        robot.code = str(self.qle.text())
        robot.state = "Code"
        self.qle.clear()
        
    def keyPressEvent(self, e):
        '''If a keyboard key is pressed, then respond accordingly'''
        if e.key() == QtCore.Qt.Key_Escape: #if the escape key is pressed, then exit
            robot.state = "Stop"
            self.close()
            
        elif e.key() == 87: #If 'W' is pressed, then tell the robot to move forward
            robot.state = "Keyboard"
            print "Moving forward"
            robot.action = 'w'
            
        elif e.key() == 83: #If 'S' is pressed, then tell the robot to move backward
            robot.state = "Keyboard"
            print "Moving backward"
            robot.action = 's'
            
        elif e.key() == 68: #If 'D' is pressed, then tell the robot to rotate clockwise
            robot.state = "Keyboard"
            print "Rotating right"
            robot.action = 'd'
            
        elif e.key() == 65: #If 'A' is pressed, then tell the robot to rotate counter-clockwise
            robot.state = "Keyboard"
            print "Rotating left"
            robot.action = 'a'
            
        elif e.key() == 81: #If 'Q' is pressed, then tell the robot to speed up
            if (robot.speed + 10) <= 500:
                robot.speed += 10
                print "Speeding up. Speed is now", robot.speed
                if robot.speed >= 0 and  robot.speed <= 200:
                    self.dial.setValue(robot.speed)
                elif robot.speed > 200:
                    self.dial.setValue(200)
                elif robot.speed < 0:
                    self.dial.setValue(0)
            return
            
        elif e.key() == 69: #If 'E' is pressed, then tell the robot to slow down
            if (robot.speed - 10) >= -500:
                robot.speed -= 10
                print "Slowing down. Speed is now", robot.speed
                if robot.speed >= 0 and robot.speed <= 200:
                    self.dial.setValue(robot.speed)
                elif robot.speed > 200:
                    self.dial.setValue(200)
                elif robot.speed < 0:
                    self.dial.setValue(0)
            return
            
        elif e.key() == 66: #If 'B' is pressed, then tell the robot to halt
            robot.state = "Keyboard"
            print "Braking"
            robot.action = "b"
            
        else: return #Otherwise, do nothing.
        self.cb.setChecked(False)
        
        
        
    def speedChange(self, newSpeed):
        '''If the speed-controlling dial is turned, then change the speed value accordingly.'''
        robot.speed = newSpeed
            
    def changeState(self, check):
        '''If the checkbox is clicked, then change the state accordingly.'''
        if check == QtCore.Qt.Checked:
            robot.state = 'Square Movement'
        else:
            robot.state = 'Keyboard'
            self.cb.setChecked(False)

    def onActivated(self, text):
        '''If a value in the combo box is clicked, then change the background color accordingly.'''
        if text == "Red Background":
            self.col = QtGui.QColor(255, 0, 0)
        elif text == "Orange Background":
            self.col = QtGui.QColor(255, 127, 0)
        elif text == "Yellow Background":
            self.col = QtGui.QColor(255, 255, 0)
        elif text == "Green Background":
            self.col = QtGui.QColor(0, 255, 0)
        elif text == "Blue Background":
            self.col = QtGui.QColor(0, 0, 255)
        elif text == "Indigo Background":
            self.col = QtGui.QColor(111, 0, 255)
        elif text == "Purple Background":
            self.col = QtGui.QColor(143, 0, 255)
        elif text == "Default Background Color":
            self.col = QtGui.QColor(245, 245, 245)
        else:
            if text == "Select Background from File":
                fname = QtGui.QFileDialog.getOpenFileName(self, 'Open file', '/home')
                if fname == '': #If the user presses 'cancel', then do nothing
                    return
                self.setStyleSheet("width: 10%;background-image: url(" + fname + ");}")
                self.setStyle(QtGui.QStyleFactory.create('Cleanlooks'))
            return
            
        self.sld1.setValue(self.col.red())
        self.sld2.setValue(self.col.green())
        self.sld3.setValue(self.col.blue())
        self.sld4.setValue(self.col.hue())
        self.sld5.setValue(self.col.saturation())
        self.sld6.setValue(self.col.value())
        self.setStyleSheet("QFrame { background-color: %s }" % self.col.name())
        self.setStyle(QtGui.QStyleFactory.create('Cleanlooks'))


    
if __name__ == '__main__':
    robot = iRobot()
    app = QtGui.QApplication(sys.argv)
    widget = robotWidget()
    sys.exit(app.exec_())
    
