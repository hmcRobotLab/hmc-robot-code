#!/usr/bin/env python
from GUIardrone import *
from sliderImageProcessor import *
import time
from PyQt4 import QtGui, QtCore

class robotWidget(QtGui.QWidget):
    ######################################
    #### Timer Event for updating     ####
    ####  things on screen            ####
    #### (for displaying Odometry)    ####
    ######################################
        
    def timerEvent(self, event):
        '''This will constantly be called because we have a timer widget
            constantly running behind the scenes in our GUI.'''
        self.altitude.display(drone.altitude)
        self.phi.display(drone.phi)
        self.psi.display(drone.psi)
        self.theta.display(drone.theta)
        self.vx.display(drone.vx)
        self.vy.display(drone.vy)
        self.vz.display(drone.vz)
        self.batLevel.display(drone.batLevel)
        self.ctrlState.display(drone.ctrlState)
        self.lbl8.setText("<font size='5'>" + self.state + "</font>")

        
    def __init__(self):
        self.state = "Keyboard"
        self.col = QtGui.QColor(245, 245, 245)
        super(robotWidget, self).__init__()
        
        hbox = QtGui.QHBoxLayout(self) # This is the main box that will hold our GUI.
        
        ########################################
        ##### Initializing Drone Data LCDs, ####
        ##### state text, and timer (which  ####
        ##### we exploit in timerEvent      ####
        ##### as a secondary main loop)     ####
        ########################################
        self.altitude  = QtGui.QLCDNumber(self)
        self.phi       = QtGui.QLCDNumber(self)
        self.psi       = QtGui.QLCDNumber(self)
        self.theta     = QtGui.QLCDNumber(self)
        self.vx        = QtGui.QLCDNumber(self)
        self.vy        = QtGui.QLCDNumber(self)
        self.vz        = QtGui.QLCDNumber(self)
        self.batLevel  = QtGui.QLCDNumber(self)
        self.ctrlState = QtGui.QLCDNumber(self)
        self.lbl8 = QtGui.QLabel(self.state, self)

        self.showData = QtCore.QBasicTimer()
        self.showData.start(1000, self)

        self.showImage = QtCore.QTimer()
        self.showImage.setInterval(0)
        self.showImage.start()
        self.showImage.timeout.connect(self.displayImage)

        
        

        ######################################
        ##### Setting up the top left box ####
        ##### (the box with the sliders)  ####
        ######################################
        
        sliderGrid = QtGui.QGridLayout()
        sliderGrid.setSpacing(0)
        sliderGrid.setColumnMinimumWidth(0, 50)
        sliderGrid.setColumnMinimumWidth(1, 255)
           
        
        lcd1 = QtGui.QLCDNumber(self)
        self.sld1 = QtGui.QSlider(QtCore.Qt.Horizontal, self)
        lbl1 = QtGui.QLabel('Red', self)
        lbl1.setText("<font style='color: red;background: white;'>Red</font>")
        self.sld1.valueChanged.connect(lcd1.display)
        
        self.sld1.setMaximum(255)
        
        sliderGrid.addWidget(lcd1, 1, 2)
        sliderGrid.addWidget(self.sld1, 1, 1)
        sliderGrid.addWidget(lbl1, 1, 0)



        lcd2 = QtGui.QLCDNumber(self)
        self.sld2 = QtGui.QSlider(QtCore.Qt.Horizontal, self)
        lbl2 = QtGui.QLabel('Green', self)
        lbl2.setText("<font style='color: green;background: white;'>Green</font>")
        self.sld2.valueChanged.connect(lcd2.display)
        self.sld2.setMaximum(255)
        
        sliderGrid.addWidget(lcd2, 2, 2)
        sliderGrid.addWidget(self.sld2, 2, 1)
        sliderGrid.addWidget(lbl2, 2, 0)



        lcd3 = QtGui.QLCDNumber(self)
        self.sld3 = QtGui.QSlider(QtCore.Qt.Horizontal, self)
        lbl3 = QtGui.QLabel("Blue", self)
        lbl3.setText("<font style='color: blue;background: white;'>Blue</font>")
        self.sld3.valueChanged.connect(lcd3.display)
        self.sld3.setMaximum(255)
        
        sliderGrid.addWidget(lcd3, 3, 2)
        sliderGrid.addWidget(self.sld3, 3, 1)
        sliderGrid.addWidget(lbl3, 3, 0)



        lcd4 = QtGui.QLCDNumber(self)
        self.sld4 = QtGui.QSlider(QtCore.Qt.Horizontal, self)
        lbl4 = QtGui.QLabel("Hue", self)
        lbl4.setText("<font style='color: brown;background: white;'>Hue</font>")
        self.sld4.valueChanged.connect(lcd4.display)
        self.sld4.setMaximum(255)
        
        sliderGrid.addWidget(lcd4, 4, 2)
        sliderGrid.addWidget(self.sld4, 4, 1)
        sliderGrid.addWidget(lbl4, 4, 0)
        
        
        
        lcd5 = QtGui.QLCDNumber(self)
        self.sld5 = QtGui.QSlider(QtCore.Qt.Horizontal, self)
        lbl5 = QtGui.QLabel("Saturation", self)
        lbl5.setText("<font style='color: grey;background: white;'>Saturation</font>")
        self.sld5.valueChanged.connect(lcd5.display)
        self.sld5.setMaximum(255)
        
        sliderGrid.addWidget(lcd5, 5, 2)
        sliderGrid.addWidget(self.sld5, 5, 1)
        sliderGrid.addWidget(lbl5, 5, 0)
        
        
        lcd6 = QtGui.QLCDNumber(self)
        self.sld6 = QtGui.QSlider(QtCore.Qt.Horizontal, self)
        lbl6 = QtGui.QLabel("Value", self)
        lbl6.setText("<font style='color: black;background: white;'>Value</font>")
        self.sld6.valueChanged.connect(lcd6.display)
        self.sld6.setMaximum(255)
        
        sliderGrid.addWidget(lcd6, 6, 2)
        sliderGrid.addWidget(self.sld6, 6, 1)
        sliderGrid.addWidget(lbl6, 6, 0)
        
        

        lcd7 = QtGui.QLCDNumber(self)
        self.sld7 = QtGui.QSlider(QtCore.Qt.Horizontal, self)
        lbl7 = QtGui.QLabel('High Red', self)
        lbl7.setText("<font style='color: red;background: white;'>High Red</font>")

        self.sld7.valueChanged.connect(lcd7.display)
        
        self.sld7.setMaximum(255)
        
        sliderGrid.addWidget(lcd7, 7, 2)
        sliderGrid.addWidget(self.sld7, 7, 1)
        sliderGrid.addWidget(lbl7, 7, 0)



        lcd8 = QtGui.QLCDNumber(self)
        self.sld8 = QtGui.QSlider(QtCore.Qt.Horizontal, self)
        lbl9 = QtGui.QLabel('High Green', self)
        lbl9.setText("<font style='color: green;background: white;'>High Green</font>")
        self.sld8.valueChanged.connect(lcd8.display)
        self.sld8.setMaximum(255)
        
        sliderGrid.addWidget(lcd8, 8, 2)
        sliderGrid.addWidget(self.sld8, 8, 1)
        sliderGrid.addWidget(lbl9, 8, 0)



        lcd9 = QtGui.QLCDNumber(self)
        self.sld9 = QtGui.QSlider(QtCore.Qt.Horizontal, self)
        lbl10 = QtGui.QLabel("High Blue", self)
        lbl10.setText("<font style='color: blue;background: white;'>High Blue</font>")
        self.sld9.valueChanged.connect(lcd9.display)
        self.sld9.setMaximum(255)
        
        sliderGrid.addWidget(lcd9, 9, 2)
        sliderGrid.addWidget(self.sld9, 9, 1)
        sliderGrid.addWidget(lbl10, 9, 0)



        lcd10 = QtGui.QLCDNumber(self)
        self.sld10 = QtGui.QSlider(QtCore.Qt.Horizontal, self)
        lbl11 = QtGui.QLabel("High Hue", self)
        lbl11.setText("<font style='color: brown;background: white;'>High Hue</font>")
        self.sld10.valueChanged.connect(lcd10.display)
        self.sld10.setMaximum(255)
        
        sliderGrid.addWidget(lcd10, 10, 2)
        sliderGrid.addWidget(self.sld10, 10, 1)
        sliderGrid.addWidget(lbl11, 10, 0)
        
        
        
        lcd11 = QtGui.QLCDNumber(self)
        self.sld11 = QtGui.QSlider(QtCore.Qt.Horizontal, self)
        lbl13 = QtGui.QLabel("High Saturation", self)
        lbl13.setText("<font style='color: grey;background: white;'>High Saturation</font>")
        self.sld11.valueChanged.connect(lcd11.display)
        self.sld11.setMaximum(255)
        
        sliderGrid.addWidget(lcd11, 11, 2)
        sliderGrid.addWidget(self.sld11, 11, 1)
        sliderGrid.addWidget(lbl13, 11, 0)
        
        
        lcd12 = QtGui.QLCDNumber(self)
        self.sld12 = QtGui.QSlider(QtCore.Qt.Horizontal, self)
        lbl14 = QtGui.QLabel("High Value", self)
        lbl14.setText("<font style='color: black;background: white;'>High Value</font>")
        self.sld12.valueChanged.connect(lcd12.display)
        self.sld12.setMaximum(255)
        
        sliderGrid.addWidget(lcd12, 12, 2)
        sliderGrid.addWidget(self.sld12, 12, 1)
        sliderGrid.addWidget(lbl14, 12, 0)
        
        self.sld1.valueChanged.connect(self.sliderChange)
        self.sld2.valueChanged.connect(self.sliderChange)
        self.sld3.valueChanged.connect(self.sliderChange)
        self.sld4.valueChanged.connect(self.sliderChange)
        self.sld5.valueChanged.connect(self.sliderChange)
        self.sld6.valueChanged.connect(self.sliderChange)
        self.sld7.valueChanged.connect(self.sliderChange)
        self.sld8.valueChanged.connect(self.sliderChange)
        self.sld9.valueChanged.connect(self.sliderChange)
        self.sld10.valueChanged.connect(self.sliderChange)
        self.sld11.valueChanged.connect(self.sliderChange)
        self.sld12.valueChanged.connect(self.sliderChange)

        top = QtGui.QFrame(self)
        top.setLayout(sliderGrid)
        
        
        ###########################################
        #### Setting up the bottom-left box    ####
        ###########################################
        
        
        ###### Row 1 Widgets  (Reset button, speed-controlling dial, and speed-displaying LCD) ######
        bottomGrid = QtGui.QGridLayout()
        bottomGrid.setVerticalSpacing(5)
        bottomGrid.setHorizontalSpacing(10)
        
       
        

        
        lbl12 = QtGui.QLabel(self)
        lbl12.setText("<center><font size = '3';font style='color: black;'> \
Keyboard Controls:<br><br> \
h: quit<br> enter/return: land <br> t: takeoff<br> r: reset<br> \
$: save color thresholds <br> &: load color thresholds<br>\
q: forward left strafe \
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;\
w: forward \
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;\
e: forward right strafe<br> \
a: strafe left \
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;\
s: hover: \
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;\
d: strafe right<br> \
z: backward left strafe \
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;\
x: backward \
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;\
e: backward right strafe<br> \
f: spin left<br> g: spin right<br> Esc (exit) </font></center><br>")

        takeoffBtn = QtGui.QPushButton('Takeoff!', self)
        takeoffBtn.clicked.connect(self.takeoff)



        ###### Row 4 Widgets (Text box with code examples, and 3 user-defined buttons) ######
           
        landingBtn = QtGui.QPushButton('Land!', self)
        newBtn2 = QtGui.QPushButton('User-defined Button 2', self)
        newBtn3 = QtGui.QPushButton('User-defined Button 3', self)
        
        landingBtn.clicked.connect(self.landing)
        newBtn2.clicked.connect(self.btn2Function)
        newBtn3.clicked.connect(self.btn3Function)
        
        ###### Adding the Widgets ######
        bottomGrid.addWidget(lbl12, 1, 0, 4, 1)
        bottomGrid.addWidget(takeoffBtn, 1, 3)
        
        bottomGrid.addWidget(landingBtn, 2, 3)
        bottomGrid.addWidget(newBtn2, 3, 3)
        bottomGrid.addWidget(newBtn3, 4, 3)
        
        bottom = QtGui.QFrame(self)
        bottom.setLayout(bottomGrid)


        #########################################
        ####  Right side (Drone data)        ####
        #########################################
        droneData = QtGui.QGridLayout()
        droneData.setSpacing(10)

        altitude = QtGui.QLabel("Altitude", self)
        droneData.addWidget(altitude, 1, 0)
        droneData.addWidget(self.altitude, 1, 1)

        phi = QtGui.QLabel("Phi", self)
        droneData.addWidget(phi, 2, 0)
        droneData.addWidget(self.phi, 2, 1)

        psi = QtGui.QLabel("Psi", self)
        droneData.addWidget(psi, 3, 0)
        droneData.addWidget(self.psi, 3, 1)

        theta = QtGui.QLabel("Theta", self)
        droneData.addWidget(theta, 4, 0)
        droneData.addWidget(self.theta, 4, 1)

        vx = QtGui.QLabel("X Velocity", self)
        droneData.addWidget(vx, 5, 0)
        droneData.addWidget(self.vx, 5, 1)

        vy = QtGui.QLabel("Y Velocity", self)
        droneData.addWidget(vy, 6, 0)
        droneData.addWidget(self.vy, 6, 1)

        vz = QtGui.QLabel("Z Velocity", self)
        droneData.addWidget(vz, 7, 0)
        droneData.addWidget(self.vz, 7, 1)

        batLevel = QtGui.QLabel("Battery Level", self)
        droneData.addWidget(batLevel, 8, 0)
        droneData.addWidget(self.batLevel, 8, 1)
        
        ctrlState = QtGui.QLabel("Control State", self)
        droneData.addWidget(ctrlState, 9, 0)
        droneData.addWidget(self.ctrlState, 9, 1)

        state = QtGui.QLabel("Current State:", self) 
        droneData.addWidget(state, 10, 0)
        droneData.addWidget(self.lbl8, 10, 1)

        right = QtGui.QFrame(self)
        right.setLayout(droneData)

        #########################################
        ####  Adding all the boxes and       ####
        ####  widgets together to the main   ####
        ####  GUI window via splitters...    ####
        #########################################
        
        splitter2 = QtGui.QSplitter(QtCore.Qt.Vertical)
        splitter2.addWidget(top)
        splitter2.addWidget(bottom)
        splitter2.setStretchFactor(0, 6)
        splitter2.setStretchFactor(1, 3)

        splitter = QtGui.QSplitter(QtCore.Qt.Horizontal)
        splitter.addWidget(splitter2)
        splitter.addWidget(right)

        hbox.addWidget(splitter)
        self.setLayout(hbox)
        QtGui.QApplication.setStyle(QtGui.QStyleFactory.create('Cleanlooks'))
        self.setGeometry(900, 0, 1024, 640) #Set the GUI window to be in the top right of the screen
        self.setWindowTitle('Drone Control Panel')
        self.show()

    
    #################################
    ####  Functions that define  ####
    ####  widget behavior        ####
    #################################
    def sliderChange(self, newValue):
        '''If a slider is moved, then the background's RGB values will be changed accordingly'''
        if self.sender() == self.sld1:
            image.change_low_red(newValue)
            
        if self.sender() == self.sld2:
            image.change_low_green(newValue)
            
        if self.sender() == self.sld3:
            image.change_low_blue(newValue)
            
        if self.sender() == self.sld4:
            image.change_low_hue(newValue)
        
        if self.sender() == self.sld5:
            image.change_low_sat(newValue)
            
        if self.sender() == self.sld6:
            image.change_low_val(newValue)
        
        if self.sender() == self.sld7:
            image.change_high_red(newValue)
            
        if self.sender() == self.sld8:
            image.change_high_green(newValue)
            
        if self.sender() == self.sld9:
            image.change_high_blue(newValue)
            
        if self.sender() == self.sld10:
            image.change_high_hue(newValue)
        
        if self.sender() == self.sld11:
            image.change_high_sat(newValue)
            
        if self.sender() == self.sld12:
            image.change_high_val(newValue)

    def takeoff(self):
        drone.keyCmd('t')
        self.state = "Takeoff"
        
    def landing(self):
        '''If the 1st user-defined button is clicked, then send a message
            to the robot saying so.'''
        drone.keyCmd(' ')
        self.state = "Landing"
        
    def btn2Function(self):
        '''If the 2nd user-defined button is clicked, then send a message
            to the robot saying so.'''
        self.state = "Button 2"
        
    def btn3Function(self):
        '''If the 3rd user-defined button is clicked, then send a message
            to the robot saying so.'''
        self.state = "Button 3"

    def displayImage(self):
        self.thresholdSet()
        image.keyboardLoop()

    def keyPressEvent(self, e):
        '''If a keyboard key is pressed, then respond accordingly'''
        try: char = chr(e.key()).lower()
        except ValueError: pass
        if e.key() == QtCore.Qt.Key_Escape: #if the escape key is pressed, then exit
            self.close()
            return
        if e.key() == QtCore.Qt.Key_Return: #If the space key is pressed, then send the spacebar keypress
            drone.keyCmd(' ')
            return
        if char == '$':
            image.save_thresholds()
            return
        if char == '&':
            image.load_thresholds()
            #self.thresholdset()
            return
        drone.keyCmd(char)
        self.state = "Keyboard"
        
    def thresholdSet(self):
        self.sld1.setValue(image.thresholds['low_red'])
        self.sld2.setValue(image.thresholds['low_green'])
        self.sld3.setValue(image.thresholds['low_blue'])
        self.sld4.setValue(image.thresholds['low_hue'])
        self.sld5.setValue(image.thresholds['low_sat'])
        self.sld6.setValue(image.thresholds['low_val'])
        self.sld7.setValue(image.thresholds['high_red'])
        self.sld8.setValue(image.thresholds['high_green'])
        self.sld9.setValue(image.thresholds['high_blue'])
        self.sld10.setValue(image.thresholds['high_hue'])
        self.sld11.setValue(image.thresholds['high_sat'])
        self.sld12.setValue(image.thresholds['high_val'])

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
    drone = Ardrone()
    image = ImageProcessor()
    app = QtGui.QApplication(sys.argv)
    widget = robotWidget()
    widget.thresholdSet()
    rospy.Subscriber(IMAGE_SOURCE, Image, image.videoUpdate, queue_size = 1)
    
    sys.exit(app.exec_())
    
