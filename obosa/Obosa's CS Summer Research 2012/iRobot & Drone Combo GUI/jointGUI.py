#!/usr/bin/env python
from GUIardrone import *
from GUI_imageProcessor import *
from FSMs import *
from iRobotFinal import *
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
        self.roombaTheta.display(robot.odometry.current_theta)
        self.roombaX.display(robot.odometry.current_x)
        self.roombaY.display(robot.odometry.current_y)
        self.speedDisplay.display(robot.speed)

        
    def __init__(self):
        self.state = "Keyboard"
        super(robotWidget, self).__init__()
        
        hbox = QtGui.QHBoxLayout(self) # This is the main box that will hold our GUI.
        
        #################################################
        ##### Initializing Drone & Roomba Data LCDs, ####
        ##### state text, and timer (which           ####
        ##### we exploit in timerEvent               ####
        ##### as a secondary main loop)              ####
        #################################################
        self.altitude  = QtGui.QLCDNumber(self)
        self.phi       = QtGui.QLCDNumber(self)
        self.psi       = QtGui.QLCDNumber(self)
        self.theta     = QtGui.QLCDNumber(self)
        self.vx        = QtGui.QLCDNumber(self)
        self.vy        = QtGui.QLCDNumber(self)
        self.vz        = QtGui.QLCDNumber(self)
        self.batLevel  = QtGui.QLCDNumber(self)
        self.ctrlState = QtGui.QLCDNumber(self)
        self.stateLabel = QtGui.QLabel(self.state, self)
        self.roombaX = QtGui.QLCDNumber(self)
        self.roombaY = QtGui.QLCDNumber(self)
        self.roombaTheta = QtGui.QLCDNumber(self)
        self.speedDisplay = QtGui.QLCDNumber(self)
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
           
        
        #Low Red
        lcd1 = QtGui.QLCDNumber(self)
        self.sld1 = QtGui.QSlider(QtCore.Qt.Horizontal, self)
        lbl1 = QtGui.QLabel('Red', self)
        lbl1.setText("<font style='color: red;background: white;'>Red</font>")
        self.sld1.valueChanged.connect(lcd1.display)
        self.sld1.setMaximum(255)
        
        sliderGrid.addWidget(lcd1, 1, 2)
        sliderGrid.addWidget(self.sld1, 1, 1)
        sliderGrid.addWidget(lbl1, 1, 0)


        #Low Green
        lcd2 = QtGui.QLCDNumber(self)
        self.sld2 = QtGui.QSlider(QtCore.Qt.Horizontal, self)
        lbl2 = QtGui.QLabel('Green', self)
        lbl2.setText("<font style='color: green;background: white;'>Green</font>")
        self.sld2.valueChanged.connect(lcd2.display)
        self.sld2.setMaximum(255)
        
        sliderGrid.addWidget(lcd2, 2, 2)
        sliderGrid.addWidget(self.sld2, 2, 1)
        sliderGrid.addWidget(lbl2, 2, 0)


        #Low Blue
        lcd3 = QtGui.QLCDNumber(self)
        self.sld3 = QtGui.QSlider(QtCore.Qt.Horizontal, self)
        lbl3 = QtGui.QLabel("Blue", self)
        lbl3.setText("<font style='color: blue;background: white;'>Blue</font>")
        self.sld3.valueChanged.connect(lcd3.display)
        self.sld3.setMaximum(255)
        
        sliderGrid.addWidget(lcd3, 3, 2)
        sliderGrid.addWidget(self.sld3, 3, 1)
        sliderGrid.addWidget(lbl3, 3, 0)


        #Low Hue
        lcd4 = QtGui.QLCDNumber(self)
        self.sld4 = QtGui.QSlider(QtCore.Qt.Horizontal, self)
        lbl4 = QtGui.QLabel("Hue", self)
        lbl4.setText("<font style='color: brown;background: white;'>Hue</font>")
        self.sld4.valueChanged.connect(lcd4.display)
        self.sld4.setMaximum(255)
        
        sliderGrid.addWidget(lcd4, 4, 2)
        sliderGrid.addWidget(self.sld4, 4, 1)
        sliderGrid.addWidget(lbl4, 4, 0)
        
        
        #Low Saturation       
        lcd5 = QtGui.QLCDNumber(self)
        self.sld5 = QtGui.QSlider(QtCore.Qt.Horizontal, self)
        lbl5 = QtGui.QLabel("Saturation", self)
        lbl5.setText("<font style='color: grey;background: white;'>Saturation</font>")
        self.sld5.valueChanged.connect(lcd5.display)
        self.sld5.setMaximum(255)
        
        sliderGrid.addWidget(lcd5, 5, 2)
        sliderGrid.addWidget(self.sld5, 5, 1)
        sliderGrid.addWidget(lbl5, 5, 0)
        

        #Low Value   
        lcd6 = QtGui.QLCDNumber(self)
        self.sld6 = QtGui.QSlider(QtCore.Qt.Horizontal, self)
        lbl6 = QtGui.QLabel("Value", self)
        lbl6.setText("<font style='color: black;background: white;'>Value</font>")
        self.sld6.valueChanged.connect(lcd6.display)
        self.sld6.setMaximum(255)
        
        sliderGrid.addWidget(lcd6, 6, 2)
        sliderGrid.addWidget(self.sld6, 6, 1)
        sliderGrid.addWidget(lbl6, 6, 0)
        
        
        #High Red
        lcd7 = QtGui.QLCDNumber(self)
        self.sld7 = QtGui.QSlider(QtCore.Qt.Horizontal, self)
        lbl7 = QtGui.QLabel('High Red', self)
        lbl7.setText("<font style='color: red;background: white;'>High Red</font>")
        self.sld7.valueChanged.connect(lcd7.display)   
        self.sld7.setMaximum(255)
        
        sliderGrid.addWidget(lcd7, 7, 2)
        sliderGrid.addWidget(self.sld7, 7, 1)
        sliderGrid.addWidget(lbl7, 7, 0)


        #High Green
        lcd8 = QtGui.QLCDNumber(self)
        self.sld8 = QtGui.QSlider(QtCore.Qt.Horizontal, self)
        lbl8 = QtGui.QLabel('High Green', self)
        lbl8.setText("<font style='color: green;background: white;'>High Green</font>")
        self.sld8.valueChanged.connect(lcd8.display)
        self.sld8.setMaximum(255)
        
        sliderGrid.addWidget(lcd8, 8, 2)
        sliderGrid.addWidget(self.sld8, 8, 1)
        sliderGrid.addWidget(lbl8, 8, 0)


        #High Blue
        lcd9 = QtGui.QLCDNumber(self)
        self.sld9 = QtGui.QSlider(QtCore.Qt.Horizontal, self)
        lbl9 = QtGui.QLabel("High Blue", self)
        lbl9.setText("<font style='color: blue;background: white;'>High Blue</font>")
        self.sld9.valueChanged.connect(lcd9.display)
        self.sld9.setMaximum(255)
        
        sliderGrid.addWidget(lcd9, 9, 2)
        sliderGrid.addWidget(self.sld9, 9, 1)
        sliderGrid.addWidget(lbl9, 9, 0)


        #High Hue
        lcd10 = QtGui.QLCDNumber(self)
        self.sld10 = QtGui.QSlider(QtCore.Qt.Horizontal, self)
        lbl10 = QtGui.QLabel("High Hue", self)
        lbl10.setText("<font style='color: brown;background: white;'>High Hue</font>")
        self.sld10.valueChanged.connect(lcd10.display)
        self.sld10.setMaximum(255)
        
        sliderGrid.addWidget(lcd10, 10, 2)
        sliderGrid.addWidget(self.sld10, 10, 1)
        sliderGrid.addWidget(lbl10, 10, 0)
        
        
        #High Saturation       
        lcd11 = QtGui.QLCDNumber(self)
        self.sld11 = QtGui.QSlider(QtCore.Qt.Horizontal, self)
        lbl11 = QtGui.QLabel("High Saturation", self)
        lbl11.setText("<font style='color: grey;background: white;'>High Saturation</font>")
        self.sld11.valueChanged.connect(lcd11.display)
        self.sld11.setMaximum(255)
        
        sliderGrid.addWidget(lcd11, 11, 2)
        sliderGrid.addWidget(self.sld11, 11, 1)
        sliderGrid.addWidget(lbl11, 11, 0)
        

        #High Value     
        lcd12 = QtGui.QLCDNumber(self)
        self.sld12 = QtGui.QSlider(QtCore.Qt.Horizontal, self)
        lbl12 = QtGui.QLabel("High Value", self)
        lbl12.setText("<font style='color: black;background: white;'>High Value</font>")
        self.sld12.valueChanged.connect(lcd12.display)
        self.sld12.setMaximum(255)
        
        sliderGrid.addWidget(lcd12, 12, 2)
        sliderGrid.addWidget(self.sld12, 12, 1)
        sliderGrid.addWidget(lbl12, 12, 0)
        
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
        
        
        #######################################################
        #### Setting up the bottom-left box                ####
        #### (The keyboard controls, and all the buttons)  ####
        #######################################################
        
        
        bottomGrid = QtGui.QGridLayout()
        bottomGrid.setVerticalSpacing(5)
        bottomGrid.setHorizontalSpacing(10)
        
       
        

        #Displaying the keyboard controls
        keyboardControls = QtGui.QLabel(self)
        keyboardControls.setText("<center><font size = '1';font style='color: black;'> \
iRobot Keyboard Controls:<br<br> \
i: forward \
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; \
k: backward \
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; \
j: rotate left \
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; \
l: rotate right <br>\
u: increase speed \
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; \
o: decrease speed \
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; \
m: brake <br><br><br> \
Drone Keyboard Controls:<br><br> \
h: quit<br> enter/return: land <br> t: takeoff<br> r: reset<br> \
$: save color thresholds <br> &: load color thresholds<br>\
1: Use forward camera <br> 2: Use bottom camera <br>\
q: forward left strafe \
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;\
w: forward \
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;\
e: forward right strafe<br> \
a: strafe left \
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;\
s: self-adjusting hover \
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;\
d: strafe right<br> \
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;\
3: regular, non-adjusting hover <br>\
z: backward left strafe \
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;\
x: backward \
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;\
e: backward right strafe<br> \
f: spin left<br> g: spin right<br> \
r: v<br> b: down<br><br> \
Press Escape to Exit </font></center><br>")

        
        #Creating the takeoff, landing, FSM, and camera-changing buttons
        takeoffBtn = QtGui.QPushButton('Takeoff!', self)
        landingBtn = QtGui.QPushButton('Land!', self)
        newBtn2 = QtGui.QPushButton('Find and Land in Front', self)
        newBtn3 = QtGui.QPushButton('Find and Follow the Leader', self) 
        newBtn4 = QtGui.QPushButton('Find and Land on Top', self)   
        newBtn5 = QtGui.QPushButton('Hover Above iRobot, then Land on Target', self)       
        self.robotFSM1 = QtGui.QPushButton('iRobot FSM: Line-follow a loop once', self)
        self.robotFSM2 = QtGui.QPushButton('iRobot FSM: Measure the area of a rectangular room', self)
        
        #Connecting the buttons to their various functions
        takeoffBtn.clicked.connect(self.takeoff)
        landingBtn.clicked.connect(self.landing)
        newBtn2.clicked.connect(self.btn2Function)
        newBtn3.clicked.connect(self.btn3Function)
        newBtn4.clicked.connect(self.btn4Function)
        newBtn5.clicked.connect(self.btn5Function)
        self.robotFSM1.clicked.connect(self.iRobotFSM)
        self.robotFSM2.clicked.connect(self.iRobotFSM)
        

        ###### Adding the Widgets to the GUI######
        bottomGrid.addWidget(keyboardControls, 1, 0, 4, 1)
        bottomGrid.addWidget(takeoffBtn, 1, 3)
        bottomGrid.addWidget(landingBtn, 2, 3)
        bottomGrid.addWidget(newBtn2, 3, 3)
        bottomGrid.addWidget(newBtn3, 4, 3)
        bottomGrid.addWidget(self.robotFSM1, 5, 0)
        bottomGrid.addWidget(self.robotFSM2, 6, 0)
        bottomGrid.addWidget(newBtn4, 5, 3)
        bottomGrid.addWidget(newBtn5, 6, 3)
        
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

        roombaTheta = QtGui.QLabel("iRobot Theta", self)
        droneData.addWidget(roombaTheta, 10, 0)
        droneData.addWidget(self.roombaTheta, 10, 1)

        roombaX = QtGui.QLabel("iRobot X Position", self)
        droneData.addWidget(roombaX, 11, 0)
        droneData.addWidget(self.roombaX, 11, 1)

        roombaY = QtGui.QLabel("iRobot Y Position", self)
        droneData.addWidget(roombaY, 12, 0)
        droneData.addWidget(self.roombaY, 12, 1)
        
        speedDisplay = QtGui.QLabel("iRobot Speed", self)
        droneData.addWidget(speedDisplay, 13, 0)
        droneData.addWidget(self.speedDisplay, 13, 1)

        state = QtGui.QLabel("Drone Current State:", self) 
        droneData.addWidget(state, 14, 0)
        droneData.addWidget(self.stateLabel, 14, 1)

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
        self.setWindowTitle('Drone and Roomba Control Panel')
        self.show()

    
    #################################
    ####  Functions that define  ####
    ####  widget behavior        ####
    #################################

    def iRobotFSM(self):
        '''Function to change which camera is being used'''
        if self.sender() == self.robotFSM1: 
            robot.state = "Button 2"
        if self.sender() == self.robotFSM2: 
            robot.state = "Button 3"

    def sliderChange(self, newValue):
        '''If a slider is moved, then the imageProcessor's threshold values will be changed accordingly'''
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
        '''The takeoff button'''
        drone.keyCmd('t')
        self.state = "Takeoff"
        time.sleep(3)
        self.state = "Airborne"
        publisher.publish(self.state)
        
    def landing(self):
        '''The landing button'''
        drone.keyCmd(' ')
        self.state = "Landing"
        publisher.publish(self.state)
        time.sleep(3)
        
    def btn2Function(self):
        '''Run the "Land in front" FSM'''
        self.state = "Button 2"
        print "RUNNING FSM0"
        FSM.FSMstart()
        FSM.FSM0()

    def btn3Function(self):
        '''Run the "Follow the leader" FSM'''
        self.state = "Button 3"
        print "RUNNING FSM1"
        FSM.FSMstart()
        FSM.FSM1()
        
    def btn4Function(self):
        '''Run the "Land on top" FSM'''
        self.state = "Button 4"
        print "RUNNING FSM2"
        FSM.FSMstart()
        FSM.FSM2()

    def btn5Function(self):
        '''Run the "Hover-follow, then land on top" FSM'''
        self.state = "Button 5"
        print "RUNNING FSM3"
        FSM.FSMstart()
        FSM.FSM3()

    def displayImage(self):
        '''Slider, state, and keyboard loop updating'''
        self.thresholdSet()
        robot.execute()
        image.keyboardLoop()
        self.stateLabel.setText("<font size='5'>" + self.state + "</font>")

    def keyPressEvent(self, e):
        '''If a keyboard key is pressed, then respond accordingly'''
        try: char = chr(e.key()).lower()
        except ValueError: char = e.key()
        if char == QtCore.Qt.Key_Escape: #if the escape key is pressed, then exit
            robot.state = "Quit!"
            robot.execute()
            self.close()
            return

        if char == QtCore.Qt.Key_Return: #If the enter key is pressed, then land the drone
            drone.keyCmd(' ')
            return

        if char == '$': #If '$' is pressed, save the thresholds
            image.save_thresholds()
            return

        if char == '&': #If '&' is pressed, load the thresholds
            image.load_thresholds()
            return

        if char == '1': #If '1' is pressed, use the drone's forward camera
            drone.keyCmd('1')
            return

        if char == '2': #if '2' is pressed, use the drone's bottom camera
            drone.keyCmd('2')
            return

        if char == 'i': #If 'i' is pressed, then tell the iRobot to move forward
            robot.state = "Keyboard"
            print "Moving forward"
            robot.action = 'w'
            return

        if char == 'k': #If 'k' is pressed, then tell the iRobot to move backward
            robot.state = "Keyboard"
            print "Moving backward"
            robot.action = 's'
            return

        if char == 'l': #If 'l' is pressed, then tell the iRobot to rotate clockwise
            robot.state = "Keyboard"
            print "Rotating right"
            robot.action = 'd'
            return

        if char == 'j': #If 'j' is pressed, then tell the iRobot to rotate counter-clockwise
            robot.state = "Keyboard"
            print "Rotating left"
            robot.action = 'a'
            return

        if char == 'm': #If 'm' is pressed, then tell the iRobot to brake
            robot.state = "Keyboard"
            print "Braking"
            robot.action = "b"
            return

        if char == 'o': #If 'o' is pressed, then tell the iRobot to slow down
            robot.speed -= 10
            print "Slowing down. Speed is now", robot.speed
            return

        if char == 'u': #If 'u' is pressed, then tell the iRobot to speed upo
            robot.speed += 10
            print "Speeding up. Speed is now", robot.speed
            return

        publisher.publish("Keyboard")
        drone.keyCmd(char)
        self.state = "Keyboard"
        
    def thresholdSet(self):
        '''Set all of the slider values to match those used by the imageProcessor'''
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

    def handle_state_data(self, data):
        '''Function for handling data from the finite state machine'''
        packet = data.data.split()
        if packet[0] == "ChangeCamera":
            drone.keyCmd(packet[1])
        else:
            #The first string should be the state. 
            #If there's a second string, it will be a drone key command.
            self.state = packet[0]
            if self.state == "Keyboard" or self.state == "Hover":
                drone.keyCmd('s')
            if self.state == "Landing":
                drone.keyCmd(' ')
            if self.state == "Takeoff":
                drone.keyCmd('t')
            else:
                if len(packet) > 1:
                    drone.keyCmd(packet[1])
        
    
if __name__ == '__main__':
    publisher = rospy.Publisher('GUI', String)
    drone = Ardrone()
    robot = iRobot()
    image = ImageProcessor()
    FSM = FSM()
    app = QtGui.QApplication(sys.argv)
    widget = robotWidget()
    widget.thresholdSet()
    rospy.Subscriber('state', String, widget.handle_state_data)
    rospy.Subscriber(IMAGE_SOURCE, Image, image.videoUpdate, queue_size = 1)
    sys.exit(app.exec_())
    
