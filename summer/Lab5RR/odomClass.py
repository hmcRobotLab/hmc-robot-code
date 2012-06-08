import math

class Odometry:

    def __init__(self):
        '''initialize the values of the Odometer'''
        self.current_x = 0
        self.current_y = 0
        self.current_theta = 0
        self.prev_distance = 0
        self.angle = 0
        self.distance = 0
        self.reset = True
        self.start_distance = 0
        self.start_angle = 0
        
        
    def updateOdometry(self, data):
        """
           updateOdometry will be called every time the robot receives a sensor packet
        """

        if self.reset == True: #If we're told to reset the odometer, then set all the values to 0.
            self.start_distance = data.distance
            self.start_angle = data.angle
            self.prev_distance = 0
            self.current_x = 0
            self.current_y = 0
            self.current_theta = 0
            print "Starting at (0, 0)."
            self.reset = False
            
        else:
            self.distance = data.distance - self.start_distance #Distance between the current position and the starting position
            self.angle = data.angle - self.start_angle #Angle between the current angle and the starting angle
            self.current_x += (self.distance - self.prev_distance) * math.cos(math.radians(self.angle)) #Update the x coordinate using a little trigonometry
            self.current_y += (self.distance - self.prev_distance) * math.sin(math.radians(self.angle)) #Update the y coordinate using a little trigonometry
            self.current_theta = self.angle #Update the current angle
            self.prev_distance = self.distance
        return self
            
                   
    def __repr__(self):
        return "The current X,Y coordinates: (" + str(self.current_x) + ", " + str(self.current_y) + ").\n The current theta (heading): " + str(self.current_theta) + " degrees.\n"

