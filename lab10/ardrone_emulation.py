def folderNames(folder_dir):
    return { ('c',0):folder_dir+"/c_0",
             ('c',4):folder_dir+"/c_4",
             ('n',3):folder_dir+"/n_3",
             ('s',0):folder_dir+"/s_0",
             ('s',4):folder_dir+"/s_4",
             ('c',1):folder_dir+"/c_1",
             ('c',5):folder_dir+"/c_5",
             ('n',0):folder_dir+"/n_0",
             ('n',4):folder_dir+"/n_4",
             ('s',1):folder_dir+"/s_1",
             ('s',5):folder_dir+"/s_5",
             ('c',2):folder_dir+"/c_2",
             ('c',6):folder_dir+"/c_6",
             ('n',1):folder_dir+"/n_1",
             ('n',5):folder_dir+"/n_5",
             ('s',2):folder_dir+"/s_2",
             ('s',6):folder_dir+"/s_6",
             ('c',3):folder_dir+"/c_3",
             ('n',2):folder_dir+"/n_2",
             ('n',6):folder_dir+"/n_6",
             ('s',3):folder_dir+"/s_3"}

class DroneEmulator:

    def __init__(self, folder_dir, northSouthRange, eastWestRange, zRange, imScale):

        # variables for the off-board images
        self.location = ('c',1) # the starting location (always a tuple)
        #self.location = ( random.choice(['c','n','s']), random.randint(0,6) )
        self.image_hour = 3 # heading at 3 o'clock == toward the markers
        #self.angle_deg = random.randint(180,360) # the angle it's facing
        self.last_image_time = time.time() # last time an image was grabbed
        self.folder_names = folderNames(folder_dir)

    def change_location(self,direction):
        """ changes the location from which an image is taken in one of six ways """
        # get the state
        current_ns_character = self.location[0]
        current_ew_number = self.location[1]
        current_image_hour = self.image_hour

        # make appropriate changes
        if direction == 'west':
            current_ew_number += 1       
            if current_ew_number > 6: current_ew_number = 6
        elif direction == 'east':
            current_ew_number -= 1
            if current_ew_number < 0: current_ew_number = 0
        elif direction == 'north':
            if current_ns_character == 'n': pass
            elif current_ns_character == 's': current_ns_character = 'c'
            elif current_ns_character == 'c': current_ns_character = 'n'
        elif direction == 'south':
            if current_ns_character == 'n': current_ns_character = 'c'
            elif current_ns_character == 's': pass
            elif current_ns_character == 'c': current_ns_character = 's'
        elif direction == 'counterclockwise':
            current_image_hour -= 1
            if current_image_hour < 0: current_image_hour = 23
        elif direction == 'clockwise':
            current_image_hour += 1
            if current_image_hour > 23: current_image_hour = 0
        else:
            print "a direction of", direction,
            print "was not recognized in change_location"

        # reset the state
        self.location = (current_ns_character,current_ew_number)
        self.image_hour = current_image_hour
        print "location and image_hour are now", self.location, self.image_hour


    def keyControls(self,c):
       # if not self.use_drone (simulated mode)
       # the arrow keys allow you to change the images you see...
       if not self.use_drone:
           if c == 82:  # up arrow is the same as ord('R')
               self.change_location( 'west' )
           if c == 84:  # down arrow is the same as ord('T')
               self.change_location( 'east' )
           if c == 81:  # left arrow is the same as ord('Q')
               self.change_location( 'north' )
           if c == 83:  # right arrow is the same as ord('S')
               self.change_location( 'south' )
           if c == ord('-'):  # '-' decreases the image_hour
               self.change_location( 'counterclockwise' )
           if c == ord('=') or c == ord('+'):  # '=' or '+' increases the image_hour
               self.change_location( 'clockwise' )

    def updateImage(self, data):
        cur_time = time.time()
        if cur_time - self.last_image_time > 0.25: # number of seconds per update
            self.last_image_time = cur_time # reset the last image time
            folder = self.folder_names[self.location]
            fn = folder + "/" + str(self.image_hour) + ".png"
            #print "filename", fn
            self.color_image=cv.LoadImageM(fn)
            self.new_image = True
            #otherwise, we don't get a new image
