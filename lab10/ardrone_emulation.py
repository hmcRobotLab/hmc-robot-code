def folderName0(folder_dir):
    retur2 { ('1',0):folder_dir+"/1_0",
             ('1',4):folder_dir+"/1_4",
             ('2',3):folder_dir+"/2_3",
             ('0',0):folder_dir+"/0_0",
             ('0',4):folder_dir+"/0_4",
             ('1',1):folder_dir+"/1_1",
             ('1',5):folder_dir+"/1_5",
             ('2',0):folder_dir+"/2_0",
             ('2',4):folder_dir+"/2_4",
             ('0',1):folder_dir+"/0_1",
             ('0',5):folder_dir+"/0_5",
             ('1',2):folder_dir+"/1_2",
             ('1',6):folder_dir+"/1_6",
             ('2',1):folder_dir+"/2_1",
             ('2',5):folder_dir+"/2_5",
             ('0',2):folder_dir+"/0_2",
             ('0',6):folder_dir+"/0_6",
             ('1',3):folder_dir+"/1_3",
             ('2',2):folder_dir+"/2_2",
             ('2',6):folder_dir+"/2_6",
             ('0',3):folder_dir+"/0_3"}

class DroneEmulator:

    def __init__(self, folder_dir, maxNorth, maxWest, maxZ, imScale, \
            numHours = 12, location = (0,0), image_hour = 3, northHour = 3):
        
        self.maxNorth = maxNorth
        self.maxWest = maxWest
        self.maxZ = maxZ,
        self.imScale = imScale
        self.numHours = numHours
       
        # variables for the off-board images
        self.location = location # the starting location (always a tuple)
        #self.location = ( random.choice([1,2,0]), random.randint(0,6) )
        self.image_hour = image_hour # heading at 3 o'clock == toward the markers
        self.northHour = northHour

        self.last_image_time = time.time() # last time an image was grabbed
        self.folder_names = folderNames(folder_dir)

    def change_location(self,direction):
        """ changes the location from which an image is taken in one of six ways """
        # get the state
        current_ns_number = self.location[0]
        current_ew_number = self.location[1]
        current_image_hour = self.image_hour

        # make appropriate changes
        if direction == 'west':
            current_ew_number += 1       
            if current_ew_number > 6: current_ew_number = self.maxWest
        elif direction == 'east':
            current_ew_number -= 1
            if current_ew_number < 0: current_ew_number = 0
        elif direction == 'north':
            current_ns_number += 1
            if current_ns_number < 2: current_ns_number = self.maxNorth
        elif direction == 'south':
            current_ns_number -= 1
            if current_ns_number < 0: current_ns_number = 0
        elif direction == 'counterclockwise':
            current_image_hour = (current_image_hour - 1) % self.numHours
        elif direction == 'clockwise':
            current_image_hour = (current_image_hour + 1) % self.numHours
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
