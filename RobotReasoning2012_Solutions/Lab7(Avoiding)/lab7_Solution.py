#!/usr/bin/env python
import roslib; roslib.load_manifest('irobot_mudd')
import rospy
import cv_bridge
import cv
import sensor_msgs.msg as sm
from irobot_mudd.srv import *
from irobot_mudd.msg import *
import irobot_mudd
import math

# Dictionary to hold all globals in
D = {}
song = rospy.ServiceProxy('song', Song)
try: tank = rospy.ServiceProxy('tank', Tank) # tank permits requests, e.g., tank(0,50,50)
except ROSSerializationException: pass

#################### INITIALIZATION FUNCTIONS ######################

def init_globals():
    """ sets up all the globals in the dictionary D
    """
    # get D so that we can change values in it
    global D

    # put threshold values into D
    D['thresholds'] = {'low_red':0, 'high_red':255,\
                       'low_green':0, 'high_green':255,\
                       'low_blue':0, 'high_blue':255,\
                       'low_hue':0, 'high_hue':255,\
                       'low_sat':0, 'high_sat':255,\
                       'low_val':0, 'high_val':255 }

#################### DICTIONARY I ADDED ####################
    D['state'] = "Keyboard"
    D['robpos'] = {'ini':(0,0), 'fin':(0,0), 'dist': 0}
    D['target'] = { 'targeting': False, 'xytar': (0,0), 'totarg': 0}
    D['obstacle'] = { 'avoiding': False, 'xyobs': (0,0), 'toobs': 0, 'adding':False}
    D['vectors'] = { 'adding': False, 'fakerob': (0,0), 'showtar': True, 'sumlength': 20, \
                     'obslength':20, 'tarlength':50, 'tarvect':(0,0), 'obsvect':(0,0), \
                     'netvect': (0,0),'showobs': False}

#################### END DICTIONARY I ADDED ####################
    
    # Set up the windows containing the image from the kinect,
    # the altered image, and the threshold sliders.
    cv.NamedWindow('image')
    cv.MoveWindow('image', 0, 0)
    cv.NamedWindow('threshold')
    cv.MoveWindow('threshold', 640, 0)
    cv.NamedWindow('sliders')
    cv.MoveWindow('sliders', 1280, 0)

    # Create the sliders within the 'sliders' window
    cv.CreateTrackbar('low_red', 'sliders', D['thresholds']['low_red'], 255, \
                          lambda x: change_slider('low_red', x) )
    cv.CreateTrackbar('high_red', 'sliders', D['thresholds']['high_red'], 255, \
                          lambda x: change_slider('high_red', x) )
    cv.CreateTrackbar('low_green', 'sliders', D['thresholds']['low_green'], 255, \
                          lambda x: change_slider('low_green', x) )
    cv.CreateTrackbar('high_green', 'sliders', D['thresholds']['high_green'], 255, \
                          lambda x: change_slider('high_green', x) )
    cv.CreateTrackbar('low_blue', 'sliders', D['thresholds']['low_blue'], 255, \
                          lambda x: change_slider('low_blue', x) )
    cv.CreateTrackbar('high_blue', 'sliders', D['thresholds']['high_blue'], 255, \
                          lambda x: change_slider('high_blue', x) )
    cv.CreateTrackbar('low_hue', 'sliders', D['thresholds']['low_hue'], 255, \
                          lambda x: change_slider('low_hue', x) )
    cv.CreateTrackbar('high_hue', 'sliders', D['thresholds']['high_hue'], 255, \
                          lambda x: change_slider('high_hue', x) )
    cv.CreateTrackbar('low_sat', 'sliders', D['thresholds']['low_sat'], 255, \
                          lambda x: change_slider('low_sat', x) )
    cv.CreateTrackbar('high_sat', 'sliders', D['thresholds']['high_sat'], 255, \
                          lambda x: change_slider('high_sat', x) )
    cv.CreateTrackbar('low_val', 'sliders', D['thresholds']['low_val'], 255, \
                          lambda x: change_slider('low_val', x) )
    cv.CreateTrackbar('high_val', 'sliders', D['thresholds']['high_val'], 255, \
                          lambda x: change_slider('high_val', x) )

    # Set the method to handle mouse button presses
    cv.SetMouseCallback('image', onMouse, None)

    # We have not created our "scratchwork" images yet
    D["created_images"] = False

    # Variable for key presses
    D["last_key_pressed"] = 255

    # The current image we want to display in the threshold window
    D["current_threshold"] = "threshed_image"

    # Not currently clicking down
    D["mouse_down"] = False

    # Highlighted area does not exist yet
    D["down_coord"] = (-1,-1)
    D["up_coord"] = (-1,-1)
    D["sections"] = []
    D["mode"] = "clear"

    # Create a connection to the Kinect
    D["bridge"] = cv_bridge.CvBridge()


def init_images():
    """ Creates all the images we'll need. Is separate from init_globals 
        since we need to know what size the images are before we can make
        them
    """
    # get D so that we can change values in it
    global D

    # Find the size of the image 
    # (we set D["image"] right before calling this function)
    D["size"] = cv.GetSize(D["image"])

    # Create images for each color channel
    D["red"] = cv.CreateImage(D["size"], 8, 1)
    D["blue"] = cv.CreateImage(D["size"], 8, 1)
    D["green"] = cv.CreateImage(D["size"], 8, 1)
    D["hue"] = cv.CreateImage(D["size"], 8, 1)
    D["sat"] = cv.CreateImage(D["size"], 8, 1)
    D["val"] = cv.CreateImage(D["size"], 8, 1)

    # Create images to save the thresholded images to
    D["red_threshed"] = cv.CreateImage(D["size"], 8, 1)
    D["green_threshed"] = cv.CreateImage(D["size"], 8, 1)
    D["blue_threshed"] = cv.CreateImage(D["size"], 8, 1)
    D["hue_threshed"] = cv.CreateImage(D["size"], 8, 1)
    D["sat_threshed"] = cv.CreateImage(D["size"], 8, 1)
    D["val_threshed"] = cv.CreateImage(D["size"], 8, 1)

    # The final thresholded result
    D["threshed_image"] = cv.CreateImage(D["size"], 8, 1)

    # Create the hsv image
    D["hsv"] = cv.CreateImage(D["size"], 8, 3)

################## END INITIALIZATION FUNCTIONS ####################

################### IMAGE PROCESSING FUNCTIONS #####################

def threshold_image():
    """ runs the image processing in order to create a 
        black and white thresholded image out of D["image"]
        into D["threshed_image"]
    """
    # get D so that we can change values in it
    global D

    # Use OpenCV to split the image up into channels,
    # saving them in their respective bw images
    cv.Split(D["image"], D["blue"], D["green"], D["red"], None)

    # This line creates a hue-saturation-value image
    cv.CvtColor(D["image"], D["hsv"], cv.CV_RGB2HSV)
    cv.Split(D["hsv"], D["hue"], D["sat"], D["val"], None)

    # Here is how OpenCV thresholds the images based on the slider values:
    cv.InRangeS(D["red"], D['thresholds']["low_red"], \
                    D['thresholds']["high_red"], D["red_threshed"])
    cv.InRangeS(D["blue"], D['thresholds']["low_blue"], \
                    D['thresholds']["high_blue"], D["blue_threshed"])
    cv.InRangeS(D["green"], D['thresholds']["low_green"], \
                    D['thresholds']["high_green"], D["green_threshed"])
    cv.InRangeS(D["hue"], D['thresholds']["low_hue"], \
                    D['thresholds']["high_hue"], D["hue_threshed"])
    cv.InRangeS(D["sat"], D['thresholds']["low_sat"], \
                    D['thresholds']["high_sat"], D["sat_threshed"])
    cv.InRangeS(D["val"], D['thresholds']["low_val"], \
                    D['thresholds']["high_val"], D["val_threshed"])

    # Multiply all the thresholded images into one "output" image,
    # named D["threshed_image"]
    cv.Mul(D["red_threshed"], D["green_threshed"], D["threshed_image"])
    cv.Mul(D["threshed_image"], D["blue_threshed"], D["threshed_image"])
    cv.Mul(D["threshed_image"], D["hue_threshed"], D["threshed_image"])
    cv.Mul(D["threshed_image"], D["sat_threshed"], D["threshed_image"])
    cv.Mul(D["threshed_image"], D["val_threshed"], D["threshed_image"])

    # Erode and Dilate shave off and add edge pixels respectively
    cv.Erode(D["threshed_image"], D["threshed_image"], iterations = 1)
    cv.Dilate(D["threshed_image"], D["threshed_image"], iterations = 1)


def find_biggest_region():
    """ finds all the contours in threshed image, finds the largest of those,
        and then marks in in the main image
    """
    # get D so that we can change values in it
    global D

    # Create a copy image of thresholds then find contours on that image
    storage = cv.CreateMemStorage(0) # Create memory storage for contours
    copy = cv.CreateImage(D["size"], 8, 1)
    cv.Copy( D["threshed_image"], copy ) # copy threshed image

    # this is OpenCV's call to find all of the contours:
    contours = cv.FindContours(copy, storage, cv.CV_RETR_EXTERNAL, \
                                   cv.CV_CHAIN_APPROX_SIMPLE)

    # Next we want to find the *largest* contour
    if len(contours) > 0:
        biggest = contours
        biggestArea = cv.ContourArea(contours)
        while contours != None:
            nextArea = cv.ContourArea(contours)
            if biggestArea < nextArea:
                biggest = contours
                biggestArea = nextArea
            contours = contours.h_next()
        
        # Use OpenCV to get a bounding rectangle for the largest contour
        br = cv.BoundingRect(biggest, update=0)
        
        # Draw a box around the largest contour, and a circle at its center
        cv.PolyLine(D["image"], [[(br[0], br[1]), (br[0], br[1] + br[3]), \
                                      (br[0] + br[2], br[1] + br[3]), \
                                      (br[0] + br[2], br[1])]],\
                        1, cv.RGB(255, 0, 0))

        # Draw the circle:
        cv.Circle(D["image"], (br[0] + int(br[2]/2), br[1] + int(br[3]/2)), 10, \
                      cv.RGB(255, 255, 0), thickness=1, lineType=8, shift=0)

        # Draw the contours in white with inner ones in green
        #cv.DrawContours(D["image"], biggest, cv.RGB(255, 255, 255), \
                          #  cv.RGB(0, 255, 0), 1, thickness=2, lineType=8, \
                            #offset=(0,0))

def mouse_section():
    """ displays a rectangle defined by dragging the mouse """
    global D

    if D["mouse_down"]:
        x0 = D["down_coord"][0]
        y0 = D["down_coord"][1]
        x1 = D["up_coord"][0]
        y1 = D["up_coord"][1]
        cv.PolyLine(D["image"], [[(x0,y0), (x0,y1), (x1,y1), (x1,y0)]], 1, cv.RGB(255, 255, 0))


def process_section():
    """ calculates the min/max slider values for a given section and sets the
        slider values to them
    """
    global D

    print "sections:", D["sections"]

    if len(D["sections"]) > 0:
        # pull out all sections with 'a' in the front
        adds = [[x[1], x[2]] for x in D["sections"] if x[0] == 'a']

        # pull out all sections with a 's' in the front
        subs = [[x[1], x[2]] for x in D["sections"] if x[0] == 's']

        print "adds: ", adds
        print "subs: ", subs
        # mins default to highest value, maxs default to lowest value
        minmax = [[255,0] for x in xrange(6)]

        for i in adds:
            
            # get coords for easy access
            x0 = i[0][0]
            y0 = i[0][1]
            x1 = i[1][0]
            y1 = i[1][1]

            # Make sure that x0, y0 are smaller than x1, y1
            if x0 > x1:
                x0, x1 = x1, x0
            if y0 > y1:
                y0, y1 = y1, y0

            # Go through pixel by pixel
            for x in xrange(x0, x1):
                for y in xrange(y0, y1):
                    in_range = True

                    # the point will be out of range if it is in a subtracted section somewhere
                    for p in subs:
                        if x > p[0][0] and y > p[0][1] and x < p[1][0] and y < p[1][1]:
                            in_range = False

                    if in_range:
                        # Know it isn't part of a subtracted region
                        (b,g,r) = D["image"][y,x]
                        (h,s,v) = D["hsv"][y,x]
                        allv = [r,g,b, h, s, v]
                        for j in xrange(len(allv)):
                            if allv[j] < minmax[j][0]:
                                minmax[j][0] = allv[j]
                            if allv[j] > minmax[j][1]:
                                minmax[j][1] = allv[j]
    
        # Now set sliders to those values
        names = [["low_red", "high_red"], ["low_green", "high_green"],\
                     ["low_blue", "high_blue"], ["low_hue", "high_hue"],\
                     ["low_sat", "high_sat"], ["low_val", "high_val"]]

        for i in xrange(len(names)):
            for j in xrange(2):
                D["thresholds"][names[i][j]] = int(minmax[i][j])
                cv.SetTrackbarPos(names[i][j], 'sliders', int(minmax[i][j]))



#################### FUNCTIONS I ADDED ####################
def robotlocation():

# get D so that we can change values in it
    global D

    # Create a copy image of thresholds then find contours on that image
    storage = cv.CreateMemStorage(0) # Create memory storage for contours
    copy = cv.CreateImage(D["size"], 8, 1)
    cv.Copy( D["threshed_image"], copy ) # copy threshed image

    # this is OpenCV's call to find all of the contours:
    contours = cv.FindContours(copy, storage, cv.CV_RETR_EXTERNAL, \
                                   cv.CV_CHAIN_APPROX_SIMPLE)

    # Next we want to find the *largest* contour
    if len(contours) > 0:
        biggest = contours
        biggestArea = cv.ContourArea(contours)
        while contours != None:
            nextArea = cv.ContourArea(contours)
            if biggestArea < nextArea:
                biggest = contours
                biggestArea = nextArea
            contours = contours.h_next()
        
        # Use OpenCV to get a bounding rectangle for the largest contour
        br = cv.BoundingRect(biggest, update=0)

        return (br[0] + int(br[2]/2), br[1] + int(br[3]/2))


def targetcircle():
    if D['target']['targeting'] == True:
        cv.Circle(D["image"], (D['target']['xytar'][0], D['target']['xytar'][1]), 10, \
            cv.RGB(255, 0, 255), thickness=1, lineType=8, shift=0)
    else:
        return

def obstaclecircle():
    if D['obstacle']['avoiding'] == True:
        cv.Circle(D["image"], (D['obstacle']['xyobs'][0], D['obstacle']['xyobs'][1]), 10, \
            cv.RGB(255, 255, 255), thickness=3, lineType=8, shift=0)
    else:
        return

    
def distancegiver():
    D['robpos']['dist'] = (D['robpos']['fin'][0]-D['robpos']['ini'][0],D['robpos']['fin'][1]-D['robpos']['ini'][1])

def distancetotarget():
    D['robpos']['fin'] = robotlocation()
   # print "The robot final poition is", D['robpos']['fin']
    dx = D['target']['xytar'][0]-D['robpos']['fin'][0]
    dy = D['target']['xytar'][1]-D['robpos']['fin'][1]
    D['target']['totarg'] = math.hypot(	dx, dy)
    return 


def anglegiver(n,b):
    dx = n[0]-b[0]
    dy = n[1]-b[1]
    rads = math.atan2(dy,dx)
    return math.degrees(rads)

def arrived():
    if D['target']['targeting'] == True:
        distancetotarget()
        if D['target']['totarg'] < 20:
            D['state'] = "InRange"
            #print D['target']['totarg']
        else:
            pass
    else:
        pass


def distancetoobs():
    D['robpos']['fin'] = robotlocation()
    dx = D['obstacle']['xyobs'][0]-D['robpos']['fin'][0]
    dy = D['obstacle']['xyobs'][1]-D['robpos']['fin'][1]
    D['obstacle']['toobs'] = math.hypot( dx, dy)


def avoid():
    if D['obstacle']['avoiding'] == True:
        distancetoobs()
        if D['obstacle']['toobs'] < 20 and D['target']['totarg'] > 20:
            D['state'] = "Avoid"
        elif D['obstacle']['toobs'] < 20 and D['target']['totarg'] > 20:
            D['state']= "InRange"
            
        #else:
           # D['state'] = "Forward"
    else:
        pass

def newPoint((x1,y1), (x2,y2), finLength):
    dist = math.hypot((x2-x1),(y2-y1))
    fraction = finLength/dist
    newX = ((x2-x1)*fraction)+x1
    newY = ((y2-y1)*fraction)+y1
    return (int(newX),int(newY))

def newobPoint((x1,y1), (x2,y2), finLength):
    dist = math.hypot((x2-x1),(y2-y1))
    fraction = finLength/dist
    newX = -((x2-x1)*fraction)+x1
    newY = -((y2-y1)*fraction)+y1
    return (int(newX),int(newY))

def drawtarvector():
    if D['vectors']['showtar'] == True and D['target']['targeting'] == True:
        strentarlength()
        newCoord = newPoint(D['robpos']['fin'], D['target']['xytar'], D['vectors']['tarlength'])
        D['vectors']['tarvect'] = newCoord
        cv.Line(D["image"], D['robpos']['fin'], D['vectors']['tarvect'], \
                cv.RGB(255, 255, 0), thickness = 1, lineType = 0, shift = 0)
        
def strentarlength():
    if D['vectors']['showtar'] == True and D['target']['targeting'] == True:
        distfromrob = math.hypot(D['robpos']['fin'][0]-D['target']['xytar'][0],\
                                 D['robpos']['fin'][1]-D['target']['xytar'][1])
        if 150 < distfromrob:
            D['vectors']['tarlength'] = 10000/distfromrob
        else:
            D['vectors']['tarlength'] = 100

def strenobslength():
    if D['vectors']['showobs'] == True and D['obstacle']['avoiding'] == True:
        distfromrob = math.hypot(D['robpos']['fin'][0]-D['obstacle']['xyobs'][0],\
                                 D['robpos']['fin'][1]-D['obstacle']['xyobs'][1])
        D['vectors']['obslength'] = int(5000/distfromrob)        

def drawobsvector():
    if D['vectors']['showobs'] == True and D['obstacle']['avoiding'] == True:
        strenobslength()
        newCoord = newobPoint(D['robpos']['fin'], D['obstacle']['xyobs'], D['vectors']['obslength'])
        D['vectors']['obsvect'] = newCoord
        cv.Line(D["image"], D['robpos']['fin'], D['vectors']['obsvect'], \
                cv.RGB(255, 0, 255), thickness = 2, lineType = 0, shift = 0)

def drawnetvector():
    if D['vectors']['showobs'] == True and D['obstacle']['avoiding'] == True\
       and D['vectors']['showtar'] == True and D['target']['targeting'] == True:
        D['vectors']['netvect'] = addvectors(D['robpos']['fin'], \
                                             D['vectors']['obsvect'], D['vectors']['tarvect'])
        D['vectors']['netvect'][0] += int(2000/D['robpos']['fin'][0] - 2000/(640-D['robpos']['fin'][0]))
        D['vectors']['netvect'][1] += int(2000/D['robpos']['fin'][1] - 2000/(480-D['robpos']['fin'][1]))
        NetVect = (D['vectors']['netvect'][0], D['vectors']['netvect'][1])
        cv.Line(D["image"], D['robpos']['fin'], NetVect, \
                cv.RGB(0, 255, 255), thickness = 2, lineType = 0, shift = 0)


def addvectors(robpos,p1,p2):
    x1 = p1[0]-robpos[0]
    x2 = p2[0]-robpos[0]
    vx = x1+x2
    y1 = p1[1]-robpos[1]
    y2 = p2[1]-robpos[1]
    vy = y1+y2
    return[vx+robpos[0],vy+robpos[1]]
    
    

#################### FUNCTIONS I END ####################


################# END IMAGE PROCESSING FUNCTIONS ###################

####################### CALLBACK FUNCTIONS #########################

def onMouse(event, x, y, flags, param):
    """ the method called when the mouse is clicked """
    global D

    if event==cv.CV_EVENT_LBUTTONDOWN: # clicked the left button
        print "x, y are", x, y
        (b,g,r) = D["image"][y,x]
        print "r,g,b is", int(r), int(g), int(b)
        (h,s,v) = D["hsv"][y,x]
        print "h,s,v is", int(h), int(s), int(v)
        D["down_coord"] = (x,y)
        D["mouse_down"] = True
        
    elif event==cv.CV_EVENT_LBUTTONUP: # let go of the left button
        print "x, y are", x, y
        (b,g,r) = D["image"][y,x]
        print "r,g,b is", int(r), int(g), int(b)
        (h,s,v)  = D["hsv"][y,x]
        print "h,s,v is", int(h), int(s), int(v)
        D["up_coord"] = (x,y)
        D["mouse_down"] = False

        if D["mode"] == "start":
            D["sections"] = [ ['a', D["up_coord"], D["down_coord"]] ]
            process_section()
        elif D["mode"] == "add":
            D["sections"].append(['a', D["up_coord"], D["down_coord"]])
            process_section()
        elif D["mode"] == "subtract":
            # put lower coordinates first
            x0 = D["down_coord"][0]
            y0 = D["down_coord"][1]
            x1 = D["up_coord"][0]
            y1 = D["up_coord"][1]

            if x0 > x1:
                x0, x1 = x1, x0
            if y0 > y1:
                y0, y1 = y1, y0
            
            D["sections"].append(['s', (x0, y0), (x1, y1)])
            process_section()
        elif D["mode"] == "clear":
            D["sections"] = []

    elif D["mouse_down"] and event==cv.CV_EVENT_MOUSEMOVE: # mouse just moved
        D["up_coord"] = (x,y)

    elif event==cv.CV_EVENT_RBUTTONDOWN and D['obstacle']['adding']== False and D['vectors']['adding']== False:
        D['target']['targeting'] = True
        D['target']['xytar'] = (x,y)
        print "target location is:", D['target']['xytar']
        
    elif event == cv.CV_EVENT_RBUTTONDOWN and D['obstacle']['adding']== True and D['vectors']['adding']== False:
        D['obstacle']['avoiding'] = True
        D['obstacle']['xyobs'] = (x,y)
        print "obstacle location is:", D['obstacle']['xyobs']
        D['obstacle']['adding']= False
        D['vectors']['showobs'] = True

    elif event == cv.CV_EVENT_RBUTTONDOWN and D['vectors']['adding']== True and D['obstacle']['adding'] == False:
        D['vectors']['fakerob'] = (x,y)
        print "The vector point is:", D['vectors']['fakerob']
        D['vectors']['adding']= False
        D['vectors']['showtar']= True

def check_key_press(key_press):
    """ this handler is called when a real key press has been
        detected, and updates everything appropriately
    """
    # get D so that we can change values in it
    global D
    D["last_key_pressed"] = key_press

    # So we know what we'll do if we get one of these key presses
    key_dictionary = {ord('w'): "threshed_image",\
                      ord('r'): "red",\
                      ord('t'): "green",\
                      ord('y'): "blue",\
                      ord('f'): "red_threshed",\
                      ord('g'): "green_threshed",\
                      ord('h'): "blue_threshed",\
                      ord('a'): "hue",\
                      ord('s'): "sat",\
                      ord('d'): "val",\
                      ord('z'): "hue_threshed",\
                      ord('x'): "sat_threshed",\
                      ord('c'): "val_threshed" }

    if key_press == ord('q') or key_press == 27: # if a 'q' or ESC was pressed
        print "quitting"
        rospy.signal_shutdown( "Quit requested from keyboard" )

    elif key_press == ord('h'):
        print " Keyboard Command Menu"
        print " =============================="
        print " q    : quit"
        print " ESC  : quit"
        print " h    : help menu"
        print " w    : show total threshold image in threshold window"
        print " r    : show red image in threshold window"
        print " t    : show green image in threshold window"
        print " y    : show blue image in threshold window"
        print " f    : show thresholded red image in threshold window"
        print " g    : show thresholded blue image in threshold window"
        print " h    : show thresholded green image in threshold window"
        print " a    : show hue image in threshold window"
        print " s    : show saturation image in threshold window"
        print " d    : show value image in threshold window"
        print " z    : show thresholded hue image in threshold window"
        print " x    : show thresholded saturation image in threshold window"
        print " c    : show thresholded value image in threshold window"
        print " v    : prints threshold values to file ( will overwrite old file )"
        print " b    : loads threshold values from file"
        print " u    : mousedrags will no longer set thresholds, kept values will be cleared"
        print " i    : mousedrag will set thresholds to area within drag, " + \
            "resets on new click or drag"
        print " j    : mousedrags will remove the area under consideration, " + \
            " must have set an area in 'i' mode first"
        print " k    : mousedrags will add the area under consideration, " + \
            " must have set an area in 'i' mode first"

    elif key_press == ord('v'):
        x = D['thresholds']
        f = open( "./thresh.txt", "w" ) # open the file "thresh.txt" for writing
        print >> f, x # print x to the file object f
        f.close() # it's good to close the file afterwards

    elif key_press == ord('b'):
        f = open( "./thresh.txt", "r" ) # open the file "thresh.txt" for reading
        data = f.read() # read everything from f into data
        x = eval( data ) # eval is Python's evaluation function
        # eval evaluates strings as if they were at the Python shell
        f.close() # its good to close the file afterwards

        # Set threshold values in D
        D['thresholds'] = x

        # Update threshold values on actual sliders
        for i in ['low_red', 'high_red', 'low_green', 'high_green', 'low_blue', 'high_blue',\
                      'low_hue', 'high_hue', 'low_sat', 'high_sat', 'low_val', 'high_val']:
            cv.SetTrackbarPos(i, 'sliders', D['thresholds'][i])

    elif key_press == ord('H'):
        f = open( "./levels.txt", "r" ) # open the file "thresh.txt" for reading
        data = f.read() # read everything from f into data
        x = eval( data ) # eval is Python's evaluation function
        # eval evaluates strings as if they were at the Python shell
        f.close() # its good to close the file afterwards

        # Set threshold values in D
        D['thresholds'] = x

        # Update threshold values on actual sliders
        for i in ['low_red', 'high_red', 'low_green', 'high_green', 'low_blue', 'high_blue',\
                      'low_hue', 'high_hue', 'low_sat', 'high_sat', 'low_val', 'high_val']:
            cv.SetTrackbarPos(i, 'sliders', D['thresholds'][i])

    elif key_press == ord('P'):
        f = open( "./reset.txt", "r" ) # open the file "thresh.txt" for reading
        data = f.read() # read everything from f into data
        x = eval( data ) # eval is Python's evaluation function
        # eval evaluates strings as if they were at the Python shell
        f.close() # its good to close the file afterwards

        # Set threshold values in D
        D['thresholds'] = x

        # Update threshold values on actual sliders
        for i in ['low_red', 'high_red', 'low_green', 'high_green', 'low_blue', 'high_blue',\
                      'low_hue', 'high_hue', 'low_sat', 'high_sat', 'low_val', 'high_val']:
            cv.SetTrackbarPos(i, 'sliders', D['thresholds'][i])
    

    elif key_press == ord('u'):
        D["mode"] = "clear"
        D["sections"] = []
        D["multiple_number"] = 0

    elif key_press == ord('i'):
        D["mode"] = "start"

    elif key_press == ord('j'):
        if len(D["sections"]) > 0:
            D["mode"] = "subtract"
        else:
            print "Cannot switch modes, need a starting area first. Press 'i' " + \
                "to select a starting area."

    elif key_press == ord('k'):
        if len(D["sections"]) > 0:
            D["mode"] = "add"
        else:
            print "Cannot switch modes, need a starting area first. Press 'i' " + \
                "to select a starting area."
    
    elif key_press in key_dictionary.keys():
        D["current_threshold"] = key_dictionary[key_press]

    elif key_press == 32:
        D['state'] = "Keyboard"
        tank(0,0)

    elif key_press == 81:
        print "turning left"
        tank(-300, 300)

    elif key_press == 83:
        print "you pressed right"
        tank(300, -300)

    elif key_press == 82:
        print "going forwards"
        tank(300, 300)

    elif key_press == 84:
        print "going backwards"
        tank(-300, -300)


    elif key_press == ord('F'):
        print "pressing an F..."
        D['state'] = "Forward"
        D['robpos']['ini'] = robotlocation()
        FSM1()

    elif key_press == ord('`'):
            D['obstacle']['adding'] = True

    elif key_press == ord('9'):
            # print "you pressed 9"
            D['vectors']['adding'] = True

def FSM1(timer_event=None ):
    """ the "go" state machine """

    global D
    
    print "Now in state", D['state']

    if D['state'] == "Keyboard": # user stopped our state machine!
        print "State machine go has been stopped!"
        tank(0,0) # stop the robot
        distancegiver()
        #print "The distances the robot traveled", D['robpos']['dist']
        return

    elif D['state'] == "Bumped":
        D['state'] = "Backward"
        FSM1()
        return
        

    elif D['state'] == "Forward": # go forward state
        tank(50,50)
        #print "The initial robot position is:", D['robpos']['ini']
        rospy.Timer( rospy.Duration(2.0), FSM1, oneshot=True )
        D['state'] = "Rotating"
        return


    elif D['state'] == "Rotating":
        #D['robpos']['fin'] = robotlocation()
        anglerob = anglegiver(D['robpos']['fin'],D['robpos']['ini'])
        angletar = anglegiver(D['vectors']['netvect'],D['robpos']['fin'])
        anglerot = anglerob - angletar
        
        
        if anglerot < 0:
            angleone = 360-abs(anglerot)
            
        else:
            angleone = anglerot-360

        angletwo = abs(anglerot)
            
        print "angle one is:", angleone
        print "angle two is:", angletwo
        
        if(abs(angleone)<angletwo):
            anglerot = angleone
            
        timeforrot = math.fabs(anglerot)/30

        ##The gazillion print statements##
       # print "the angle between the robot and the target:",angletar
       # print "The robots angle is:", anglerob
       # print "The final robots position is", D['robpos']['fin']
       # print "angle to rotate", anglerot
       # print "time to rotate", timeforrot
       # print "distance to target", D['target']['totarg']
       # print "Distance to obstacle", D['obstacle']['toobs']


        #Checks proper direction to turn#
        if anglerot > 0:
            tank(-50, 50)
        if anglerot < 0:
            tank(50, -50)

        if anglerot != 0:
            rospy.Timer( rospy.Duration(timeforrot), FSM1, oneshot=True )
        D['robpos']['ini'] = D['robpos']['fin']
        D['robpos']['ini'] = robotlocation()
        D['state'] = "Forward"
        return

    elif D['state'] == "Backward": # go backward state
        tank(-50,-50)
        rospy.Timer( rospy.Duration(2.0), FSM1, oneshot=True )
        # set the state we'd like to wake up in
        D['state'] = "Keyboard"
        return

    elif D['state'] == "InRange":
        print "Found my Target!"
        tank(0,0)
        D['state'] = "Keyboard"
        return
        
    else: # state of confusion
        print "I have no idea what state I'm in! Stopping."
        tank(0,0)
        D['state'] = "Keyboard"
        return


# Function for changing the slider values
def change_slider(name, new_threshold):
    # get D so that we can change values in it
    global D
    D['thresholds'][name] = new_threshold


def ros_services():
        """ sets data members tank and song, analogous to lab 1 """
        # obtain the tank service
        rospy.wait_for_service('tank') # won't continue until the "tank" service is on
        # obtain the song service
        rospy.wait_for_service('song') #

        # this next line subscribes to sensorPacket
        # it tells ROS that the method handle_sensor_data
        # will handle each incoming sensorPacket
        rospy.Subscriber('sensorPacket', SensorPacket, handle_sensor_data)

def handle_sensor_data( data ):
        """
           handle_sensor_data is called every time the robot gets a new sensorPacket
        """
        global D
        #  uncomment this to see all of the fields of the sensorPacket
        #print dir( data )

        # check for a bump
        if data.bumpRight or data.bumpLeft:
            print "Bumped!, but for now going into keyboard"
            D['state'] = "Keyboard"
            #D['state'] = "Bumped"

        if data.wheeldropCaster:
            print "Wheel drop!"
            tank(0,0)
            rospy.signal_shutdown("robot picked up... so we're shutting down")


def handle_data(data):
    """ this method processes data given:
        - key presses
        - images from Kinect
    """
    # get D so that we can change values in it
    global D

    # Get the incoming image from the Kinect
    D["image"] = D["bridge"].imgmsg_to_cv(data, "bgr8")

    if D["created_images"] == False:
        # Initialize the additional images we need for processing
        # We only need to run this one time
        init_images()
        D["created_images"] = True

    # Recalculate threshold image
    if D['mode'] != "multiple":
        threshold_image()

    # Recalculate blob in main image
        find_biggest_region()
    else:
        threshold_mulimage(D['multiple_number']-1)
        find_center(D['multiple_number']-1)

    


################## SIMILAR THINGS SHOULD BE ADDED##################
    targetcircle()
    obstaclecircle() 
    arrived()
    drawtarvector()
    drawobsvector()
    drawnetvector()
################## SIMILAR THINGS SHOULD BE ADDED END #############

    # Check on the display of dragged section
    mouse_section()

    # Get any incoming keypresses
    # To get input from keyboard, we use cv.WaitKey
    # Only the lowest eight bits matter (so we get rid of the rest):
    key_press_raw = cv.WaitKey(5) # gets a raw key press
    key_press = key_press_raw & 255 # sets all but the low 8 bits to 0
    
    # Handle key presses only if it's a real key (255 = "no key pressed")
    if key_press != 255:
        check_key_press(key_press)

    # Update the displays:
    # Main image:
    cv.ShowImage('image', D["image"])

    # Currently selected threshold image:
    cv.ShowImage('threshold', D[ D["current_threshold"] ] )

##################### END CALLBACK FUNCTIONS #######################

def main():
    """ the main program that sets everything up
    """
    
    # Initialize our node
    rospy.init_node('blobFinder')

    # Initialize all the global variables we will need
    init_globals()

    # Subscribe to the image_color topic
    # Each time new data comes in (Kinect color image, keypresses), we will call handle_data 
    # to deal with it
    rospy.Subscriber('/camera/rgb/image_color', sm.Image, handle_data)

    ros_services()
    
    # Run until something stops us, such as a call to rospy.signal_shutdown
    rospy.spin()

# this is the "main" trick: it tells Python
# what code to run when you run this as a stand-alone script:
if __name__ == "__main__":
    main()
