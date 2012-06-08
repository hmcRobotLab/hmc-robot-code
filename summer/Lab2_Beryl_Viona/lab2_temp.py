#!/usr/bin/env python
import roslib; roslib.load_manifest('Frizzle')
import rospy
import cv_bridge
import cv
import sensor_msgs.msg as sm


class BlobFinder:
             
    # this creates self.threshed_image from self.image
    # by thresholding to the correct color
    def threshold_image(self):
        """ this method runs the image processing in order to
            create a black and white image out of self.image
        """
        # Use OpenCV to split the image up into channels,
        # saving them in their respective bw images
        cv.Split(self.image, self.blue, self.green, self.red, None)

        # This line creates a hue-saturation-value image
        cv.CvtColor(self.image, self.hsv, cv.CV_RGB2HSV)
        cv.Split(self.hsv, self.hue, self.sat, self.val, None)
        
        #Here is how OpenCV thresholds the images based on the slider values:
        cv.InRangeS(self.red, self.thresholds['low_red'],\
                    self.thresholds['high_red'], self.red_threshed)
        cv.InRangeS(self.green, self.thresholds['low_green'],\
                    self.thresholds['high_green'], self.green_threshed)
        cv.InRangeS(self.blue, self.thresholds['low_blue'],\
                    self.thresholds['high_blue'], self.blue_threshed)
        cv.InRangeS(self.hue, self.thresholds['low_hue'],\
                    self.thresholds['high_hue'], self.hue_threshed)
        cv.InRangeS(self.sat, self.thresholds['low_sat'],\
                    self.thresholds['high_sat'], self.sat_threshed)
        cv.InRangeS(self.val, self.thresholds['low_val'],\
                    self.thresholds['high_val'], self.val_threshed)

        #Multiply all the thresholded images into one "output" image,
        # named self.threshed_image
        cv.Mul(self.red_threshed, self.green_threshed, self.threshed_image)
        cv.Mul(self.threshed_image, self.blue_threshed, self.threshed_image)
        cv.Mul(self.threshed_image, self.hue_threshed, self.threshed_image)
        cv.Mul(self.threshed_image, self.sat_threshed, self.threshed_image)
        cv.Mul(self.threshed_image, self.val_threshed, self.threshed_image)
        
        #Erode and Dilate shave off and add edge pixels respectively
        cv.Erode(self.threshed_image, self.threshed_image, iterations = 1)
        cv.Dilate(self.threshed_image, self.threshed_image, iterations = 1)

        # no need to return: all of these are data members of self's object


    def find_biggest_region(self):
        """ finds all of the contours in self.threshed_image
            and then finds the largest one
        """
    
        #Create a copy image of thresholds then find contours on that image
        storage = cv.CreateMemStorage(0) # create memory storage for contours
        copy = cv.CreateImage(self.size, 8, 1)
        cv.Copy( self.threshed_image, copy )  # copy self.threshed_image

        # this is OpenCV's call to find all of the contours:
        contours = cv.FindContours(copy, storage, cv.CV_RETR_EXTERNAL,\
                                   cv.CV_CHAIN_APPROX_SIMPLE)
                                   
        # Next we want to find the *largest* contour
        if len(contours)>0:
            biggest = contours
            biggestArea=cv.ContourArea(contours)
            while contours != None:
                nextArea=cv.ContourArea(contours)
                if biggestArea < nextArea:
                    biggest = contours
                    biggestArea = nextArea
                contours=contours.h_next()
            
            #Use OpenCV to get a bounding rectangle for the largest contour
            br = cv.BoundingRect(biggest,update=0)
            #print br

            #print "in find_regions, br is", br

            # you will want to change these so that they draw
            # a box around the largest contour and a circle at
            # its center:

            #Example of drawing a red box
            #cv.PolyLine(self.image,[[(42,42),(42,100),
            #                         (100,100),(100,42)]],
            #                            1, cv.RGB(255, 0, 0))

            cv.PolyLine(self.image,[[(br[0],br[1]),(br[0],br[1] + br[3]),
                                     (br[0]+br[2],br[1]+br[3]),(br[0]+br[2],br[1])]],
                                        1, cv.RGB(255, 0, 0))           
            #Example of drawing a yellow circle
            cv.Circle(self.image,(br[0] + int(br[2]/2),br[1] + int(br[3]/2)), 10, cv.RGB(255, 255, 0),\
                      thickness=1, lineType=8, shift=0)
            
            #Draw the contours in white with inner ones in green
            cv.DrawContours(self.image, biggest, cv.RGB(255,255,255),
                            cv.RGB(0, 255, 0), 1, thickness=2, lineType=8,
                            offset=(0, 0))  
