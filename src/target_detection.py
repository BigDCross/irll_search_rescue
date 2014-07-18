#!/usr/bin/env python

# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies
import roslib; roslib.load_manifest('irll_search_rescue')
import rospy

# Import the two types of messages we're interested in
from sensor_msgs.msg import Image        # for receiving the video feed
from ardrone_autonomy.msg import Navdata # for receiving navdata feedback

# We need to use resource locking to handle synchronization between GUI thread and ROS topic callbacks
from threading import Lock

# An enumeration of Drone Statuses
from drone_status import DroneStatus

# The GUI libraries
'''from PySide import QtCore, QtGui'''

# We need an image convereted to use opencv
# GABE
import cv2.cv as cv
import cv2 as cv2
from image_converter import ToOpenCV, ToRos
import numpy as np
from std_msgs.msg import Int32, Float32, Bool, String

class TargetDetection ():   
    def __init__(self):
        rospy.init_node('TargetDetection')
        # Subscribe to the drone's video feed, calling self.ReceiveImage when a new frame is received
        self.subVideo = rospy.Subscriber('/camera/rgb/image_raw', Image, self.processVideo)
        self.pubPuck = rospy.Publisher('/irll_search/has_puck', String)

    def processVideo (self, ros_image):
        # Holds the image frame received from the drone and later processed by the GUI
        self.image = ros_image
        #cv.NamedWindow("windowimage", cv.CV_WINDOW_AUTOSIZE)

        # Convert the ROS image into a QImage which we can display
        image_cv = ToOpenCV(self.image)
        frame = np.asarray(image_cv)
        detect = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
         
        # ADJUST these values to get a greater range of color
        #lower_gray = np.array([ 8, 6, 71], dtype=np.uint8)
        #upper_gray = np.array([50,45,135], dtype=np.uint8)

        #lower_white = np.array([ 0, 0,245], dtype=np.uint8)
        #upper_white = np.array([10,10,255], dtype=np.uint8)
        # Threshold the HSV image to get only white color
        #goal_mask = cv2.inRange(hsv, lower_white, upper_white)

        # Threshold the HSV image to get only gray colors
        #mask = cv2.inRange(hsv, lower_gray, upper_gray)
        flag, detect = cv2.threshold(detect, 160, 180, cv2.THRESH_BINARY)
        has_puck = detect[40][320]
        if has_puck == 0:
            self.pubPuck.publish("1")
        else:
            self.pubPuck.publish("0")
        #flag, goal_mask = cv2.threshold(goal_mask, 100, 255, cv2.THRESH_BINARY)                        

        #cv2.imshow("Target Detection", detect)
        #cv2.imshow("mask", mask)
        #cv.ShowImage("windowimage", image_cv)
        cv.WaitKey(25)

target = TargetDetection()
rospy.spin()
