#TODO
#Find Out when I need to toggle the camera and how

'''
Ardrone flys in a grid path relative to take off. Using SLAM, it can keep track of it's
relative coodinates and relay the coordinates of the target to the turtlebot once the target
is found. The Ardrone must toggle between cameras to keep track of it's position as well as 
lookfor the target.

Subscribers
	- Drone estimated position
	- Ardrone Image Raw
Publishers
	- Auto pilate controller (/tum_ardrone/com)
	- Publish Coordinates to turtlebot for Puck
'''

from sensor_msgs.msg import Image 	# for receiving the video feed
import cv2
import numpy as np
from tum_ardrone import filter_state
import os
from blob_detection import Vison_Processor
from image_converter import ToOpenCV

class ARsearch () :
  def __init__ ():
    self.confirmed_target = 0
    self.status = 0
    self.currentState = (0,0,0,0)
    self.targetCoordinates = (0,0,0,0)
    self.vision = Vision_Processor ()
    self.autoPub = rospy.Publisher('/tum_ardrone/com', String)
    self.targetPub = rospy.Publisher('/ardrone/targetCoordinates', String)
    self.subVideo = rospy.Subscriber('/ardrone/image_raw', Image, self.find_target)
    self.relPosition = rospy.Subscriber('/ardrone/predictedPose', filter_state, self.get_state)

  #Moves the Ardrone to the given coordinates relative to where it has taken off
  def go_to (x, y, z, yaw):
    self.autoPub.publish("c clearCommands")
    self.autoPub.publish("c goto " + str(x) + " " + str(y) + " " + str(z) + " " + str(yaw))
  
  #updates the current coordinates of the Ardrone relative to where it took off
  def get_state (state):
    self.currentState[0] = state->x
    self.currentState[1] =  state->y
    self.currentState[2] = state->z
    self.currentState[3] = state->yaw

  #Because both cameras need to be used, we must toggle between them
  #Index 1 = front camera. Index 2 = bottom camera.
  def toggle_cam(index):
    os.system("rosservice call /ardrone/setcamchannel " + str(index))

  #Detects the red puck used as our tempory target
  def find_target(image):
    image_cv = ToOpenCV(image)
    if self.vision.process_image(image_cv):
      self.go_to(self.currentState[0], self.currentState[1], self.currentState[2], self.currentState[3])
      self.confirmed_target += 1
    else:
      self.confirmed_target -= 1
      if self.confirmed_target < 0:
	self.confirmed_target = 0
        self.fly_grid()
    if self.confirmed_target > 200:
      self.targetCoordinates = self.currentState
      self.targetPub.publish (str(self.targetCoordinates[0]) + " " + str(self.targetCoordinates[0]) + " " + str(self.targetCoordinates[0]) + " " + str(self.targetCoordinates[0]))
      self.go_to (0,0,0,0)
      self.autoPub.publish("c land")

  #Flys in a set grid pattern until the target is found
  def fly_grid():
  
    self.autoPub.publish("c autoInit 500 800 4000 0.5")
    self.autoPub.publish("c setReference $POSE$")
    self.autoPub.publish("c setInitialReachDist 0.2") 
    self.autoPub.publish("c setStayWithinDist 0.3")
    self.autoPub.publish("c setStayTime 3")
    self.autoPub.publish("c lockScaleFP")

    self.autoPub.publish("c goto 0 0 0 0")
    self.autoPub.publish("c goto 0 0 1 0")
    self.autoPub.publish("c goto 0 0 0 0")

    while (self.status == 0):
      self.autoPub.publish("c goto -0.5 -0.5 0 0")
      self.autoPub.publish("c goto -0.5 0.5 0 0")
      self.autoPub.publish("c goto -0.3 0.5 0 0")
      self.autoPub.publish("c goto -0.3 -0.5 0 0")
      self.autoPub.publish("c goto -0.1 -0.5 0 0")
      self.autoPub.publish("c goto -0.1 0.5 0 0")
      self.autoPub.publish("c goto 0.1 0.5 0 0")
      self.autoPub.publish("c goto 0.1 -0.5 0 0")
      self.autoPub.publish("c goto 0.3 -0.5 0 0")
      self.autoPub.publish("c goto 0.3 0.5 0 0")
      self.autoPub.publish("c goto 0.5 0.5 0 0")
      self.autoPub.publish("c goto 0.5 -0.5 0 0")

  
  
if __name__=="__main__":
  search = ARsearch ()    
  search.fly_grid()
