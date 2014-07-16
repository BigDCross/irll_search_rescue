#!/usr/bin/env python

import roslib
#roslib.load_manifest('')
import rospy

from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import String

import sys, select, termios, tty
from math import atan2, pi
from time import sleep
import math
import transformations as tf

#Include Rotation Class
from discrete_rotation import TurtlebotDiscreteRotation

class TurtlebotDiscreteLinear ():
    def __init__(self):
        rospy.init_node('turtle_discrete_linear')
        self.pub = rospy.Publisher('/mobile_base/commands/velocity', Twist)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.saveModelStates)
	self.rotate = TurtlebotDiscreteRotation ()

	self.target_position = (0,0,0)

	self.success = 0

	self.cell_length = 1

	self.turtlebot_position = None

	self.last_turtlebot_position = None

    def publishDrive (self, twist):
        self.pub.publish(twist)
    
    def saveModelStates (self, model_states):
        m = model_states.name.index ("mobile_base")
	
	if self.turtlebot_position != None:
		self.last_turtlebot_position = self.turtlebot_position

        self.turtlebot_position = model_states.pose[m].position

	if self.target_position == None:
		self.target_position = (0, 0, 0)

        self.update_movement ()
 
    def set_destination(self,x,y):
	z = 0
 	self.target_position = (x,y,z)

    def forward(self):
	self.speed = 1
	
	#Forward in the +x direction
	if self.rotate.get_direction() == 3.0:
		x = self.target_position[0] + self.cell_length
		x = math.ceil(x)
	        y = math.ceil(self.target_position[1])

	#Foward in the +y direction
	elif self.rotate.get_direction() == -2.0:
		y = self.target_position[1] + self.cell_length
		x = math.ceil(self.target_position[0])
		y = math.ceil(y)

	#Forward in the -x direction
	elif self.rotate.get_direction() == 1.0:
		x = self.target_position[0] - self.cell_length
		y = math.floor(self.target_position[1])
		x = math.floor(x)

	elif self.rotate.get_direction() == 0.0:
		y = self.target_position[1] - self.cell_length
		x = math.floor(self.target_position[0])
		y = math.floor(y)
	
	self.set_destination(x,y)

    def update_movement (self):
	#print "banana"
	linear_twist = Twist ()
        #print self.turtlebot_orientation
        #print
	
	#If the turtlebot and the puck are not in the same place
	if abs(self.turtlebot_position.x - self.target_position[0]) >= .1 or abs(self.turtlebot_position.y - self.target_position[1]) >= .1:
	  self.arrived = 0
	  #if we aren't getting closer turn left
	  if abs(self.turtlebot_position.x - self.target_position[0]) >= abs(self.last_turtlebot_position.x - self.target_position[0]) or abs(self.turtlebot_position.y - self.target_position[1]) >= abs(self.last_turtlebot_position.y - self.target_position[1]):
	      #if you're to the left of the puck, turn right
 	      if self.turtlebot_position.x > self.target_position[0] or self.turtlebot_position.y > self.target_position[1]:
                linear_twist.angular.z = -.1 * self.speed
 	        linear_twist.linear.x = .05 * self.speed
		print "Turning Right!"	
	      else:
		print "Turning Left!"	
	        #else, turn left
                #linear_twist.angular.z = .1 * self.speed
 	        linear_twist.linear.x = .05 * self.speed	
	  #else go straight
	  else:
	      linear_twist.angular.z = 0
	      linear_twist.linear.x = .05 * self.speed

          self.publishDrive (linear_twist)

	else:
	  self.arrived = 1

	#print "Current Position =  X: " + str(self.turtlebot_position.x) + " Y: " + str(self.turtlebot_position.y)
	#print "Target Position = X: " + str(self.target_position[0]) + " Y: " + str(self.target_position[1])


if __name__=="__main__":
    sar = SearchAndRescue ()
    sleep(2)
    sar.forward()

    rospy.spin()        
