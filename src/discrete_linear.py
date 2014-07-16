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
        #rospy.init_node('turtle_discrete_linear')
	self.rotate = TurtlebotDiscreteRotation ()
        self.pub = rospy.Publisher('/mobile_base/commands/velocity', Twist)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.saveModelStates)

	self.target_position = (0,0,0)

	self.success = 0

	self.cell_length = 1

	self.turtlebot_position = None

	self.last_turtlebot_position = None

	print "Setup Finished"
	print
	print "--------------"
	print

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
	print "New Destination Set"
	print

    def forward(self):
	
	#Forward in the +x direction
	if self.rotate.get_direction() == 3.0:
		x = self.target_position[0] + self.cell_length
		x = math.ceil(x)
	        y = math.ceil(self.target_position[1])

	#Foward in the +y direction
	elif self.rotate.get_direction() == 0.0:
		y = self.target_position[1] + self.cell_length
		x = math.ceil(self.target_position[0])
		y = math.ceil(y)

	#Forward in the -x direction
	elif self.rotate.get_direction() == 1.0:
		x = self.target_position[0] - self.cell_length
		y = math.floor(self.target_position[1])
		x = math.floor(x)

	#Forward in the -y direction
	elif self.rotate.get_direction() == 2.0:
		y = self.target_position[1] - self.cell_length
		x = math.floor(self.target_position[0])
		y = math.floor(y)
	
	print (self.rotate.get_direction())
	self.set_destination(x,y)

    def update_movement (self):
	linear_twist = Twist ()
	
	#If the turtlebot and the puck are not in the same place
	if abs(self.turtlebot_position.x - self.target_position[0]) >= .1 or abs(self.turtlebot_position.y - self.target_position[1]) >= .1:
		self.success = 0

		#If we aren't getting closer turn left
		if abs(self.turtlebot_position.x - self.target_position[0]) >= abs(self.last_turtlebot_position.x - self.target_position[0]) or abs(self.turtlebot_position.y - self.target_position[1]) >= abs(self.last_turtlebot_position.y - self.target_position[1]):

			#if you're to the left of the puck, turn right
			if self.turtlebot_position.x > self.target_position[0] or self.turtlebot_position.y > self.target_position[1]:
				linear_twist.angular.z = -.1 
				linear_twist.linear.x = .05 

			else:
				#else, turn left
				linear_twist.angular.z = .1
				linear_twist.linear.x = .05	

		#else go straight
		else:
			linear_twist.angular.z = 0
			linear_twist.linear.x = .05

		self.publishDrive (linear_twist)

	else:
		self.success = 1

	#print "Current Position =  X: " + str(self.turtlebot_position.x) + " Y: " + str(self.turtlebot_position.y)
	#print "Target Position = X: " + str(self.target_position[0]) + " Y: " + str(self.target_position[1])


if __name__=="__main__":
	#print "GAP"
	#linear = TurtlebotDiscreteLinear ()
	#print "HEIEHOIEH"
	rotation = TurtlebotDiscreteRotation ()
	sleep(3)

	print "Turning Right!"
	rotation.rotate90Right ()
	while not rotation.rotation_success:
       		rotation.checkRotation ()
		print "Turning!"
	
 	#print "It's Moving Forward!"
	#linear.forward()
	#while not linear.success :
	#	linear.success = 0

	print "Turning LEft"
	rotation.rotate90Left ()
	while not rotation.rotation_success:
       		rotation.checkRotation ()

	rospy.spin()        
