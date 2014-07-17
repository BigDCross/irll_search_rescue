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

	sleep(5)

    def publishDrive (self, twist):
        self.pub.publish(twist)

    def getTargetPose (self):
	return self.puck_position
   
    def getTurtlePose (self):
	return self.turtlebot_position	
 
    def saveModelStates (self, model_states):
        m = model_states.name.index ("mobile_base")
	p = model_states.name.index ("unit_cylinder_2")	
	if self.turtlebot_position != None:
		self.last_turtlebot_position = self.turtlebot_position

        self.turtlebot_position = model_states.pose[m].position
	self.puck_position = model_states.pose[p].position

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
		#x = math.ceil(x)
	        y = self.target_position[1]

	#Foward in the +y direction
	elif self.rotate.get_direction() == 0.0:
		y = self.target_position[1] + self.cell_length
		x = self.target_position[0]
		#y = math.ceil(y)

	#Forward in the -x direction
	elif self.rotate.get_direction() == 1.0:
		x = self.target_position[0] - self.cell_length
		y = self.target_position[1]
		#x = math.floor(x)

	#Forward in the -y direction
	elif self.rotate.get_direction() == 2.0:
		y = self.target_position[1] - self.cell_length
		x = self.target_position[0]
		#y = math.floor(y)
	
	print (self.rotate.get_direction())
	self.set_destination(x,y)

    def update_movement (self):
	linear_twist = Twist ()
	
	#If the turtlebot and the puck are not in the same place
	if abs(self.turtlebot_position.x - self.target_position[0]) >= .2 or abs(self.turtlebot_position.y - self.target_position[1]) >= .2:
		self.success = 0

		print self.rotate.get_direction ()
		#if you are facing the positive x direction
		if self.rotate.get_direction () == 3.0 and abs(self.turtlebot_position.y - self.target_position[1]) > .04:
			#If target is on the right, rotate right
			if self.turtlebot_position.y >= self.target_position[1]:
				print "Should be turning Right"
				linear_twist.angular.z = -0.035
                       		linear_twist.linear.x = .05
			elif self.turtlebot_position.y < self.target_position[1]:
				print "Should be turning Left"
                                linear_twist.angular.z = 0.035
                                linear_twist.linear.x = .05
					
		# -x direction
		elif self.rotate.get_direction () == 1.0 and abs(self.turtlebot_position.y - self.target_position[1]) > .05:
			print "-x direction"
			if self.turtlebot_position.y > self.target_position[1]:
                	        linear_twist.angular.z = 0.035
                        	linear_twist.linear.x = .05
                        elif self.turtlebot_position.y < self.target_position[1]:
                                linear_twist.angular.z = -0.035
                                linear_twist.linear.x = .05
			
			# +y direction
		elif self.rotate.get_direction () == 0.0 and abs(self.turtlebot_position.x - self.target_position[0]) > .05:
			print "+y direction"
			if self.turtlebot_position.x > self.target_position[0]:
                        	linear_twist.angular.z = 0.035
                                linear_twist.linear.x = .05
                       	elif self.turtlebot_position.x < self.target_position[0]:
                                linear_twist.angular.z = -0.035
                                linear_twist.linear.x = .05

			# -y direction
		elif self.rotate.get_direction () == 2.0: #and abs(self.turtlebot_position.x - self.target_position[0]) > .1:
			print "-y direction"
			if self.turtlebot_position.x > self.target_position[0]:
                                linear_twist.linear.x = .05
				print "111"
                        	linear_twist.angular.z = 0.035 #positive is left?
                        elif self.turtlebot_position.x < self.target_position[0]:
                                #linear_twist.angular.z = 0.035
                                linear_twist.linear.x = .05
				print "222"
		else:
			print "Going Straight"
			linear_twist.angular.z = 0.0
			linear_twist.linear.x = .05

		self.publishDrive (linear_twist)

	else:
		self.success = 1

	print "Current Position =  X: " + str(self.turtlebot_position.x) + " Y: " + str(self.turtlebot_position.y)
	print "Target Position = X: " + str(self.target_position[0]) + " Y: " + str(self.target_position[1])

'''
if __name__=="__main__":
	linear = TurtlebotDiscreteLinear ()
	rotation = TurtlebotDiscreteRotation ()
	sleep(3)

	while 1:	
	 	print "It's Moving Forward!"
		linear.forward()
		while not linear.success :
			linear.success = 0
		sleep(5)
		print "Turning LEft"
		rotation.rotate90Left ()
		while not rotation.rotation_success:
       			rotation.checkRotation ()

		sleep(5)

	rospy.spin()
'''        
