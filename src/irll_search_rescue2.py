#!/usr/bin/env python

#TODO FIX LEFT

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

#From Dustin

import transformations as tf

#----------

class SearchAndRescue ():
    def __init__(self):

	#From Lorin
	self.arrived = 1
        self.x = 0
	self.speed = 1
	self.cell_length = .5
	self.target_position = None
	self.turtlebot_position = None
	self.last_turtlebot_position = None
        rospy.init_node('turtle_discrete_movement')
        self.pub = rospy.Publisher('/mobile_base/commands/velocity', Twist)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.saveModelStates)

	#From Dustin
        self.desired_orientation = 0.0
        self.success = 0
        #self.P = 0.4
        self.P = 0.0
        self.yaw = 0.0


    def publishDrive (self, twist):
        self.pub.publish(twist)

        self.x += 0.1
        if self.x > 1.5: self.x = 0.0
    
    def saveModelStates (self, model_states):
        m = model_states.name.index ("mobile_base")
	
	if self.turtlebot_position != None:
	    self.last_turtlebot_position = self.turtlebot_position
        self.turtlebot_position = model_states.pose[m].position
        self.turtlebot_orientation = model_states.pose[m].orientation
	if self.target_position == None:
	   self.target_position = (0, 0, 0)

	#From Dustin
	
	self.euler_angles = tf.euler_from_quaternion([self.turtlebot_orientation.w, self.turtlebot_orientation.x, self.turtlebot_orientation.y, self.turtlebot_orientation.z])

        self.yaw = pi - (self.euler_angles[0] + pi)
        self.compass = ((self.yaw - pi/4) // (pi / 2))

        if self.compass == -1.0:
            self.compass = 3.0

        self.checkRotation ()
        self.printModelStates ()

	#----------

        self.update_movement ()
 
    def set_destination(self,x,y):
	z = 0
        while not self.arrived:
          sleep(1)
 	self.target_position = (x,y,z)

    #from Dustin
    def get_direction (self):
        return self.compass
	#return 0

    def rotate90Right (self):
        self.P = 0.4
        if self.desired_orientation == -pi: self.desired_orientation = -pi/2
        elif self.desired_orientation == -pi/2: self.desired_orientation = 0.0
        elif self.desired_orientation == 0.0: self.desired_orientation = pi/2
        elif self.desired_orientation == pi/2: self.desired_orientation = -pi
        sleep(13)

    def rotate90Left (self):
        self.P = 0.4
        if self.desired_orientation == -pi: self.desired_orientation = pi/2
        elif self.desired_orientation == pi/2: self.desired_orientation = 0.0
        elif self.desired_orientation == 0.0: self.desired_orientation = -pi/2
        elif self.desired_orientation == -pi/2: self.desired_orientation = -pi
        sleep(13)

    def checkRotation (self):
        #print "Desired Orientation: ",
        #print self.desired_orientation
        error = self.desired_orientation - self.yaw
        #print "Error: ",
        #print abs (error)

        twist = Twist ()

        #print "Success: ",
        #print self.success
        if abs (error) > 0.2:
            #print "Correcting!"
            twist.angular.z = self.P * error
            self.publishDrive (twist)
            self.success = 0
        else:
            self.success = 1

    def printModelStates (self):
	'''
        print "Yaw: ",
        print self.yaw
        print "Compass: ",
        print self.compass
        print "--------------------"
	'''
	pass
     
    #--------


    def forward(self):
	self.speed = 1
	if self.get_direction() == 1.0 or self.get_direction() == -3.0:
	   for z in range(1000):
	       print "Moving forward in the x direction"
	   x = self.target_position[0] + self.cell_length
           x = math.ceil(x)
	   y = math.ceil(self.target_position[1])
	elif self.get_direction() == -2.0:
	   for z in range(1000):
	       print "Moving forward in the y direction"
	   y = self.target_position[1] + self.cell_length
	   x = math.ceil(self.target_position[0])
           y = math.ceil(y)
	elif self.get_direction() == 3.0:
	   for z in range(1000):
	       print "Moving backwards in the x direction"
	   x = self.target_position[0] - self.cell_length
	   y = math.floor(self.target_position[1])
           x = math.floor(x)
	elif self.get_direction() == 0.0:
	   for z in range(1000):
	       print "Moving backwards in the y direction"
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
    #sar.forward()
    print "1"
    #sar.rotate90Right()
    #sar.forward()
    #sar.rotate90Left()
    #sleep(10)
    #sar.forward()
    #sar.rotate90Left()
    #sleep(10)
    #sar.forward()
    #sleep(15)
    #sar.rotate90Left()
    #sleep(10)
    #sar.forward()
    #sleep(15)
    #sar.rotate90Left()
    #sleep(10)
    #sar.forward()
    sar.rotate90Left()
    sleep(10)
    sar.rotate90Left()
    sleep(10)


    #sar.rotate90Left() sar.forward() sar.forward()
    print "3"
    #sar.rotate90Left()
    print "4"
    #sar.forward()

    rospy.spin()        
