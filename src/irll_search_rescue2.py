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

class SearchAndRescue ():
    def __init__(self):
        self.x = 0


	self.cell_length = 1
	self.target_position = None
	self.turtlebot_position = None
	self.last_turtlebot_position = None
        rospy.init_node('irll_search_rescue')
        self.pub = rospy.Publisher('/mobile_base/commands/velocity', Twist)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.saveModelStates)

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
	   self.target_position = (self.turtlebot_position.x, self.turtlebot_position.y, 0)

        self.test ()
 
    def set_destination(self,x,y):
	z = 0
	self.target_position = (x,y,z)
	sleep(8)

    #from Dustin
    def get_direction(self):
	return 0

    def forward(self):
	self.speed = 1
	if self.get_direction() == 0:
	   x = self.target_position[0] + self.cell_length
           x = math.ceil(x)
	   y = self.turtlebot_position.y
	elif self.get_direction() == 1:
	   y = self.target_position[1] + self.cell_length
	   x = self.turtlebot_position.x
           y = math.ceil(y)
	elif self.get_direction() == 2:
	   x = self.target_position[0] - self.cell_length
	   y = self.turtlebot_position.y
           x = math.ceil(x)
	else:
	   y = self.target_position[1] - self.cell_length
	   x = self.turtlebot_position.x
           y = math.ceil(y)
		
	self.set_destination(x,y)

    def backwards(self):
	self.speed = -1
	if self.get_direction() == 2:
	   x = self.target_position[0] + self.cell_length
           x = math.ceil(x)
	   y = self.turtlebot_position.y
	elif self.get_direction() == 3:
	   y = self.target_position[1] + self.cell_length
	   x = self.turtlebot_position.x
           y = math.ceil(y)
	elif self.get_direction() == 0:
	   x = self.target_position[0] - self.cell_length
	   y = self.turtlebot_position.y
           x = math.ceil(x)
	else:
	   y = self.target_position[1] - self.cell_length
	   x = self.turtlebot_position.x
           y = math.ceil(y)
		
	self.set_destination(x,y)
	
			
    def rotate_right(self):
	print "Rotate"

    def rotate_left(self):
	print "Rotate"

    def test (self):
	print "banana"
        twist = Twist ()
        #print self.turtlebot_orientation
        #print
	
	#If the turtlebot and the puck are not in the same place
	if abs(self.turtlebot_position.x - self.target_position[0]) >= .3 or abs(self.turtlebot_position.y - self.target_position[1]) >= .3:
	  #if we aren't getting closer turn left
	  if abs(self.turtlebot_position.x - self.target_position[0]) >= abs(self.last_turtlebot_position.x - self.target_position[0]) or abs(self.turtlebot_position.y - self.target_position[1]) >= abs(self.last_turtlebot_position.y - self.target_position[1]):
	      #if you're to the left of the puck, turn right
 	      if self.turtlebot_position.x > self.target_position[0] or self.turtlebot_position.y > self.target_position[1]:
                twist.angular.z = -.1 * self.speed
 	        twist.linear.x = .2 * self.speed	
	      else:
	        #else, turn left
                twist.angular.z = .1 * self.speed
 	        twist.linear.x = .2 * self.speed	
	  #else go straight
	  else:
	      twist.angular.z = 0
	      twist.linear.x = .2 * self.speed

          self.publishDrive (twist)


if __name__=="__main__":
    sar = SearchAndRescue ()
    sleep(2)
    sar.forward()
    sar.forward()
    sar.forward()
    sar.forward()
    #rospy.spin()        
    while 1:
	#print "hi"
	pass
