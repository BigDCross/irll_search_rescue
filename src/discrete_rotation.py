#!/usr/bin/env python

import roslib
#roslib.load_manifest('')
import rospy

from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
import transformations as tf

import sys, select, termios, tty
from math import atan2, pi, acos, degrees
from time import sleep

class TurtlebotDiscreteRotation:
    def __init__(self):
        rospy.init_node('turtlebot_discrete_rotation')
        self.pub = rospy.Publisher('/mobile_base/commands/velocity', Twist)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.saveModelStates)

        self.desired_orientation = 0

        self.rotation_success = 0

        # PID Tuning Variable. Could be changed but keep the sign positive!
        self.P = 0.004

        self.yaw = 0.0

        self.compass = 0.0

    def deltaAngle (self, current, target):
        """
        DeltaAngle calculates the shortest angle between two angles
        accounting for the 360 degree runover as well as the sign of
        the error
        """

        shortestAngle = min (abs (target - current), abs (target - (current + 360)))
        if (target - current) > shortestAngle:
            return -shortestAngle
        else:
            if (target - current) > 0:
                return shortestAngle
            else:
                return -shortestAngle

    def publishDrive (self, twist):
        self.pub.publish(twist)

    def rotate90Right (self):
        self.rotation_success = 0

        self.desired_orientation -= 90
        self.desired_orientation = self.desired_orientation % 360

    def rotate90Left (self):
        self.rotation_success = 0

        self.desired_orientation += 90
        self.desired_orientation = self.desired_orientation % 360

    def checkRotation (self):
        twist = Twist ()

        error = self.deltaAngle (self.yaw, self.desired_orientation)

        # This could be pared down if target +- 10 is too large
        if abs (error) > 10:
            twist.angular.z = self.P * error
            self.publishDrive (twist)
            self.rotation_success = 0
        else:
            self.rotation_success = 1

        # Manual update at 5 hz
        sleep (.2)

    def printModelStates (self):
        #print "Position:"
        #print self.turtlebot_position.x
        #print self.turtlebot_position.y
        #print self.turtlebot_position.z
        #print "Orientation:"
        #print self.turtlebot_orientation.x
        #print self.turtlebot_orientation.y
        #print self.turtlebot_orientation.z
        #print self.turtlebot_orientation.w
        #print "Yaw: ",
        #print self.yaw
        #print "Compass: ",
        #print self.compass
        #print "--------------------"
        pass

    def saveModelStates (self, model_states):
        # Model states comes in as a list of all entities in the enviroment
        m = model_states.name.index ("mobile_base")

        self.turtlebot_position = model_states.pose[m].position
        self.turtlebot_orientation = model_states.pose[m].orientation

        # Getting yaw (in x, y, z) rather than a quaternion
        self.euler_angles = tf.euler_from_quaternion([self.turtlebot_orientation.w, self.turtlebot_orientation.x, self.turtlebot_orientation.y, self.turtlebot_orientation.z])

        # Yaw is in range [0, 360)
        self.yaw = degrees ((pi - (self.euler_angles[0])))
        self.compass = (self.yaw - degrees (pi/4)) // degrees ((pi / 2))
 
        if self.compass == -1.0:
            self.compass = 3.0

        #self.printModelStates ()

    def getDirection (self):
        # Check if this still works
        return self.compass
