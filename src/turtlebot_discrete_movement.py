#!/usr/bin/env python

from discrete_rotation import TurtlebotDiscreteRotation
from discrete_linear import TurtlebotDiscreteLinear
import sys
import rospy
from time import sleep

rotate = TurtlebotDiscreteRotation ()
linear = TurtlebotDiscreteLinear ()
def getTargetPose ():
	return linear.getTargetPose ()

def getTurtlePose ():
	return linear.getTurtlePose ()

def forward ():
	print "Going Forward"
	linear.forward()
	while not linear.success:
		linear.success = 0
	sleep(5)

def turnRight ():
    print "Turning Right"
    rotate.rotate90Right ()
    while not rotate.rotation_success:
        rotate.checkRotation ()
    sleep(5)
	
def turnLeft ():
    print "Turning Left"
    rotate.rotate90Left ()
    while not rotate.rotation_success:
        rotate.checkRotation ()
    sleep(5)

def main ():
    while not rospy.is_shutdown ():
        #turnLeft ()
	forward ()
        turnLeft ()
	forward ()
        turnLeft ()

if __name__=="__main__":
    main ()
