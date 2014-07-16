#!/usr/bin/env python

from discrete_rotation import TurtlebotDiscreteRotation
import sys
import rospy

turtlebot = TurtlebotDiscreteRotation ()

def turnRight ():
    print "Turning Right"
    turtlebot.rotate90Right ()
    while not turtlebot.rotation_success:
        turtlebot.checkRotation ()

def turnLeft ():
    print "Turning Left"
    turtlebot.rotate90Left ()
    while not turtlebot.rotation_success:
        turtlebot.checkRotation ()

def main ():
    while not rospy.is_shutdown ():
        turnRight ()
        turnRight ()
        turnLeft ()

if __name__=="__main__":
    main ()
