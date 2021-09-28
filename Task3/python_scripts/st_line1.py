#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import sys

def move_turtle(line_vel):

    rospy.init_node('turtlemove',anonymous=True)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rate  = rospy.Rate(10)

    vel = Twist()

    while True:
        vel.linear.x = line_vel
        vel.linear.y = 0 
        vel.linear.y = 0

        vel.angular.x = 0
        vel.angular.y = 0
        vel.angular.z = 0

        pub.publish(vel)
        rate.sleep()

#let me take value from keyboard

if __name__ == '__main__':
    try:
        move_turtle(float(sys.argv[1]))
    except rospy.ROSInterruptException:
        pass
