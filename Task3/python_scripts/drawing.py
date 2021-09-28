#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import sys
import math
import time


def position_bot(pose_msg):
    # for rostopic /turtle1/Pose
    global x
    global y, yaw
    x = pose_msg.x
    y = pose_msg.y
    yaw = pose_msg.theta

def forward(line_vel, dist, vel_pub):

    #rospy.init_node('turtlemove', anonymous=True)
    #vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)

    vel = Twist()

    t = dist/line_vel
    t0 = rospy.Time.now().to_sec()

    while rospy.Time.now().to_sec()-t0 < t:

        rospy.loginfo(
            "moving forward with velocity=%f, distance=%f", line_vel, dist)
        vel.linear.x = line_vel
        vel.linear.y = 0
        vel.linear.z = 0

        vel.angular.x = 0
        vel.angular.y = 0
        vel.angular.z = 0

        vel_pub.publish(vel)
        rate.sleep()

    rospy.loginfo("Covered distance %f with vel %f", dist, line_vel)
    # rospy.loginfo(rospy.Time.now().to_sec())


def rotate(ang_vel, takeTime, vel_pub):

    #rospy.init_node('turtlemove', anonymous=True)
    #vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)

    ang_vel_deg = ang_vel
    ang_vel = math.radians(ang_vel)

    vel = Twist()

    t = takeTime
    t0 = rospy.Time.now().to_sec()
    # margin_sec = 0.1 #rospy is slow to react to changes, adding margin for error

    while (rospy.Time.now().to_sec()-t0 < t):
        vel.linear.x = 0
        vel.linear.y = 0
        vel.linear.z = 0

        vel.angular.x = 0
        vel.angular.y = 0
        vel.angular.z = ang_vel

        rospy.loginfo("Rotating by angle %f in %f secondds",
                      ang_vel_deg, takeTime)
        # rospy.loginfo(rospy.Time.now().to_sec())
        vel_pub.publish(vel)
        rate.sleep()
    rospy.loginfo("Rotated by angle %f in %f secondds", ang_vel_deg, takeTime)


def square(vel_pub, pose_sub):
    forward(3.0, 5.0, vel_pub)
    rotate(90, 1, vel_pub)
    forward(3.0, 5.0, vel_pub)
    rotate(90, 1, vel_pub)
    forward(3.0, 5.0, vel_pub)
    rotate(90, 1, vel_pub)
    forward(3.0, 5.0, vel_pub)
    rotate(90, 1, vel_pub)


def hexagon(vel_pub, pose_sub):
    forward(5.0, 3.0, vel_pub)
    rotate(60, 1, vel_pub)
    forward(5.0, 3.0, vel_pub)
    rotate(60, 1, vel_pub)
    forward(5.0, 3.0, vel_pub)
    rotate(60, 1, vel_pub)
    forward(5.0, 3.0, vel_pub)
    rotate(60, 1, vel_pub)
    forward(5.0, 3.0, vel_pub)
    rotate(60, 1, vel_pub)
    forward(5.0, 3.0, vel_pub)
    rotate(60, 1, vel_pub)


def circle_half(line_vel, dia_circle, vel_pub, pose_sub):

    forward(line_vel,dia_circle,vel_pub)
    rotate(90,1,vel_pub)

    rate = rospy.Rate(10)
    t0 = rospy.Time.now().to_sec()
    #t = (line_vel*2)/(math.pi*dia_circle)
    ang_vel= (line_vel*2)/dia_circle # v/r=omega
    t = ang_vel/math.radians(180)

    vel = Twist()

    while (rospy.Time.now().to_sec()-t0 < t+0.5):
        vel.linear.x = line_vel
        vel.linear.y = 0
        vel.linear.z = 0

        vel.angular.x = 0
        vel.angular.y = 0
        vel.angular.z = ang_vel

        vel_pub.publish(vel)
        rate.sleep()
        rospy.loginfo("drawing curve with vel=%f and angular vel=%f",line_vel,ang_vel)
    rospy.loginfo("created a D")

if __name__ == '__main__':
    try:
        rospy.init_node('turtlesim_motion_pose', anonymous=True)

        vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        pose_sub = rospy.Subscriber('/turtle1/pose', Pose, position_bot)
        time.sleep(2)

        circle_half(4.0,5.0, vel_pub, pose_sub)
        #hexagon(vel_pub,pose_sub)
        #square(vel_pub,pose_sub)
        #forward(velocity_publisher, 1.0, 9.0)
        #forward( 1.0, 4.0, vel_pub)

    except rospy.ROSInterruptException:
        pass
