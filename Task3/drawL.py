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

def forward(velocity_publisher, line_vel, dist):

    global x, y
    x0 = x
    y0 = y

    rate  = rospy.Rate(10)

    vel = Twist()

    dist_cov = 0.0

    while True:

        rospy.loginfo("forward")
        vel.linear.x = line_vel
        vel.linear.y = 0 
        vel.linear.z = 0

        vel.angular.x = 0
        vel.angular.y = 0
        vel.angular.z = 0

        dist_cov = abs(math.sqrt(((x-x0) ** 2) + ((y-y0) ** 2)))

        if not dist_cov < dist:
            rospy.loginfo("stop")
            break

        velocity_publisher.publish(vel)
        rate.sleep()

def rotate(velocity_publisher, angular_speed_degree, relative_angle_degree):

    velocity_message = Twist()

    angular_speed = math.radians(abs(angular_speed_degree))

    angle_moved = 0.0

    loop_rate = rospy.Rate(10)
    cmd_vel_topic = '/turtle1/cmd_vel'
    velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

    t0 = rospy.Time.now().to_sec()
    current_angle_degree = 0
    timeTaken = relative_angle_degree/angular_speed_degree
    while (rospy.Time.now().to_sec() - t0 < timeTaken):
        rospy.loginfo("Rotating")
        velocity_publisher.publish(velocity_message)

        loop_rate.sleep()
        print(current_angle_degree - relative_angle_degree)
    rospy.loginfo("reached")

    velocity_message.angular.z = 0
    velocity_publisher.publish(velocity_message)



if __name__ == '__main__':
    try:
        rospy.init_node('turtlesim_motion_pose', anonymous=True)

        velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        
        pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, position_bot)
        time.sleep(2)

        forward(velocity_publisher, 1.0, 9.0)
        rotate(velocity_publisher, 45, 90)
        forward(velocity_publisher, 1.0, 9.0)

    except rospy.ROSInterruptException:
        pass
