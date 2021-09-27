import rospy
from geometry_msgs.msg import Twist
import sys

def move_turtle (line_vel,ang_vel):

    rospy.init_node ('turtlemove',anonymous=True)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rate  = rospy.Rate(10)

    vel = Twist()

    while True:
        vel.linear.x = line_vel
        vel.linear.y = 0 
        vel.linear.y = 0

        vel.angular.x = 0
        vel.angular.y = 0
        vel.angular.z = ang_vel

        rospy.loginfo("Linear Value is ---> %f: Angular Value is ---> %f:", line_vel, ang_vel)

        pub.publish(vel) #publishing topic /turtle1/cmd_vel
        rate.sleep()     #sleep as long as desired for the Rate defined above

#let me take value from keyboard

if __name__ == '__main__':
    try:
        move_turtle(float(sys.argv[1]),float(sys.argv[2]))
    except rospy.ROSInterruptException:
        pass