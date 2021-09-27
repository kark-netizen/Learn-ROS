import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time
from std_srvs.srv import Empty

def poseCallback(pose_message):
    global x
    global y, yaw
    x = pose_message.x
    y = pose_message.y
    yaw = pose_message.theta


def move(velocity_publisher, vel, distance, direction):
    vel_msg = Twist()
    global x, y
    x0 = x
    y0 = y

    if (direction):
        vel_msg.linear.x = abs(vel)
    else:
        vel_msg.linear.x = -abs(vel)

    distance_moved = 0.0
    loop_rate = rospy.Rate(10)

    while True:
        rospy.loginfo("moving forward")
        velocity_publisher.publish(vel_msg)

        loop_rate.sleep()

        distance_moved = abs(math.sqrt(((x-x0) ** 2) + ((y-y0) ** 2)))
        print(distance_moved)
        print(x)
        if not (distance_moved < distance):
            rospy.loginfo("reached")
            break
    vel_msg.linear.x = 0
    velocity_publisher.publish(vel_msg)

if __name__ == '__main__':
    try:

        rospy.init_node('turtlesim_motion_pose', anonymous=True)

        # declare velocity publisher
        cmd_vel_topic = '/turtle1/cmd_vel'
        velocity_publisher = rospy.Publisher(
            cmd_vel_topic, Twist, queue_size=10)

        position_topic = "/turtle1/pose"
        pose_subscriber = rospy.Subscriber(position_topic, Pose, poseCallback)
        time.sleep(2)

        move(velocity_publisher, 1.0, 9.0, True)

    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")

