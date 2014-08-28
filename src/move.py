#!/usr/bin/env python

import roslib
import rospy

from geometry_msgs.msg import Twist

speed = 2

if __name__ == "__main__":
    rospy.init_node('turtlebot_move')

    pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist)

    twist = Twist()
    twist.linear.x, twist.angular.z = speed, 0     # speed, rotation
    #twist.linear.y, twist.linear.z = 0, 0          # not used
    #twist.angular.x, twist.angular.y = 0, 0        # not used

    rospy.loginfo("moving now")
    for i in range(30):
        pub.publish(twist)
        rospy.sleep(0.1)

    twist = Twist()  # defaults zero

    rospy.loginfo("Stop")
    pub.publish(twist)


