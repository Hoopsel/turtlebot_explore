#!/usr/bin/env python

import roslib
roslib.load_manifest('turtlebot_explore')
import rospy

from geometry_msgs.msg import Twist

class TurtleTeleOp(object):

    def __init__(self, speed=2, omega=0.5):
        rospy.init_node('turtlebot_move')

        pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist)

        self.speed = speed
        self.omega = omega
        twist = Twist()
        twist.linear.x, twist.angular.z = speed, 0     # speed, rotation

    def move(distance):
         # move some distance? send a certain amount of messages?

        some_cofactor = 10
        num_msgs = some_cofactor * distance
        self.__publish(num_msgs)

    def stop():
        pub.publish(Twist())

    def turn(rotation):
        some_cofactor = 10
        num_msgs = some_cofactor * rotation

        self.twist.linear.x = 0
        self.twist.angular.z = omega
        self.__publish(num_msgs)

    def __publish(num_msgs):
        rospy.loginfo("moving now")
        for i in range(num_msgs):
            pub.publish(self.twist)
            rospy.sleep(0.1)

        rospy.loginfo("Stop")
        pub.publish(Twist())        # defaults zero




