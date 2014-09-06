#!/usr/bin/env python2

import roslib
roslib.load_manifest('turtlebot_explore')
import rospy

from geometry_msgs.msg import Twist

class TurtleTeleOp(object):

    def __init__(self, topic='cmd_vel_mux/input/navi', speed=2, rotation=0):
        rospy.init_node('turtlebot_move', anonymous=True)

        self.pub = rospy.Publisher(topic, Twist)

        self.twist = Twist()
        set_move(speed, rotation) 

    def set_move(speed, rotation):
        self.twist.linear.x, self.twist.angular.z = speed, rotation

    def move(self, speed):
        self.set_move(speed, 0)
        self.publish()

    def stop(self):
        self.publish(Twist())

    def turn(self, rotation):
        self.set_move(0, rotation)
        self.publish()

    def publish(self, twist=self.twist):
        self.pub.publish(twist)

if __name__ == '__main__':
    move = TurtleTeleOp()
    move.move(2)


