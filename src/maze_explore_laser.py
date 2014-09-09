#!/usr/bin/env python2

import roslib
roslib.load_manifest("turtlebot_explore")
import rospy
import numpy as np
import matplotlib.pyplot as plt

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class MazeExplorer(object):
    def __init__(self):
        self.ranges = []
        self.angles = []
        self._spacing = 0.4
        self._min_spacing = 0.25
        self._angle_margin = np.deg2rad(20)
        self._speed = 0.3  
        self._omega = 0.3
        self._last_rotation = 0
        self._front_angle = np.deg2rad(10)


        rospy.init_node("maze_explorer", anonymous=True)
        self.sub = None
        self.pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=3)
        self.cmd_sub = rospy.Subscriber('turtle_cmd', String, self.wait_cmd)
        rospy.spin()

    def wait_cmd(self, data):
        if data.data == "explore":            # start subscriber
            self.sub = rospy.Subscriber("scan", LaserScan, self.scan_callback)
        elif data.data == "stop explore":     # stop exploring
            self.sub = None


    def scan_callback(self, data):
        ranges = data.ranges
        angle_min, angle_increment = data.angle_min, data.angle_increment
        self.ranges = [x for x in ranges if x > 0]
        self.angles = [angle_min + i * angle_increment for i, j in enumerate(ranges) if j > 0]
    
        closest_point = self.closest_point()
        point_in_front = self.point_in_front()

        twist = Twist()
        speed = 0
        rotation = 0

        # if nothing on the right
        if not closest_point:
            # Arc movement
            speed = 0.5
            rotation = np.deg2rad(-90)
        # if some in front & on right, turn left
        elif point_in_front and point_in_front[0] <= self._spacing:
            rotation = np.deg2rad(90) + point_in_front[1]
            #rospy.sleep(1.0)                 # wait for camera
        # if on right, nothing in front go forward
        else:
            # adjust angle if too close - turn slightly away from the wall 
            if closest_point[0] <= self._min_spacing:
                rotation = np.deg2rad(90) - closest_point[1] + self._angle_margin 
                speed = self._speed * 0.75
            # adjust angle if too far - turn slightly towards the wall 
            elif closest_point[0] >= self._spacing:
                rotation = np.deg2rad(-90) + closest_point[1] + self._angle_margin 
                speed = self._speed * 0.75
            # right distance from the wall
            else:
                speed = self._speed
                rotation = np.deg2rad(90) + closest_point[1]
            

            if np.fabs(rotation) < self._angle_margin:
                rotation = 0
                speed = self._speed
        
        if rotation < -self._omega: 
            rotation = -self._omega
        elif rotation > self._omega:
            rotation = self._omega

        twist.linear.x = speed
        twist.angular.z = rotation
        self.pub.publish(twist)

    # closest point on right
    def closest_point(self):
        try:
            index, distance = min(((i,x) for i, x in enumerate(self.ranges) if self.angles[i] < -self._angle_margin and self.angles[i] > np.deg2rad(-90)), key=lambda x: x[1])
            if distance < 0.85:
                return (distance, self.angles[index])
        except ValueError:
            pass
        return False 
        
    def point_in_front(self):
        try:
            index, distance = min(((i,x) for i, x in enumerate(self.ranges) if np.fabs(self.angles[i]) < self._angle_margin), key=lambda x: x[1])
            if distance < self._spacing:
                return (distance, self.angles[index])
        except ValueError:
            pass
        return False 


def xy_dist(x, y):
    return np.linalg.norm(np.array(x) - np.array(y))

def to_xy(r, a):
    return (r*np.cos(a), r*np.sin(a))

if __name__ == '__main__':
    try:
        node = MazeExplorer()
    except rospy.ROSInterruptException:
        pass
