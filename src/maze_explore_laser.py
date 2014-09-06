#!/usr/bin/env python2

import roslib
roslib.load_manifest("turtlebot_explore")
import rospy
import numpy as np
import matplotlib.pyplot as plt

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class MazeExplorer(object):
    def __init__(self):
        self.ranges = []
        self.angles = []
        self.angle_min, self.angle_max, self.angle_increment = 0, 0, 0
        self._spacing = 0.4
        self._min_spacing = 0.2
        self._angle_margin = np.deg2rad(20)
        self._speed = 2
        self._last_rotation = 0
        self._front_angle = np.deg2rad(10)


        rospy.init_node("maze_explorer", anonymous=True)
        self.sub = rospy.Subscriber("scan", LaserScan, self.scan_callback)
        self.pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist)
        rospy.spin()

    def scan_callback(self, data):
        ranges = data.ranges
        angle_min, angle_max, angle_increment = data.angle_min, data.angle_max, data.angle_increment
        self.ranges = [x for x in ranges if x > 0]
        self.angles = [angle_min + i * angle_increment for i, j in enumerate(ranges) if j > 0]
        self.xy = [to_xy(r, a + np.pi/2) for r, a in zip(self.ranges, self.angles)]
    
        closest_point = self.closest_point()
        point_in_front = self.point_in_front()

        twist = Twist()
        speed = 0
        rotation = 0

        # if nothing on the right
        if not closest_point:
            print "nothing on right"
            # Arc movement
            speed = 0.5
            rotation = np.deg2rad(-90)
        # if some in front & on right, turn left
        elif point_in_front and point_in_front[0] <= self._spacing:
            rotation = np.deg2rad(90) + point_in_front[1]
            print "something in front"
        # if on right, nothing in front go forward
        else:
            print "nothing in front"
            # adjust angle if too close - turn slightly away from the wall 
            if closest_point[0] <= self._min_spacing:
                rotation = np.deg2rad(90) - closest_point[1] + 2*self._angle_margin 
                speed = self._speed
            # adjust angle if too far - turn slightly towards the wall 
            elif closest_point[0] >= self._spacing:
                rotation = np.deg2rad(-90) + closest_point[1] + 2*self._angle_margin 
                speed = self._speed
            # right distance from the wall
            else:
                speed = self._speed
                rotation = np.deg2rad(90) + closest_point[1]

            if np.fabs(rotation) < self._angle_margin:
                rotation = 0
                speed = self._speed
            else: 
                rotation = rotation * 0.6

            """
            if closest_point[0] < self._spacing:
                rotation = np.deg2rad(-90) - closest_point[1]
            else:
                rotation = np.deg2rad(90) + closest_point[1]
            """
             
        twist.linear.x = speed
        twist.angular.z = rotation
        self.pub.publish(twist)

    # closest point on right
    def closest_point(self):
        try:
            index, distance = min(((i,x) for i, x in enumerate(self.ranges) if self.angles[i] < -self._angle_margin and self.angles[i] > np.deg2rad(-90)), key=lambda x: x[1])
            if distance < self._spacing + self._min_spacing:
                return (distance, self.angles[index])
        except ValueError:
            pass
        return False 
        
    def is_wall(self, segment):
        if len(segment) < 10: return False
        some_thd = 0.05
        xa, ya = segment[0]
        xb, yb = segment[len(segment)/2]
        xc, yc = segment[-1]
        return (xa*(yb - yc) + xb*(yc - ya) + xc*(ya- yb)) < some_thd

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
