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
        self._spacing = 0.25
        self._angle_margin = np.deg2rad(20)
        self._speed = 2
        self._last_rotation = 0
        self._front_angle = np.deg2rad(10)
        self._right_turn = True


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

        """
        segments = self.segmentise()
        for segment in segments:
            if self.is_wall(segment):
                plt.scatter(*zip(*segment), c="red")
            else:
                plt.scatter(*zip(*segment), c="blue")
        plt.show()
        """
    
        # check for more busy side
        left = sum(x for i, x in enumerate(self.ranges) if self.angles[i] < 0)
        right = sum(x for i, x in enumerate(self.ranges) if self.angles[i] > 0)

        closest_point = self.closest_point()
        point_in_front = self.point_in_front()

        twist = Twist()
        speed = 0
        rotation = 0
        # if on right, nothing in front go forward
        if not closest_point:
            if self._right_turn:
                self._right_turn = False
                rotation = np.deg2rad(30)
            else:
                self._right_turn = True
                speed = self._speed

        elif closest_point and (not point_in_front or point_in_front[0] > self._spacing):
            closest_point_xy = to_xy(*closest_point)
            rotation = np.deg2rad(-90) + closest_point[1]
            speed = 0
            if np.fabs(rotation) < self._angle_margin:
                rotation = 0
                speed = self._speed
        # if some in front & on right, turn left
        elif point_in_front and point_in_front[0] <= self._spacing:
            rotation = np.deg2rad(90) + point_in_front[1]
             
        twist.linear.x = speed
        twist.angular.z = rotation
        self.pub.publish(twist)


        """
        if closest_point[1] > 0 or right > left:
            rotation = np.deg2rad(-90) + closest_point[1]
        else:
            rotation = np.deg2rad(90) + closest_point[1]


        if np.fabs(rotation) < self._angle_margin:
            rotation = 0

        twist = Twist()

        if rotation == 0:
            #drive
            twist.linear.x = self._speed
        else:
            #rotate
            twist.angular.z = rotation * 0.75

        rospy.loginfo(twist)
        if rotation * self._last_rotation >= 0:
            pass
            self.pub.publish(twist)
        else:
            twist.angular.z = self._last_rotation * 0.75
            for i in range (10): 
                #self.pub.publish(twist)
                rospy.sleep(0.1)
        self._last_rotation = rotation
        """
            

    def closest_point(self):
        try:
            index, distance = min(((i,x) for i, x in enumerate(self.ranges) if to_xy(x, self.angles[i])[0] > 0), key=lambda x: x[1])
            return (distance, self.angles[index])
        except ValueError:
            return False 

    def segmentise(self):
        d_thd = 0.05
        segments = []
        segment = []

        for i in range(len(self.ranges) - 1):
            segment.append(to_xy(self.ranges[i], self.angles[i]))
            if segment is [] or abs(self.ranges[i+1] - self.ranges[i]) < d_thd:
                pass
            else:
                segments.append(segment)
                segment= []

        if segment is not []: segments.append(segment)

        return segments

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
            return (distance, self.angles[index])
        except ValueError:
            return False 


    def _distance(self, x, y):
        return np.linalg.norm(np.array(x) - np.array(y))

        

def to_xy(r, a):
    return (r*np.cos(a), r*np.sin(a))

if __name__ == '__main__':
    try:
        node = MazeExplorer()
    except rospy.ROSInterruptException:
        pass
