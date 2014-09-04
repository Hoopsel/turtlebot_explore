#!/usr/bin/env python2

import roslib
roslib.load_manifest("turtlebot_explore")
import rospy
import numpy as np

from sensor_msgs.msg import LaserScan

class MazeExplorer(object):
    def __init__(self):
        self.ranges = []
        self.angles = []
        self.angle_min, self.angle_max, self.angle_increment = 0, 0, 0
        self._spacing = 0.3

        rospy.init_node("maze_explorer", anonymous=True)
        self.sub = rospy.Subscriber("scan", LaserScan, self.scan_callback)
        rospy.spin()   # ??? do we want to run nodes separately so they run all the time?

    def scan_callback(self, data):
        ranges = data.ranges
        angle_min, angle_max, angle_increment = data.angle_min, data.angle_max, data.angle_increment
        self.ranges = [x for x in ranges if x > 0]
        self.angles = [angle_min + i * angle_increment for i, j in enumerate(ranges) if j > 0]
        self.xy = [to_xy(r, a) for r, a in zip(self.ranges, self.angles)]

        closest = self.closest_point()

        if closest[0] > 0:
            x = closest[0]  
        elif closest[0] < 0:
        
        else:
        



    def closest_point(self):
        index, range = min(enumerate(self.ranges), key=lambda x: x[1])
        return to_xy(range, self.angles[index])

    def segmentise(self):
        d_thd = 0.05
        segments = []
        segment = []

        for i in range(len(self.ranges) - 1):
            segment.append((self.ranges[i], self.angles[i]))
            if segment is [] or abs(self.ranges[i+1] - self.ranges[i]) < d_thd:
                pass
            else:
                segments.append(segment)
                segment= []

        if segment is not []: segments.append(segment)

        return segments

    def _distance(self, x, y):
        return np.linalg.norm(np.array(x) - np.array(y))

def to_xy(r, a):
    return (r*np.cos(a), r*np.sin(a)

if __name__ == '__main__':
    try:
        node = MazeExplorer()
    except rospy.ROSInterruptException:
        pass