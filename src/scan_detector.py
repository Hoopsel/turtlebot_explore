import roslib
roslib.load_manifest("turtlebot_explore")
import rospy
import numpy as np

from sensor_msgs.msg import LaserScan

class ScanDetector(object):
    def __init__(self, is_sim=True):
        self.ranges = []
        self.angles = []
        self.angle_min, self.angle_max, self.angle_increment = 0, 0, 0

        if not is_sim:
            rospy.init_node("scan_detector", anonymous=True)
            self.sub = rospy.Subscriber("scan", LaserScan, self.scan_callback)
            rospy.spin()   # ??? do we want to run nodes separately so they run all the time?

    def set_values(self, ranges, angle_min, angle_max, angle_increment):
        self.ranges = [x for x in ranges if x > 0]
        self.angles = [angle_min + i * angle_increment for i, j in enumerate(ranges) if j > 0]
        self.angle_min, self.angle_max, self.angle_increment = angle_min, angle_max, angle_increment

    def scan_callback(self, data):
        ranges = data.ranges
        angle_min, angle_max, angle_increment = data.angle_min, data.angle_max, data.angle_increment
        self.ranges = [x for x in ranges if x > 0]
        self.angles = [angle_min + i * angle_increment for i, j in enumerate(ranges) if j > 0]

        segments = self.segmentise()
        print "num segments: %d" % len(segments)
        for segment in segments:
            circle_centre = self.find_circle(segment)
            if circle_centre is not None:
                print "circle found at (%f, %f)" % circle_centre


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

    def find_circle(self, segment):
        if len(segment) < 10: return None    # ignore small segments, calculations result in false positive
        xy = [(r*np.cos(a), r*np.sin(a)) for r, a in segment]

        x1, y1 = xy[0]
        x2, y2 = xy[-1]
        xm, ym = xy[len(xy)/2]
        centre = ((x1 + x2) / 2, (y1 + y2) / 2)
        radius = self._distance((x1, y1), (x2, y2)) / 2

        if abs(self._distance((xm, ym), centre) - radius) < 0.03: return centre
        else: return None

    def _distance(self, x, y):
        return np.linalg.norm(np.array(x) - np.array(y))


if __name__ == '__main__':
    try:
        node = ScanDetector(is_sim=False)
    except rospy.ROSInterruptException:
        pass
