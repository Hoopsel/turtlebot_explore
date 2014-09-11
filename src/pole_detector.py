#!/usr/bin/env python2
import sys
import roslib
import rospy
roslib.load_manifest('turtlebot_explore')

import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import String
from sensor_msgs.msg import Image
from turtlebot_explore.msg import Beacon 


from scan_detector import ScanDetector

class PoleDetector:

    def __init__(self):
        self.image_sub = rospy.Subscriber("/camera/rgb/image_color",Image, self.callback, queue_size = 1)
        self.bridge = CvBridge()
        self.cv_image = None
        self.scanner = ScanDetector()
        self._bound_size = 60
        self._bound_margin = 30
        self._kernel = np.ones((5,5), np.uint8)
        self.pub = rospy.Publisher('turtlebot_beacon', Beacon, queue_size=1)
        self.process_sub = rospy.Subscriber("maze_explorer/process_image", String, self.process_image, queue_size = 1) 
        self.win_name = 'image'
        cv2.namedWindow(self.win_name)
        

    def callback(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8") # image created here
        except CvBridgeError as e:
            print e 
        self.process_image()

    def process_image(self):
        self.cv_image = cv2.morphologyEx(self.cv_image, cv2.MORPH_CLOSE, self._kernel)
        bounds = {}
        for c in ['yellow', 'green', 'blue', 'pink']:
            bounds[c] = self.find_bounding(c)

        im = self.cv_image        # show result
        heights = []
        for c, box in bounds.items():
            if box is None: continue
            x, y, w, h = box
            cv2.rectangle(im, (x, y), (x + w, y + h), (0, 255, 0), 2)
            heights.append((c, y))

        heights = sorted(heights, key=lambda x: x[1])

        cv2.imshow(self.win_name, im)
        cv2.waitKey(1)
        if len(heights) == 2 and np.fabs(heights[0][1] - heights[1][1]) < 2*self._bound_size:
            rospy.loginfo('beacon found %s on %s' % (heights[0][0], heights[-1][0]))
            #pole_pos = self.scanner.get_circle_pos()
            self.beacon_found((0, 0), heights[0][0], heights[-1][0]) # replace with actual top, bot colour

    def set_image(self, im):
        self.cv_image = im
        cv2.imshow(self.win_name, im)

    def beacon_found(self, pos, top_colour, bot_colour):
        msg = Beacon()
        msg.top_colour = top_colour
        msg.bot_colour = bot_colour
        msg.x = pos[0]
        msg.y = pos[1]
        self.pub.publish(msg)

    def find_bounding(self, colour):
        # set colour range based on colour specified
        if colour == 'yellow':
            colour_min, colour_max = np.array([0,160,160], np.uint8), np.array([180,255,255], np.uint8)
        elif colour == 'blue':
            colour_min, colour_max = np.array([80,115,140], np.uint8), np.array([120,190,240], np.uint8)
        elif colour == 'green':
            colour_min, colour_max = np.array([60,80,40], np.uint8), np.array([100,220,170], np.uint8)
        elif colour == 'pink':
            colour_min, colour_max = np.array([130,80,80], np.uint8), np.array([255,220,255], np.uint8)
        else:
            return None

        im = self.cv_image 
        im_h, im_w, im_d = im.shape
        hsv_img = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
        frame_thd = cv2.inRange(hsv_img, colour_min, colour_max)
        ret, thd = cv2.threshold(frame_thd, 127, 255, 0)
        cv2.imshow(self.win_name,thd)
        contours, hierarchy = cv2.findContours(thd, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        boundings = [cv2.boundingRect(c) for c in contours]
        for bounding in boundings:
            x, y, w, h = bounding
            if y > im_h / 2: continue                   # ignore lower half of image
            if self._in_bound(w) and self._in_bound(h):
                return bounding

        return None # no matches

    def _in_bound(self, x):
        return np.fabs(x - self._bound_size) < self._bound_margin

def main():
    detector = PoleDetector()
    rospy.init_node('pole_detector', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "shutting down"

    cv2.destroyAllWindows()

if __name__ == '__main__': main()
