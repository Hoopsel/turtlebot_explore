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
#from turtlebot_explore.msg import Beacon

from scan_detector import ScanDetector

class PoleDetector:

    def __init__(self):
        self.image_sub = rospy.Subscriber("/camera/rgb/image_color",Image, self.callback, queue_size = 1)
        self.bridge = CvBridge()
        self.cv_image = None
        self.scanner = ScanDetector()
#        self.pub = rospy.Publisher('turtlebot_beacon', Beacon)

    def callback(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8") # image created here
        except CvBridgeError as e:
            print e 

        self.process_image()

    def process_image(self):
        yellowBox = self.find_bounding('y')
        blueBox = self.find_bounding('b')
        greenBox = self.find_bounding('g')
        pinkBox = self.find_bounding('p')

        im = self.cv_image        # show result
        found = False             # condition for if beacon is seen?
        for box in [yellowBox, blueBox, greenBox, pinkBox]:
            x, y, w, h = box
            if w < 100 and h < 100:
                cv2.rectangle(im, (x, y), (x + w, y + h), (0, 255, 0), 2)
                print 'box found!'
                found = True

        if found:
            cv2.imshow("show", im)
            cv2.waitKey()
            pole_pos = self.scanner.get_circle_pos()
            if pole_pos is not None:
                beacon_found(pole_pos, 'b', 'g') # replace with actual top, bot colour

    def set_image(self, im):
        self.cv_image = im

    def beacon_found(self, pos, top_colour, bot_colour):
#        msg = Beacon()
        msg.top_colour = top_colour
        msg.bot_colour = bot_colour
        msg.x = pos[0]
        msg.y = pos[1]
        self.pub.publish(msg)

    def find_bounding(self, colour):
        # set colour range based on colour specified
        if colour == 'y':
            colour_min, colour_max = np.array([0,200,200], np.uint8), np.array([225,255,255], np.uint8)
        elif colour == 'b':
            colour_min, colour_max = np.array([30,80,120], np.uint8), np.array([120,205,255], np.uint8)
        elif colour == 'g':
            colour_min, colour_max = np.array([50,85,55], np.uint8), np.array([120,240,180], np.uint8)
        elif colour == 'p':
            colour_min, colour_max = np.array([140,100,130], np.uint8), np.array([255,180,240], np.uint8)
        else:
            return None

        im = self.cv_image # < this may be none! must handle
        hsv_img = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
        frame_thd = cv2.inRange(hsv_img, colour_min, colour_max)
        ret, thd = cv2.threshold(frame_thd, 127, 255, 0)
        contours, hierarchy = cv2.findContours(thd, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # find largest contour in set
        areas = [cv2.contourArea(c) for c in contours]
        largest = np.argmax(areas)
        contour = contours[largest]

        bounding = cv2.boundingRect(contour)
        return bounding

def main(args):
    detector = PoleDetector()
    rospy.init_node('pole_detector', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "shutting down"

    cv2.destroyAllWindows()

def test():
    pd = PoleDetector()
    im = cv2.imread('/home/andrew/ros/indigo/src/turtlebot_explore/assets/image.jpg')
    #im = cv2.cvtColor(im, cv2.COLOR_RGB2BGR)
    pd.set_image(im)
    pd.process_image()


    cv2.imshow("show", im)
    cv2.waitKey()
    cv2.destroyAllWindows()


if __name__ == '__main__': test()
