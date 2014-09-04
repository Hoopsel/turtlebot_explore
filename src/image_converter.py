#!/usr/bin/env python2

import roslib
import sys
import rospy

import cv2

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ImageConverter:

    def __init__(self):
        self.image_sub = rospy.Subscriber("/camera/rgb/image_color",
                               Image, self.callback, queue_size = 1)
        self.bridge = CvBridge()
        
    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print e

        print "saving image"
        cv2.imwrite("image.jpg", cv_image)
        print "image saved"



def main(args):
    ic = ImageConverter()
    rospy.init_node('image_converter', anonymous=True)
    try: 
        rospy.spin()
    except KeyboardInterrupt:
        print "shutting down"
    cv2.destroyAllWindows()

if __name__ == '__main__': main(sys.argv)
