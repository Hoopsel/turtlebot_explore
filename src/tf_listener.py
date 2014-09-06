#!/usr/bin/env python2
import roslib
roslib.load_manifest('turtlebot_explore')
import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('tf_turtle')

    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)
    (trans, rot) = (0, 0)
    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform('/slamGrid', '/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print "no transform"
            continue

        print (trans, rot)           
        rate.sleep()
