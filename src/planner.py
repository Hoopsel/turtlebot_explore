#!/usr/bin/env python
import rospy
from std_msgs.msg import String

class Planner(object):
    def __init__(self):
        cmd_pub = rospy.Publisher('/turtle_cmd', String, queue_size=1)
        rospy.init_node('planner', anonymous=True)
        
    def start_explore(self):
        self.cmd_pub.publish("explore") # some duplication here, not sure py version of headers are

    def stop_explore(self):
        self.cmd_pub.publish("stop explore") 

if __name__ == '__main__':
    # example to start the exploring phase. the other node will listen and subscribe/unsubscribe to /scan.
    planner = Planner()
    planner.start_explore()
