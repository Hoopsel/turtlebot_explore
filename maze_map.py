#!/usr/bin/env python2
import roslib
roslib.load_manifest("turtlebot_explore")
import rospy
import numpy as np
import tf

from nav_msgs.msg import OccupancyGrid
from turtlebot_explore.msg import Beacon



class MazeMap(object):

    def __init__(self):
        self.map_sub = rospy.Subscriber('/slamGrid', OccupancyGrid, self._update_grid, queue_size = 1)
        self.beacon_sub = rospy.Subscriber('/turtlebot_beacon', Beacon, self._beacon_found, queue_size = 1) 
        self._occ_grid = None
        self.map_position = None
        self.beacons = {} #store as beacon colours : position -> [pink,blue] : (27,32)
        self.heading = None
    

    # listen for map updates
    def _update_grid(self, data):
        self._occ_grid = data

    def update(self, listener):
        try:
            (position, quaternion) = listener.lookupTransform('/slamGrid', '/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return

        # Find position in map
        self.map_position = ((current_position[0] - self._occ_grid.origin[0]) / self._occ_grid.resolution,
                            (current_position[1] - self._occ_grid.origin[1]) / self._occ_grid.resolution)
        
        
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quaternion)
        self.heading = yaw


    # listen for beacon found
    def _beacon_found(self, data):
          
        # figure out beacon position and target position for beacon 
        rotation = -self.heading 

        # rotate point so it is as if robot is facing north
        x_dist = (data.x * np.cos(rotation) - data.y * np.sin(rotation)) / self._occ_grid.resolution
        y_dist = (data.x * np.sin(rotation) + data.y * np.cos(rotation)) / self._occ_grid.resolution

        x = self.map_position[0] + x_dist
        y = self.map_position[1] + y_dist

        beacons[[data.top_colour, data.bot_colour]] = (x,y) 

         
def main():
    rospy.init_node('turtlebot_maze_map')
    maze_map = MazeMap()
    listener = tf.TransformListener()
    rospy.Rate(10.0)
    maze_map.update(listener)
    rospy.spin()

if __name__ == '__main__':
    main()
