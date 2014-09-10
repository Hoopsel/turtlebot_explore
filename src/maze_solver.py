#!/usr/bin/env python2
import roslib
roslib.load_manifest("turtlebot_explore")
import rospy
import numpy as np
import tf

from nav_msgs.msg import OccupancyGrid
from turtlebot_explore.msg import Beacon
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from grid_search import grid_search



class MazeSolver(object):

    def __init__(self):
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self._update_grid, queue_size = 1)
        self.beacon_sub = rospy.Subscriber('/turtlebot_beacon', Beacon, self._beacon_found, queue_size = 1) 
        self.pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=1)
        self.explore_pub = rospy.Publisher('turtle_cmd', String, queue_size=1)
        self._occ_grid = None
        self.map_position = None
        self.beacons = {} #store as beacon colours : position -> (pink,blue) : (27,32)
        self.goals = []
        self.heading = None
        self._angle_margin = np.deg2rad(10) 
        self._speed = 2
        self._unpack_goals()
        self._finished = False
    
    def _unpack_goals(self):
        beacons = rospy.get_param('beacons')
        for i in range(len(beacons)):
            key = "beacon%d" % i
            beacon = beacons[key]
            self.goals.append((beacon["top"], beacon["bottom"]))
        rospy.loginfo(self.goals)

    # listen for map updates
    def _update_grid(self, data):
        self._occ_grid = data

    def update_position(self, listener):
        while not self._finished:
            try:
                #now = rospy.Time.now()
                #listener.waitForTransform("/odom_combined", "/base_footprint", now, rospy.Duration(1.0))
                # could be /slam/slam_frame /icp/odom_frame /odom and second could be /base_footprint or /base_link
                (position, quaternion) = listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logerr("Transformation error")
                continue

            # Find position in map
            self.map_position = ((current_position[0] - self._occ_grid.origin[0]) / self._occ_grid.resolution,
                                (current_position[1] - self._occ_grid.origin[1]) / self._occ_grid.resolution)
            
            rospy.logdebug("Map position %s and heading %s", str(position), str(quaternion))
            
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quaternion)
            self.heading = yaw


    # listen for beacon found
    def _beacon_found(self, data):

        if self.map_position is None: 
            rospy.logdebug("Beacon found, no current position")
            return

        # figure out beacon position and target position for beacon 
        # rotation = -self.heading 

        # rotate point so it is as if robot is facing north
        # x_dist = (data.x * np.cos(rotation) - data.y * np.sin(rotation)) / self._occ_grid.resolution
        # y_dist = (data.x * np.sin(rotation) + data.y * np.cos(rotation)) / self._occ_grid.resolution

        # x = self.map_position[0] + x_dist
        # y = self.map_position[1] + y_dist

        x = self.map_position[0]
        y = self.map_position[1]

        self.beacons[(data.top_colour, data.bot_colour)] = (x,y) 

        rospy.loginfo("%s on %s beacon found at (%d, %d)", data.top_colour, data.bot_colour)


        # check if last beacon found
        if not set(self.goals).issubset(self.beacons.keys()):
            return
        
        # stop explore
        self.stop_explore()
        # solve
        self.solve_maze()
    
    
    def start_explore(self):
        for i in range(3):
            self.explore_pub.publish('explore') # some duplication here, not sure py version of headers are
            rospy.sleep(1.0)

    def stop_explore(self):
        for i in range(3):
            self.explore_pub.publish("stop explore") 
            rospy.sleep(1.0)

    def solve_maze():

        for goal in self.goals:
            position = self.beacons[goal]
            path = grid_search(self._occ_grid.data, self._occ_grid.width, self._occ_grid.height, self.map_position, position)
            for coord in path:
                move(*coord)
            rospy.loginfo("Reached goal %s", str(goal))
        rospy.loginfo("Solved maze")
        self._finished = True


    def move(to_x, to_y):

        while np.fabs(self.map_position[0] - to_x) >= 1 or np.fabs(self.map_position[y] - to_y) >= 1:
            partial_move = self._partial_move(to_x, to_y)
            self.pub.publish(partial_move)


    def _partial_move(to_x, to_y):
        required_heading = np.atan2(to_y - self.map_position[1], 
                                    to_x - self.map_position[0])
        rotation = required_heading - self.heading

        if np.fabs(rotation) < self._angle_margin:
            rotation = 0

        speed = 0 if rotation > 0 else self._speed 

        twist = Twist()
        twist.linear.x = speed 
        twist.angular.z = rotation
        return twist

         
def main():
    rospy.init_node('turtlebot_maze_solver', log_level=rospy.DEBUG)
    maze_solver = MazeSolver()
    maze_solver.start_explore()
    listener = tf.TransformListener()
    rospy.Rate(10.0)
    maze_solver.update_position(listener)

if __name__ == '__main__':
    main()
