import roslib
roslib.load_manifest('turtlebot-explore')
import rospy
import math
import tf

from geometry_msgs.msgs import Twist
from nav_msgs.msgs import MapMetaData, OccupancyGrid


class MazeExplorer(object):

    def __init__(self):
        self.running = True
        self._buffer_cells = 4
        self._occ_grid = None
        self._map_header = None
        self._explored = set()
        self.map_sub = rospy.Subscriber("/map",
                               OccuapncyGrid, self._update_grid, queue_size = 3)
        self.map_data_sub = rospy.Subscriber("/map_metadata",
                               MapMetaData, self._update_map_data, queue_size = 3)
         

    def _update_grid(self, data): 
        self._occ_grid = data

    def _update_map_data(self, data):
        self._map_header = data

    def _explore(self, callback):
        rospy.init_node('turtlebot_maze_explorer')
        listener = tf.TransformListener()
        rate = rospy.Rate(10.0)
        
        while self.running:
            #Get current position (I think /base_link is right)
            if self._occ_grid is None or self._map_header is None:
                continue

            try:
                (position, quaternion) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            next_postion = self._find_next_position(trans, rot)
            callback(next_postion)


    #topics required nav_msgs/MapMetaData nav_msgs/OccupancyGrid 
    def _find_next_position(self, current_position, current_quaternion):

        #Find position in map
        map_position = ((current_position[0] - self._map_header.origin[0]) / self._map_header.resolution,
                        (current_position[1] - self._map_header.origin[1]) / self._map_header.resolution)


        nearest_wall = self._find_nearest_wall(map_position)

        target_position = self._position_by_wall(neearest_wall)

        if target_position:
            self._explored.add(nearest_wall)
        else:
            return None

        #Find absolute position
        target_map_position = (target_position[0]*self._map_header.resolution,
                               target_position[1]*self._map_header.resolution)

        #TODO Rotate robot and send in correct direction
        # Adjust heading
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(current_quaternion)
        #Create Twist 


    #TODO make sure position isn't impeded by another wall
    def _position_by_wall(self, nearest_wall, robot_position):

        # x to left or right of robot
        if robot_postion[0] < nearest_wall[0]:
            x = nearest_wall[0] - self._buffer_cells 
        elif robot_position[0] > nearest_wall[0]:
            x = nearest_wall[0] + self.buffer_cells
        else:
            x = nearest_wall[0]
        
        # y to top or bottom of robot
        if robot_postion[1] < nearest_wall[1]:
            y = nearest_wall[1] - self._buffer_cells 
        elif robot_position[1] > nearest_wall[1]:
            y = nearest_wall[1] + self.buffer_cells
        else:
            y = nearest_wall[1]

        return (x,y)


    def _find_nearest_wall(self, map_position):

        max_distance = max(map_position[0], self._map_header.width - map_position[0]) + \
                       max(map_posiiton[1], self._map_header.height - map_position[1])

        for i in range(1, max_distance):
            for j in range(0, i + 1):
                x = map_position[0] - i + j        
                y = map_position[1] - j
                if self._cell_contains_wall(x,y):
                    return (x,y)

                x = map_position[0] + i - j        
                y = map_position[1] + j
                if self._cell_contains_wall(x,y):
                    return (x,y)

            for j in range(1, i):
                x = map_position[0] - j
                y = map_position[1] + i - j
                if self._cell_contains_wall(x,y):
                    return (x,y)

                x = map_position[0] + i - j
                y = map_position[1] - i
                if self._cell_contains_wall(x,y):
                    return (x,y)
        return None 

    def _cell_contains_wall(self, x, y):
        if x > self._map_header.width - 1 or x < 0 or \
           y > self.map_header.height - 1 or y < 0 or \
           (x,y) in self._explored:
               return False
        index = y*self._map_header.width + x
        if (self._occ_grid[index] >= 50):
            return True
        return False

    
    def stop(self):
        self.running = False

    def start(self, callback):
        self.running = True
        self._explore(callback)
