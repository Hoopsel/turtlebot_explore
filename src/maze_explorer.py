import roslib
roslib.load_manifest('turtlebot_explore')
import rospy
import math
import tf

from geometry_msgs.msg import Twist
from nav_msgs.msg import MapMetaData, OccupancyGrid


class MazeExplorer(object):

    def __init__(self):
        self._buffer_cells = 10 
        self._occ_grid = None
        self._explored = set()
        self._map_topic = '/slamGrid'
        self.map_sub = rospy.Subscriber(self._map_topic,
                               OccupancyGrid, self._update_grid, queue_size = 3)
        self.pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist) 
        self.speed = 2
         

    def _update_grid(self, data): 
        self._occ_grid = data

    def explore(self, listener):
        # Get current position (I think /base_link is right)
        if self._occ_grid is None:
            print ".."
            continue

        try:
            (position, quaternion) = listener.lookupTransform(self._map_topic, '/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        twist = self._find_next_heading(position, quaternion)
        self.pub.publish(twist)
         

    # topics required nav_msgs/MapMetaData nav_msgs/OccupancyGrid 
    def _find_next_heading(self, current_position, current_quaternion):

        # Find position in map
        map_position = ((current_position[0] - self._occ_grid.origin[0]) / self._occ_grid.resolution,
                        (current_position[1] - self._occ_grid.origin[1]) / self._occ_grid.resolution)


        nearest_wall = self._find_nearest_wall(map_position)

        target_position = self._position_by_wall(neearest_wall)

        if target_position:
            self._explored.add(nearest_wall)
        else:
            return None

        # Find absolute position
        target_map_position = (target_position[0]*self._occ_grd.resolution,
                               target_position[1]*self._occ_grd.resolution)

        # Adjust heading
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(current_quaternion)
        required_heading = math.atan2(target_map_position[1] - current_position[1], 
                                      target_map_position[0] - current_position[0])
        
        rotation = required_heading - yaw 
        if math.fabs(rotation) > math.pi:
            rotation = math.pi - rotation if rotation < 0 else rotation - math.pi
        
        # Create Twist 
        twist = Twist()
        twist.linear.x = self.speed 
        twist.angular.z = rotation
        return twist
        
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

        max_distance = max(map_position[0], self._occ_grd.width - map_position[0]) + \
                       max(map_posiiton[1], self._occ_grd.height - map_position[1])

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
        if x > self._occ_grd.width - 1 or x < 0 or \
           y > self._occ_grd.height - 1 or y < 0 or \
           (x,y) in self._explored or not self._cell_can_be_reached(x,y):
               return False
        index = y*self._occ_grd.width + x
        if (self._occ_grid.data[index] >= 50):
            return True
        return False

    #TODO make sure position isn't impeded by another wall
    def _cell_can_be_reached(self, x, y):
        return True
  

def listener(explorer):
    tflistener = tf.TransformListener()
    rate = rospy.Rate(10.0)
    explorer.explore(tflistener) 
    rospy.spin()


def main():
    rospy.init_node('turtlebot_maze_explorer')
    explorer = MazeExplorer()
    listener(explorer)


if __name__ == '__main__':
    main()
