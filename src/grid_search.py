import astar
from math import sqrt
from itertools import product

class AStarGrid(astar.AStar):
    def __init__(self, graph):
        astar.AStar.__init__(self, graph)

    def heuristic(self, node, start, end):
        return sqrt((end.x - node.x)**2 + (end.y - node.y)**2) # euclidean distance

class GridNode(astar.Node):
    def __init__(self, x, y):
        astar.Node.__init__(self)
        self.x, self.y = x, y
        
    def cost(self, other):
        return 1

    def __repr__(self):
        return '(%d, %d)' % (self.x, self.y)

def grid_search(grid, height, width, start, end):
    nodes = [[GridNode(x, y) for y in range(grid_height)] for x in range(grid_width)]
    graph = {}

    # create graph
    for x, y in product(range(grid_width), range(grid_height)):
        node = nodes[x][y]
        graph[node] = []
        for i, j in product([-1, 0, 1], [-1, 0, 1]):            # 1 step away in any direction
            if (i != 0 and j != 0):                    continue # ignore diagonals
            if (x + i < 0 or x + i > grid_width - 1):  continue # check that step in bounds
            if (y + j < 0 or y + j > grid_height - 1): continue
            if grid[(x + i) + (y + j)*grid_width] > 0: continue # obstacle
            graph[node].append(nodes[x+i][y+j])

    paths = AStarGrid(graph)
    start, end = nodes[start[0]][start[1]] , nodes[end[0]][end[1]]
    path = paths.search(start, end)

    if path is None:
        print "No path"
    else:
        print "path: ", path

    return path



if __name__ == '__main__': 
    grid = [0, 0, 0, 0,
            0, 1, 1, 0,
            0, 1, 1, 0,
            0, 0, 0, 0]
    grid_height, grid_width = 4, 4
    start, end = (0, 0), (3, 3)

    grid_search(grid, grid_height, grid_width, start, end)
