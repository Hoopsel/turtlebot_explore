import astar
from itertools import product

def main():
    grid = [0, 0, 0, 0,
            0, 1, 1, 0,
            0, 1, 1, 0,
            0, 0, 0, 0]
    grid_height, grid_width = 4, 4

    nodes = [[astar.Node(x, y) for y in range(grid_height)] for x in range(grid_width)]
    graph = {}

    for x, y in product(range(grid_width), range(grid_height)):
        node = nodes[x][y]
        graph[node] = []
        for i, j in product([-1, 0, 1], [-1, 0, 1]):        # 1 step away in any direction
            if (x + i < 0 or x + i > grid_width - 1):  continue # check that step in bounds
            if (y + j < 0 or y + j > grid_height - 1): continue
            if grid[(x + i) + (y + j)*grid_width] > 0:             continue # obstacle
            graph[node].append(nodes[x+i][y+j])

    paths = astar.AStar(graph)
    start, end = nodes[0][0] , nodes[3][3]
    path = paths.search(start, end)

    if path is None:
        print "No path"
    else:
        print "path: ", path



if __name__ == '__main__': main()
