from math import sqrt

class AStar():
    def __init__(self, graph):
        self.graph = graph
        
    def heuristic(self, node, start, end):
        return sqrt((end.x - node.x)**2 + (end.y - node.y)**2) # euclidean distance

    def search(self, start, end):
        closedset = set()
        openset =  set()
        openset.add(start)

        while openset:
            current = min(openset, key=lambda x:x.g + x.h)
            if current == end:
                return self.reconstruct_path(current.parent, end)

            openset.remove(current)
            closedset.add(current)
            for node in self.graph[current]:
                if node in closedset: 
                    continue
                if node in openset: 
                    tentative_g = current.g + current.cost(node)
                    if node.g > tentative_g:
                        node.g = tentative_g
                        node.parent = current
                else:
                    node.g = current.g + current.cost(node)
                    node.h = self.heuristic(node, start, end)
                    node.parent = current
                    openset.add(node)

        return None                                                     # no path found

    def reconstruct_path(self, parent, current):
        # traverse backwards along path to recreate it
        path = []
        while current.parent:
            path.append(current)
            current = current.parent
        path.append(current)        
        return path[::-1]                                               # reverse reversed path


class Node():
    def __init__(self, x, y):
        self.x, self.y = x, y
        self.g, self.h = 0, 0
        self.parent = None

    def cost(self, other):
        diagonal = abs(self.x - other.x) == 1 and abs(self.y - other.y) == 1
        return 15 if diagonal else 5

    def __repr__(self):
        return '(%d, %d)' % (self.x, self.y)





    
