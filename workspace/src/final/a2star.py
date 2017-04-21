
# a star algorithm file

import math
from Queue import PriorityQueue
from geometry_msgs.msg import Point

#start with a node definition

class Node:
    
    def __init__(self, x, y, g, parent):
        self.x = x
        self.y =y
        self.g = g
        self.h = 0
        self.parent = parent
        self.obstacleProb = 50

    @staticmethod
    def makeNode(x, y, g = 0, parent = None):
        return Node(x, y, g, parent)

    def __eq__(self, other):
        if isinstance(other, self.__class__):
            return abs(other.x - self.x) < 0.00001 and abs(other.y - self.y) < 0.00001
        else:
            return False

    def __str__(self):
        return "(" + str(self.x) + "," + str(self.y) + ") " + str(self.g) + " h = "+ str(self.h) # + " " + str(self.parent) + ";"

    def __hash__(self):
        return 137*self.x + 149 * self.y


class Graph:

    def __init__(self, width, height, mapData):
        self.width = width
        self.height = height
        self.graph = {}
        self.map = mapData
        self.cellStates = mapData.data
        # self.constructGraph(width, height)

    def constructGraph(self, width, height):
        neighborsOf = {}
        for x in range(0, width):
            for y in range(0, height):
                newNode = Node.makeNode(x,y)
                neighborsOf[newNode] = self.getNeighbors(newNode)

        return neighborsOf

    def distance(self, nodeA, nodeB):
        return abs(math.sqrt(pow(nodeA.x - nodeB.x, 2) + pow(nodeA.y - nodeB.y,2)))

    def getNeighbors(self, aNode):
        if aNode in self.graph:
            # print aNode, "is in the graph"
            return self.graph[aNode]

        # print aNode, "not in graph"
        neighbors = []
        for dx in [-1,0,1]:
            x = aNode.x + dx
            for dy in [-1,0,1]:
                y = aNode.y + dy

                if not (x == aNode.x and y == aNode.y) and self.isValidLocation(x, y):
                        neighbors.append(Node.makeNode(x,y, g = aNode.g +1))
        self.graph[aNode] = neighbors
        return self.graph[aNode]

    def isValidLocation(self, x, y):

        yes = self.isInGrid(x, y) and not self.isObstacle(x, y)
        # print "is valid location? ", (x,y), yes
        return yes

    def isInGrid(self,x, y):
        return x in range(0,self.width) and y in range(0,self.height)
    # is a hack at the moment, can be fixed later on

    def isObstacle(self, x, y):
        # builds borders for the "grid"
        listIndex = y * self.width + x
        yes = self.cellStates[listIndex] > 50
        # print "Is obstacle?", (x,y), " ",yes
        return yes

    def cellValue(self, node):
        listIndex = node.y * self.width + node.x
        return self.cellStates[listIndex]

    def convertNodeToPoint(self, node):
        resolution = self.map.info.resolution
        offsetX = self.map.info.origin.position.x
        offsetY = self.map.info.origin.position.y
        xAdj = yAdj = 0.5
        point = Point()
        point.x = (node.x * resolution) + (xAdj * resolution) + offsetX
        point.y = (node.y * resolution) + (yAdj * resolution) + offsetY
        return point
 
 # takes in two nodes as defined above and returns the 
def aStar(start, goal, graph, openSet = PriorityQueue(), costSoFar = {}, publishAll = None):
    # global waypoints
    path = []
    openSet.put((0,start))
    costSoFar[start] = 0
    # for key in graph.graph:
    #   print key, ": ", str(graph.graph[key])
    print "starting a-star"

    while not openSet.empty():
        currentNode = openSet.get()[1]
        # print "selected ", currentNode
        
        if currentNode == goal:
            print "Reached ", goal, "from", start
            path = printPath(currentNode, path, start)
            return path
            break

        neighbors = graph.getNeighbors(currentNode)

        for n in neighbors:
            newCost = currentNode.g + graph.distance(n, currentNode)
            # print currentNode, " + ", graph.distance(n, currentNode) 
            if n not in costSoFar or newCost < costSoFar[n]:
                costSoFar[n] = newCost
                n.g = newCost
                n.parent = currentNode
                n.h = heurisitic(n, goal)
                openSet.put(((n.g + n.h), n))

        # publishSet = []
        # for tup in openSet.queue:
        #   publishSet.append(tup[1])

        # publishAll(publishSet, costSoFar)
        
    print "DONE!"
    return path


def heurisitic(node, goal):
    hValue = math.sqrt(pow(node.x - goal.x, 2) + pow(node.y - goal.y, 2))
    # print "H from", node, "to", goal, " = ", hValue
    return hValue

def printPath(node, path, start):
    path.append(node)
    if node.parent is not None and node != start:
        printPath(node.parent, path, start)
    return path


##########################################
def totalCostSoFar(costList):
    totalCost = 0
    for n in costList:
        totalCost+= costList[n]
    return totalCost
##########################################


if __name__ == '__main__':
    start = Node(0,0,0, None)
    goal = Node(9,9, 0, None)
    graph = Graph(10,10)
    path = aStar(start, goal, graph)
    
    

