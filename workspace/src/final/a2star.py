#!/bin/usr/env python
# a star algorithm file
# @author Everett Harding

import math
from geometry_msgs.msg import Point

import rospy, tf, numpy, math, sys

from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose, Point
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import GridCells

from Queue import *
from math import *
from lab3_pathPlanning import publishCells

# Node Class:
# A node represents a point on the map represented as a grid. Nodes contain
# the (x,y) location of the node, 
# the cost (g) of reaching the node from a starting position,
# the heuristic value (h) of the cost of the node
# the parent of the node, representing the previous node in a path
# the probablility that the node represents a space occupied by an obstacle
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


# the Graph class is build from an OccupancyGrid message, 
# and is used to populate a graph of Node objects from the map
# data contained in the occupancy grid.
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

    # returns the real-valued distance between nodeA and nodeB
    def distance(self, nodeA, nodeB):
        return abs(math.sqrt(pow(nodeA.x - nodeB.x, 2) + pow(nodeA.y - nodeB.y,2)))

    # returns a list of nodes that are adjacent to the given node (aNode)
    def getNeighbors(self, aNode):
        if aNode in self.graph:
            return self.graph[aNode]

        neighbors = []
        # iterate through the eight nodes surrounding aNode
        for dx in [-1,0,1]:
            x = aNode.x + dx
            for dy in [-1,0,1]:
                y = aNode.y + dy

                if not (x == aNode.x and y == aNode.y) and self.isValidLocation(x, y):
                    gCost = aNode.g+1
                    if self.isObstacle(x,y):
                        gCost = 10000000000000000
                    neighbors.append(Node.makeNode(x,y, g = gCost))
        self.graph[aNode] = neighbors
        return self.graph[aNode]

    # return true if the given x and y values represent a valid location in the OccupancyGrid
    def isValidLocation(self, x, y):

        yes = self.isInGrid(x, y)# and not self.isObstacle(x, y) #comment out obstacle check to allow "obstacle nodes"
        # print "is valid location? ", (x,y), yes
        return yes

    # return true if the x and y values are within the bounds of the OccupancyGrid
    def isInGrid(self,x, y):
        return x in range(0,self.width) and y in range(0,self.height)

    # returns true if the x and y coordiantes represent a point with a high
    # probaility of being an obstacle
    def isObstacle(self, x, y):
        # builds borders for the "grid"
        listIndex = y * self.width + x
        yes = self.cellStates[listIndex] > 50
        return yes

    # returns the probability of the cell containing an obstacle
    def cellValue(self, node):
        listIndex = node.y * self.width + node.x
        return self.cellStates[listIndex]

    # converts the given node to its corrsponding point in the map frame
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
    path = []
    openSet.put((0,start))
    costSoFar[start] = 0
    print "starting a-star"

    while not openSet.empty():
        currentNode = openSet.get()[1]
       
        if currentNode == goal:
            print "Reached ", goal, "from", start
            path = printPath(currentNode, path, start)
            return path

        neighbors = graph.getNeighbors(currentNode)

        for n in neighbors:
            newCost = currentNode.g + graph.distance(n, currentNode)
            if n not in costSoFar or newCost < costSoFar[n]:
                costSoFar[n] = newCost
                n.g = newCost
                n.parent = currentNode
                n.h = heurisitic(n, goal)
                openSet.put(((n.g + n.h), n))
        
    print "No path could be found from", start, " to ", goal
    return path

# returns the heuristic value of the node
def heurisitic(node, goal):
    hValue = math.sqrt(pow(node.x - goal.x, 2) + pow(node.y - goal.y, 2))
    # print "H from", node, "to", goal, " = ", hValue
    return hValue

# recursively prints the path found by a* function
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
    global bfs_pub
    start = Node(0,0,0, None)
    goal = Node(9,9, 0, None)
    graph = Graph(10,10)
    path = aStar(start, goal, graph)
    # bfs_pub = rospy.Publisher('/bfs', OccupancyGrid, queue_size = 1)
    
    

