#!/usr/bin/env python
 
from Queue import PriorityQueue
from math import sqrt

 
#node class to handle grid cells
# cost is 1 if space is free, 1000000 if space contains an obstacle
class Node():
 
    def __init__(self, parent, x, y, cost):
        self.parent = parent
        self.x = int(x)
        self.y = int(y)
        self.cost = cost
 
    def __str__(self):
        return "(" + str(self.x) + "," +  str(self.y) + ") cost: " + str(self.cost)
 
    def __eq__(self, other):
        if isinstance(other, self.__class__):
            return self.x == other.x and self.y == other.y
        else :
            return False
 
# run AStar takes in the starting node and the goal node
# it returns the optimal path between the two
def runAStar(startNode, goalNode, w, h):
    gridWidth = w
    gridHeight = h
    closedSet = []
    openSet = PriorityQueue()
    path = []
    aStar(startNode, goalNode, path, closedSet, openSet)
    return path
 
# aStar algorithm
def aStar(startNode, goalNode, path, closedSet, openSet):
    
    print "Adding ", startNode, "to openSet"
    openSet.put((totalCost(startNode, goalNode),startNode))
    while not openSet.empty():
        # get the end of the current best path and add it to the closed set
        currentNode = openSet.get()[1]
        closedSet.append(currentNode)
 
        print "Expanding ", currentNode
        # check if the current node is the goal
        if currentNode == goalNode:
            print "Reached Goal at", currentNode
            pathTrace(currentNode, path)
            break
 
        # build a set of the neighbors of the current grid cell
        children = getNeighbors(currentNode)
 
        # for each of the next nodes, either put it into the open set
        # using its cost as a priority or ignore it if it is in the closed set
        for child in children:
            if child not in openSet and child not in closedSet:               
                print "Adding ", child, "to openSet"
                openSet.put((totalCost(child, goalNode), child))
            elif child.cost < 
            else if child:
                print "Ignoring ", child, " (it is already in the closedSet)"
 
# can add "turning cost"
def totalCost(node, goalNode):
    return node.cost + heuristicValue(node, goalNode)
 
# this functions returns the heuristic value of the node
# at the moment it returns zero, which makes the algorithm
# behave like greedy search
def heuristicValue(node, goalNode):
    return sqrt((goalNode.x - node.x) ** 2 + (goalNode.y - node.y) ** 2)
 
# pathtrace shows the path found by the algorithm
def pathTrace(node, path):
    if node.parent is not None:
        path += pathTrace(node.parent, path)
    print node
    return path
   
 
# returns the neighbors of this node
# will need to add check for free/obstacle
def getNeighbors(node):
    neighbors = []
    for dx in [-1,0, 1]:
        for dy in [-1,0,1]:
            if node.x == 0 and dx == -1:
                pass
            if node.x == gridWidth and dx == 1:
                pass
            if node.y == 0 and dy ==-1:
                pass
            if node.y == gridHeight and dy == 1:
                pass
            
            if not (dx == 0 and dy == 0):
                if mapData[(node.y+dy) * gridWidth + (node.x+dx)] < 50:
                    neighbors.append(Node(node, node.x + dx, node.y +dy, node.cost+1))
    return neighbors
 
# if __name__ == '__main__':
#     node = Node(None, 2,2,1)
#     end = Node(None, 10, 7, 1)
#     runAStar(node, end)