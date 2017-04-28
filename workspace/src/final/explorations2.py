#!/usr/bin/env python

# a module for running a ROS node that locates nearest unknown cells
# on the global expanded map

# @author Everett Harding
# @author Andrey Yuvzik
import rospy, tf, copy, math
import numpy as np
from move_base_msgs.msg import MoveBaseActionGoal
from nav_msgs.msg import Odometry, OccupancyGrid
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
from geometry_msgs.msg import Twist, Pose, PoseStamped, PoseWithCovarianceStamped, Quaternion, Pose2D
from tf.transformations import euler_from_quaternion
from std_msgs.msg import String
from nav_msgs.msg import GridCells
from Queue import *
from math import *
from a2star import Node, Graph
from usefulFunctions import *


# keeps the global up to date with the most current published map
def updateMap(occGrid):
    print "Read globalMap"
    global globalMap
    globalMap = occGrid

# listens to the status messsage of the robot, checking for the 
# phrase "Goal reached". when the phrase is detected, the node triggers the 
# frontierFinder method
def statusListener(status_msg):
    global atGoal
    print "Checking msg:\n", status_msg.status_list
    print "At goal?",atGoal
    lst = status_msg.status_list[0]
    yay = "Goal reached"
    alsoyay = "Failed to find a valid plan. Even after executing recovery behaviors"
    if yay in lst.text or alsoyay in lst.text and not atGoal:
        atGoal = True

# runs the actual bfs to find frontiers
# can add functions to find entire frontier and pick point in the middle of it
def frontierFinder():    
    global atGoal
    print "Running frontier search"

    # gather intial pose of the robot and find the 
    initPose = getPoseWithCovarianceStampedFromOdom(odom_list)
    (initX, initY) = convertToGridCoords(initPose.pose.pose.position.x, initPose.pose.pose.position.y, globalMap)
    
    # set-up for bfs search
    start = Node.makeNode(initX, initY, 0, None) 
    graph = Graph(globalMap.info.width, globalMap.info.height, globalMap)  
    
    # find the cells on the frontier from BFS search
    frontierCells = breadthFirst(start, graph, 50)
    frontierCoords = map(lambda x:(x.x,x.y), frontierCells)

    # if there is a frontier to be found, publish the next cell as a goal
    if len(frontierCells) !=0:
        atGoal = False
        goalCell = frontierCells[0]
        newGoal = buildPoseStamped(goalCell, globalMap)
        print "New Goal", newGoal
        bfsNode_pub.publish(newGoal)

# returns all the cells on the frontier without consideration for their distance from the robot
def getUnknownWithNeighbors(graph):
    width = globalMap.info.width
    height = globalMap.info.height
    mappedgridData = map(lambda x: (Node(index(x)%width, index(x)/width, 0, None), index(x)), globalMap.data)
    unknowns = filter(lambda x: graph.cellValue(x[0]) == -1, mappedgridData)

    frontier = []
    for tup in unknowns:
        for node in graph.getNeighbors(tup[0]):
            if graph.cellValue(node) == 0:
                frontier.append(tup[0])

    for node in frontier:
        print node

# runs a depth-limited breadth first traversal of the graph, terminating at the edge of known space
def breadthFirst(start, graph, depthLimit):
    openSet = Queue()
    found = []
    frontier = []
    openSet.put(start)

    resolution = globalMap.info.resolution

    cells = GridCells()
    cells.header.frame_id = 'map'
    cells.cell_width = resolution 
    cells.cell_height = resolution
    
    while not openSet.empty() and not rospy.is_shutdown():
        current = openSet.queue[0]
        print "Examining: ", current
        print "Cell Value", graph.cellValue(current)

        # found an unknown cell
        if graph.cellValue(current) == -1:
            print "Found -1 at:\n", current
            found.append(current)
            publishCells(frontier, opB_pub, globalMap)
            break
        else:
            for n in graph.getNeighbors(current):
                if graph.cellValue(n) == -1 and current not in frontier and graph.cellValue(current) < 50:
                    frontier.append(current)
                # n.g - how far away fro start the cell is
                if n not in found and n not in openSet.queue and n.g < depthLimit:
                    print "n", n
                    openSet.put(n)
        found.append(openSet.get())
    # sub_status = rospy.Subscriber('/move_base/status', GoalStatusArray, frontierFinder, queue_size = 10)
    return frontier

# set up globals and initialize node, subscribers, publishers
def main():
    rospy.init_node('exploration')

    global opB_pub
    global bfsNode_pub
    global sub_status
    global globalMap
    global odom_list
    global atGoal
    atGoal = True
    sub_status = rospy.Subscriber('/move_base/status', GoalStatusArray, statusListener, queue_size = 10)
    # sub_gMap = rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, updateMap, queue_size = 10)
    sub_gMap = rospy.Subscriber('/exp_map', OccupancyGrid, updateMap, queue_size = 10)

    # fix this topic
    opB_pub = rospy.Publisher('/frontierCells', GridCells, None, queue_size = 10) 
    bfsNode_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, None, queue_size = 10)

    odom_list = tf.TransformListener()

    while not rospy.is_shutdown():
        if atGoal:
            frontierFinder()
        


if __name__ == '__main__':
    main()