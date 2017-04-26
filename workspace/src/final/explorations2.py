#!/usr/bin/env python
import rospy, tf, copy, math
from move_base_msgs.msg import MoveBaseActionGoal
from math import degrees
from nav_msgs.msg import Odometry, OccupancyGrid
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
from geometry_msgs.msg import Twist, Pose, PoseStamped, PoseWithCovarianceStamped,Quaternion,Pose2D
from tf.transformations import euler_from_quaternion
import numpy as np
from std_msgs.msg import String

from nav_msgs.msg import GridCells

from Queue import *
from math import *
from a2star import Node, Graph
from lab3_pathPlanning import publishCells
from usefulFunctions import *

def convertToGridCell(xPos, yPos):
    resolution = globalMap.info.resolution
    return int(((xPos - globalMap.info.origin.position.x) - 0.5 * resolution) / resolution), int(((yPos - globalMap.info.origin.position.y) - 0.5 * resolution) / resolution)

# keeps the global up to date with the most currnet published
def updateMap(occGrid):
    print "Read globalMap"
    global globalMap
    globalMap = occGrid

# runs the actual bfs to find frontiers
# can add functions to find entire frontier and pick point in the middle of it
def frontierFinder(status_msg):
    print "Checking msg:\n", status_msg.status_list
    lst = status_msg.status_list[0]
    yay = "Goal reached"
    if yay in lst.text: 
        # do the breadth fist search things and publish the resulting destination
        # otherwise do nothing
        sub_status.unregister()
        print "Running frontier search"

        # not convinced we need anything more than the x and y from this but I included the whole
        # thing just in case
        initPose = getPoseWithCovarianceStampedFromOdom(odom_list)
        (initX, initY) = convertToGridCell(initPose.pose.pose.position.x, initPose.pose.pose.position.y)
        
        start = Node.makeNode(initX, initY, 0, None) 
        graph = Graph(globalMap.info.width, globalMap.info.height, globalMap)  
        #  BFS starts here#########################################################
        frontierCells = breadthFirst(start, graph)
        frontierCoords = map(lambda x:(x.x,x.y), frontierCells)

        # if there is a frontier to be found, publish the next cell as a goal
        if len(frontierCells) !=0 :
            goalCell = frontierCells[0]
            newGoal = buildPoseStamped(goalCell, globalMap)
            bfsNode_pub.publish(newGoal)

def breadthFirst(start, graph):
    openSet = Queue()
    found = []
    frontier = []
    openSet.put(start)

    resolution = globalMap.info.resolution

    cells = GridCells()
    cells.header.frame_id = 'map'
    cells.cell_width = resolution 
    cells.cell_height = resolution
    
    # keepSearching = True
    while not openSet.empty() and keepSearching:
        current = openSet.queue[0]
        print "Examining: ", current
        print "Cell Value", graph.cellValue(current)

        # found an unknown cell, uncommend the following line for frontier discovery
        if graph.cellValue(current) == -1:
            print "Found -1 at:\n", current
            found.append(current)
            frontier.append(current)
            # publish the new cell
            publishCells([current], opB_pub, globalMap)
        else:
            # print"Graph", graph.getNeighbors(current), graph.width, graph.height
            for n in graph.getNeighbors(current):
                if n not in found and n not in openSet.queue:
                    print "n", n
                    openSet.put(n)
            found.append(openSet.get())

    return frontier

# set up globals and initialize node/subscribers, rospy
def main():
    rospy.init_node('exploration')

    global opB_pub
    global bfsNode_pub
    global sub_status
    global globalMap
    global odom_list

    sub_status = rospy.Subscriber('/move_base/status', GoalStatusArray, frontierFinder, queue_size = 10)
    sub_gMap = rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, updateMap, queue_size = 10)

    # fix this topic
    opB_pub = rospy.Publisher('/frontierCells', GridCells, None, queue_size = 10) 
    bfsNode_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, None, queue_size = 10)

    odom_list = tf.TransformListener()

    while not rospy.is_shutdown():
        rospy.spin()


if __name__ == '__main__':
    main()