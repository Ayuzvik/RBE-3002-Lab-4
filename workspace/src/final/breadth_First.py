#!/usr/bin/env python

import rospy, tf, numpy, math, sys

#from kobuki_msgs.msg import BumperEvent
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose, Point
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion
#from tf.transformations import euler_from_quaternion
from nav_msgs.msg import GridCells

#from Queue import PriorityQueue
from Queue import *
from math import *
from a2star import Node, Graph
from lab3_pathPlanning import publishCells

def findUnknownBF(start, graph):
#   openSet = []
    openSet = Queue()
    found = []
    openSet.put(start)

    cells = GridCells()
    cells.header.frame_id = 'map'
    cells.cell_width = resolution 
    cells.cell_height = resolution
    # while len(openSet) != 0:
    while not openSet.empty():
        current = openSet.queue[0]
        print "Examining: ", current
        print "Cell Value", graph.cellValue(current)
        if graph.cellValue(current) == -1:
            # print "Found -1 at:\n", current
            found.append(current)
            point = graph.convertNodeToPoint(current)
            cells.cells = [point]
            opB_pub.publish(cells)
            pose = PoseStamped()
            pose.pose.position = point
            bfsNode_pub.publish(pose)            
            return current

        # for n in openSet.queue:
        #   print "open set", n

        for n in graph.getNeighbors(current):
            if n not in found and n not in openSet.queue:
                # print "n", n
                openSet.put(n)
        found.append(openSet.get())
        
    for node in found:
        point = graph.convertNodeToPoint(node)
        cells.cells.append(point)

    #dummy = raw_input ("Press Enter...")
    #print "found", found
    opB_pub.publish(cells)

    print "Could not find any new frontiers"
    return None

def publishCells(info, publisher):
    # print "publishing", str(publisher)

    # resolution and offset of the map
    a=0
    cells = GridCells()
    cells.header.frame_id = 'map'
    cells.cell_width = resolution 
    cells.cell_height = resolution
    
    for node in info:
        point = graph.nodeToPoint(node)
        cells.cells.append(point)
    pub.publish(cells)
    publisher.publish(cells)


def runBFSearch(globalMap):
    global resolution
    global initPose
    global offsetX
    global offsetY
#########################################
    initPose = Pose()
    odom_list = tf.TransformListener()
    odom_list.waitForTransform('odom','base_footprint', rospy.Time(0), rospy.Duration(1.0)) 
    (position, orientation) = odom_list.lookupTransform('odom','base_footprint', rospy.Time(0))
    initPose.position.x = position[0] 
    initPose.position.y = position[1]



#########################################
    resolution = globalMap.info.resolution
    offsetX = globalMap.info.origin.position.x
    offsetY = globalMap.info.origin.position.y
    w = globalMap.info.width
    h = globalMap.info.height
    cells = globalMap.data
    graph = Graph(w,h,globalMap)

    (initX, initY) = convertToGridCell(initPose.position.x, initPose.position.y)

    start = Node.makeNode(initX, initY, 0, None)
    #start = Node(2000,1920, 0, None)
    print "starting bfSearch"
    print findUnknownBF(start, graph)


def convertToGridCell(xPos, yPos):
    return int(((xPos - offsetX) - 0.5 * resolution) / resolution), int(((yPos - offsetY) - 0.5 * resolution) / resolution)

if __name__ == '__main__':
    #rospy.init_node('bfSearch')
    rospy.init_node('breadth_First')

    #map_sub = rospy.Subscriber('/map', OccupancyGrid, runBFSearch, queue_size = 10)
   
    # start_sub = rospy.Subscriber('/', PoseWithCovarianceStamped, setStart, queue_size = 10)
    map_sub = rospy.Subscriber('/exp_map', OccupancyGrid, runBFSearch, queue_size = 10)
    opB_pub = rospy.Publisher('/opB', GridCells, None, queue_size = 10)
    bfsNode_pub = rospy.Publisher('/move_base_simple/goal1', PoseStamped,queue_size =1)
    #exp_map_sub = rospy.Subscriber('/exp_map', OccupancyGrid, runBFSearch, queue_size = 10)

    odom_list = tf.TransformListener()

    while not rospy.is_shutdown():
        rospy.spin()
