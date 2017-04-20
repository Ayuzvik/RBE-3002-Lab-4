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
    # while len(openSet) != 0:
    while not openSet.empty():
        current = openSet.queue[0]
        # print "Examinging: ", current
        # print "Cell Value", graph.cellValue(current)
        if graph.cellValue(current) == -1:
            # print "Found -1 at:\n", current
            return current

        # for n in openSet.queue:
        #   print "open set", n

        for n in graph.getNeighbors(current):
            if n not in found and n not in openSet.queue:
                # print "n", n
                openSet.put(n)
        found.append(openSet.get())

        
        cells = GridCells()
        cells.header.frame_id = 'map'
        cells.cell_width = resolution 
        cells.cell_height = resolution
        
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
    #pub.publish(cells)
    publisher.publish(cells)

def runBFSearch(globalMap):
    global resolution
    resolution = globalMap.info.resolution
    w = globalMap.info.width
    h = globalMap.info.height
    cells = globalMap.data
    graph = Graph(w,h,globalMap)

    start = Node(0, 0, 0, None)
    print "starting bfSearch"
    print findUnknownBF(start, graph)



if __name__ == '__main__':
    #rospy.init_node('bfSearch')
    rospy.init_node('breadth-First')
    map_sub = rospy.Subscriber('/map', OccupancyGrid, runBFSearch, queue_size = 10)
    # start_sub = rospy.Subscriber('/', PoseWithCovarianceStamped, setStart, queue_size = 10)
    # map_sub = rospy.Subscriber('/move_base/local_costmap/costmap', OccupancyGrid, runBFSearch, queue_size = 10)
    opB_pub = rospy.Publisher('/opB', GridCells, None, queue_size = 10)
    #exp_map_sub = rospy.Subscriber('/exp_map', OccupancyGrid, runBFSearch, queue_size = 10)

    while not rospy.is_shutdown():
        rospy.spin()
