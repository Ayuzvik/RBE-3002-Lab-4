#!/usr/bin/env python

import rospy, tf, numpy, math, sys

#from kobuki_msgs.msg import BumperEvent
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose, Point
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion
#from tf.transformations import euler_from_quaternion
from nav_msgs.msg import GridCells

from Queue import PriorityQueue
from math import *
from a2star import Node, Graph

def findUnknownBF(start, graph):
	openSet = []
	found = []
	openSet.append(start)
	while len(openSet) != 0:
		current = openSet[0]
		print "Examinging: ", current
		if graph.cellValue(current) == -1:
			print "Found -1 at:\n", current
			return current

		for n in graph.getNeighbors(current):
			if n not in found:
				openSet.append(n)
		found.append(openSet.pop())

	print "Could not find any new frontiers"
	return None

def runBFSearch(globalMap):
	w = globalMap.info.width
	h = globalMap.info.height
	cells = globalMap.data
	graph = Graph(w,h,cells)

	start = Node(10, 10, 0, None)
	print "starting bfSearch"
	print findUnknownBF(start, graph)

if __name__ == '__main__':
	rospy.init_node('bfSearch')
	map_sub = rospy.Subscriber('/map', OccupancyGrid, runBFSearch, queue_size = 10)

	while not rospy.is_shutdown():
		rospy.spin()