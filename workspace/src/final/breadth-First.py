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

def findUnknownBF(start, graph):
#	openSet = []
	openSet = Queue()
	found = []
	openSet.put(start)
	# while len(openSet) != 0:
	while not openSet.empty():
		current = openSet.queue[0]
		print "Examinging: ", current
		print "Cell Value", graph.cellValue(current)
		if graph.cellValue(current) == -1:
			print "Found -1 at:\n", current
			return current

		# for n in openSet.queue:
		# 	print "open set", n

		for n in graph.getNeighbors(current):
			if n not in found and n not in openSet.queue:
				print "n", n
				openSet.put(n)
		found.append(openSet.get())

		#dummy = raw_input ("Press Enter...")
		#print "found", found
		#opB_pub.publish(openSet)


	print "Could not find any new frontiers"
	return None

def runBFSearch(globalMap):
	w = globalMap.info.width
	h = globalMap.info.height
	cells = globalMap.data
	graph = Graph(w,h,cells)

	start = Node(0, 0, 0, None)
	print "starting bfSearch"
	print findUnknownBF(start, graph)

if __name__ == '__main__':
	#rospy.init_node('bfSearch')
	rospy.init_node('breadth-First')
	#map_sub = rospy.Subscriber('/map', OccupancyGrid, runBFSearch, queue_size = 10)
	map_sub = rospy.Subscriber('/move_base/local_costmap/costmap', OccupancyGrid, runBFSearch, queue_size = 10)
	#opB_pub = rospy.publish('/opB', OccupancyGrid, queue_size = 10)
	#exp_map_sub = rospy.Subscriber('/exp_map', OccupancyGrid, runBFSearch, queue_size = 10)

	while not rospy.is_shutdown():
		rospy.spin()
 