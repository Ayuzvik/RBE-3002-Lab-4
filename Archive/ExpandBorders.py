#!/usr/bin/python
import rospy, math
import rospy, tf, numpy, math, sys
from std_msgs.msg import String
#from geometry_msgs.msg import Twist, Pose, Point
#from nav_msgs.msg import Odometry, OccupancyGrid
#from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import GridCells
#from Queue import PriorityQueue
from math import sqrt
from nav_msgs.msg import OccupancyGrid


def mapCallback(data):
	global pub_expMap

	expBy = int(math.ceil(rospy.get_param('exp', 0.25)/data.info.resolution))

	height = data.info.height
	width = data.info.width

	expMap = OccupancyGrid()
	expMap.info = data.info
	expMap.data = [0 for x in range(width*height)]

	for node in range(0, width*height):
		for n in getNeighbors(data, node, expBy):
			if n > 70:
				expMap.data[node] = 100
				break

	pub_expMap.publish(expMap)

def getNeighbors(data, node, expBy):
	width = data.info.width
	height = data.info.height
	neighbors = []

	for x in range(-expBy, expBy+1):
		for y in range(-expBy, expBy+1):
			a = node + x + width*y
			if a > 0 and a < width*height:
				neighbors.append(data.data[a])

	return neighbors

if __name__ == '__main__':

	global pub_expMap
	global map_sub

	rospy.init_node('ExpandBorders')
	
	pub_expMap = rospy.Publisher('/exp_map', OccupancyGrid, mapCallBack)
	map_sub = rospy.Subscriber('/map', OccupancyGrid, mapCallback, queue_size=10)
	rospy.spin()