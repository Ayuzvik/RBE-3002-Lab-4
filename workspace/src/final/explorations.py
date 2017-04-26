#!/usr/bin/env python
import rospy, tf, copy, math
from move_base_msgs.msg import MoveBaseActionGoal
from math import degrees
from nav_msgs.msg import Odometry, OccupancyGrid
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
from geometry_msgs.msg import Twist, Pose, PoseStamped, PoseWithCovarianceStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose2D
from tf.transformations import euler_from_quaternion
import numpy as np
from std_msgs.msg import String

from nav_msgs.msg import GridCells

from Queue import *
from math import *
from a2star import Node, Graph
from lab3_pathPlanning import publishCells

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
        odom_list.waitForTransform('odom','base_footprint', rospy.Time(0), rospy.Duration(5.0)) 
        (position, orientation) = odom_list.lookupTransform('odom','base_footprint', rospy.Time(0))
        initPose = PoseWithCovarianceStamped()
        initPose.pose.pose.position.x = position[0] 
        initPose.pose.pose.position.y = position[1]
        initPose.pose.pose.position.z = position[2]
        quat = orientation         #in quaternion
        q = [quat[0], quat[1], quat[2], quat[3]] 
        roll, pitch, yaw = euler_from_quaternion(q) #from quat to euler 

        initPose.pose.pose.orientation.z = yaw    
        theta = math.degrees(yaw) 

        (initX, initY) = convertToGridCell(initPose.pose.pose.position.x, initPose.pose.pose.position.y)
        start = Node.makeNode(initX, initY, 0, None) 

        graph = Graph(globalMap.info.width, globalMap.info.height, globalMap)  
        #  BFS starts here#########################################################
        openSet = Queue()
        found = []
        openSet.put(start)

        resolution = globalMap.info.resolution

        cells = GridCells()
        cells.header.frame_id = 'map'
        cells.cell_width = resolution 
        cells.cell_height = resolution
        # while len(openSet) != 0:
        keepSearching = True
        while not openSet.empty() and keepSearching:
            current = openSet.queue[0]
            print "Examining: ", current
            print "Cell Value", graph.cellValue(current)

            # found an unknown cell
            # if graph.cellValue(current) == -1:
            if current.x == 90 and current.y == 90:
                print "Found -1 at:\n", current
                found.append(current)
                point = graph.convertNodeToPoint(current)
                cells.cells = [point]
                opB_pub.publish(cells)
                
                pose = PoseStamped()
                pose.pose.position = point
                pose.pose.orientation.x = 0
                pose.pose.orientation.y = 0
                pose.pose.orientation.z = 0
                pose.pose.orientation.w = 1
                # need to convert the pose to map frame to make it work as a goal?
                bfsNode_pub.publish(pose)
                keepSearching = False           
                break

            # print"Graph", graph.getNeighbors(current), graph.width, graph.height
            for n in graph.getNeighbors(current):
                if n not in found and n not in openSet.queue:
                    print "n", n
                    openSet.put(n)
            found.append(openSet.get())
            # print"Found", found
        ##################################################################3

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