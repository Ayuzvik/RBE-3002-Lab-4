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
from a2star import *

 
xAdjust = 0.5
yAdjust = 0.5
def mapCallBack(data):
    global globalMap 
    global mapData
    global width
    global height
    global mapgrid
    global resolution
    global offsetX
    global offsetY

    globalMap = data
    offsetX = data.info.origin.position.x
    offsetY = data.info.origin.position.y
    mapgrid = data
    resolution = data.info.resolution
    mapData = data.data
    width = data.info.width
    height = data.info.height
    print "Loaded global map"
    
    # publishCells(mapgrid)
    # printMapData(mapData, width, height)

    #print data.info
#___________________________________________________

def readLocalMap(data):
    global local_mapData
    global local_width
    global local_height
    global local_mapgrid
    global local_resolution
    global local_offsetX
    global local_offsetY

    print "Updating local costmap"

    local_offsetX = data.info.origin.position.x
    local_offsetY = data.info.origin.position.y
    local_mapgrid = data
    local_resolution = data.info.resolution
    local_mapData = data.data
    local_width = data.info.width
    local_height = data.info.height
    

def parseMapData(data, width, height):
    myMap = []
    row = 0
    for i in range(len(data)):
        if i % width == 0:
            row += 1
            print "" # newline after full row
        sys.stdout.write('%4s' %(data[i]))    

def buildPoseStamped(node, nextNode):
    msg = PoseStamped()
    p1 = nodeToPoint(node)
    p2 = nodeToPoint(nextNode)

    yaw = atan2((p2.y - p1.y),(p2.x-p1.x))

    msg.pose.position = nodeToPoint(node)
    orientation = tf.transformations.quaternion_from_euler(yaw, 0, 0)
    msg.pose.orientation.x = orientation[0]
    msg.pose.orientation.y = orientation[1]
    msg.pose.orientation.z = orientation[2]
    msg.pose.orientation.w = orientation[3]
    print "About to publish to navToPose node:\n", msg
    # dummy = raw_input("Press Enter to send...")
    return msg


def publishCells(info, publisher):
    # print "publishing", str(publisher)

    # resolution and offset of the map
    a=0
    cells = GridCells()
    cells.header.frame_id = 'map'
    cells.cell_width = resolution 
    cells.cell_height = resolution
    
    for node in info:
        point = nodeToPoint(node)
        cells.cells.append(point)
    #pub.publish(cells)
    publisher.publish(cells)

def convertToGridCell(xClick, yClick):
    return int(((xClick - offsetX) - xAdjust * resolution) / resolution), int(((yClick - offsetY) - yAdjust * resolution) / resolution)

def nodeToPoint(node):
    point = Point()
    point.x = (node.x * resolution) + (xAdjust * resolution) + offsetX
    point.y = (node.y * resolution) + (yAdjust * resolution) + offsetY
    return point

def nodeToPointReal(node, xAdj, yAdj):
    point = Point()
    point.x = (node.x * resolution) + (xAdj * resolution) + offsetX
    point.y = (node.y * resolution) + (yAdj * resolution) + offsetY
    return point


def convertInitNode(pose):
    global init_x_cord
    global init_y_cord
    global final_x_cord
    global final_y_cord
    print "converting node"    
    init_x_cord, init_y_cord = convertToGridCell(pose.pose.pose.position.x , pose.pose.pose.position.y)
    print "Start: ", (init_x_cord, init_y_cord)
    start = Node.makeNode(init_x_cord, init_y_cord)
    goal = Node.makeNode(final_x_cord, final_y_cord)
    
    # getNCheck(goal) #runs the getNCheck function
    waypoints = []
    if start != goal:
        openSet = PriorityQueue()
        costSoFar = {}
        graph = Graph(width, height, globalMap)
        path, waypoints = runNavByWaypoint(start, goal, graph, openSet, costSoFar, publishAll)
        
        # print waypoints
        publishCells(path, pub_path)
        publishCells(waypoints, pub_waypoints)
        # print "Start:", start, "   Goal:", goal, "  Waypoint[-1]:", waypoints[-1]
        # for n in waypoints:
        #     print n
        
        print "\n"
        next = waypoints[-1]
        next2 = waypoints[-2]
        pub_myWaypoint.publish(buildPoseStamped(next, next2))
        start = next

        # dummy = raw_input("Press enter to continue...")

def convertFinalNode(pose):
    global final_x_cord
    global final_y_cord
    final_x_cord, final_y_cord = convertToGridCell(pose.pose.position.x, pose.pose.position.y)
    print "Goal:", (final_x_cord, final_y_cord)
    

def isTurn(before, now, after):
    # if not ((before.x == after.x) ^ (before.y == after.y)):
    #     return True

    if before.x - after.x != 0 and before.x - now.x !=0:
        mBtoA = float(before.y - after.y) / (before.x - after.x)
        mBtoN = float(before.y - now.y) / (before.x - now.x)
        sameLine = abs(mBtoA - mBtoN) > 0.000001
        # print "Same Line?", before, now, after, sameLine
        return sameLine
    else :
        return now.x - after.x != 0

# def getNCheck(goal):

#     while ((node.x != goal.x) && (node.y != goal.y)):

#         node.x = waypoints(1).x
#         node.y = waypoints(1).y
        #run A*

def runNavByWaypoint(start, goal, graph, openSet, costSoFar, publishAll = None):
    path = aStar(start, goal, graph, openSet, costSoFar, publishAll)
    # print path
    print "Path length:", len(path)
    waypoints = []
    for i in range(1, len(path)-1):
        if isTurn(path[i-1], path[i], path[i+1]) and path[i] != start:
            waypoints.append(path[i])
    # if len(path) != 0:
    #     waypoints.append(path[-1])
    waypoints.insert(0, goal)
    
    return path, waypoints

def atEdgeOfVision(node):
    pose = Pose() 
    odom_list = tf.TransformListener()
    odom_list.waitForTransform('odom','base_footprint', rospy.Time(0), rospy.Duration(1.0)) 
    (position, orientation) = odom_list.lookupTransform('odom','base_footprint', rospy.Time(0))
    pose.position.x = position[0] 
    pose.position.y = position[1]
    pose.position.z = position[2]
    quat = orientation         #in quaternion
    q = [quat[0], quat[1], quat[2], quat[3]] 
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(q) #from quat to euler 

    pose.orientation.z = yaw    

    point = node
    x, y = convertToGridCell(pose.position.x, pose.position.y)
    transformer = tf.TransformListener()
    transformer.waitForTransform('odom', 'map', rospy.Time(0), rospy.Duration(1.0))
    point2 = transformer.lookupTransform('/map', '/odom', rospy.Time(0))
    leftEdge = local_offsetX - local_width/2 + x
    rightEdge = local_offsetX + local_width/2 + x
    bottomEdge = local_offsetY - local_height/2 + y
    topEdge = local_offsetY + local_height/2 + y
    # print "Point:", point
    # print "(left, right, top, bottom)", (leftEdge, rightEdge, topEdge, bottomEdge)
    yes = point.x < leftEdge or point.x > rightEdge or point.y  < bottomEdge or point.y < topEdge
    if yes:
        print node, "is past the edge of the current vision"
    return yes

def publishAll(openSet, costSoFar):
    closed = []
    # for key in costSoFar:
    #     if key not in openSet:
    #         closed.append(key)

    publishCells(costSoFar, pub_closed)
    publishCells(openSet, pub_open)

def run():
    global pub_open
    global pub_closed
    global pub_path
    global pub_waypoints
    global pub_myWaypoint
    global pub_initPose

    global mapData
    global localMap
    global width
    global height
    global mapgrid
    global resolution
    global offsetX
    global offsetY


    
    rospy.init_node('lab3_pathPlanning')
    # timer = rospy.Timer(rospy.Duration(0.0000001), publishAll)
    #gridCellsDet is a topic for sending data. Can rename it to whatever we want
    pub_open = rospy.Publisher("/openCells", GridCells, queue_size=1) 
    pub_closed = rospy.Publisher("/closedCells", GridCells, queue_size=1) 
    pub_path = rospy.Publisher("/pathCells", GridCells, queue_size=1) 
    pub_waypoints = rospy.Publisher("/waypointCells", GridCells, queue_size = 1)
    pub_myWaypoint = rospy.Publisher('/mywaypoint',PoseStamped, queue_size = 1)
    # pun_initPose = rospy.Publisher('/initialpose2',PoseStamped, queue_size = 1)

    sub = rospy.Subscriber("/exp_map", OccupancyGrid, mapCallBack)
    sub_initPose = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, convertInitNode, queue_size=10)   #initail pose 1 
    sub_intermediatePose = rospy.Subscriber('/intermediatepose', PoseWithCovarianceStamped, convertInitNode, queue_size=10)
    sub_finalPose = rospy.Subscriber('/move_base_simple/goal1', PoseStamped, convertFinalNode, queue_size=10) #goal 1 - need that to avoid Rviz running built in stuff
    sub_exp_map = rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, readLocalMap, queue_size = 1)
    rospy.sleep(10)



    while (1 and not rospy.is_shutdown()):
        # publishCells(mapData) 
        rospy.sleep(2)  
        # print("Complete")


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass


