#!usr/bin/env python
# a collection of useful functions for conversions
# or other things for turtlebots
import rospy, tf, copy, math

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, 
from tf.transformations import euler_from_quaternion


def getPoseWithCovarianceStampedFromOdom(odom_list):
	initPose = PoseWithCovarianceStamped
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
  	
  	return initPose

def convertToGridCoords(xPos, yPos, globalMap):
    resolution = globalMap.info.resolution
    return int(((xPos - globalMap.info.origin.position.x) - 0.5 * resolution) / resolution), int(((yPos - globalMap.info.origin.position.y) - 0.5 * resolution) / resolution)

def nodeToPoint(node, globalMap):
	resolution = glboalMap.info.resolution
	offsetX = globalMap.info.origin.position.x
	offsetY = globalMap.info.origin.position.y
    point = Point()
    point.x = (node.x * resolution) + (0.5 * resolution) + offsetX
    point.y = (node.y * resolution) + (0.5 * resolution) + offsetY
    return point

def publishCells(listOfNodes, publisher, globalMap):
    # print "publishing", str(publisher)
    resolution = globalMap.info.resolution
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

def buildPoseStamped(node, globalMap):
	msg = PoseStamped()
	orientation = tf.transformations.quaternion_from_euler(0, 0, 0)

    msg.pose.position = nodeToPoint(node, globalMap)
    msg.pose.orientation.x = orientation[0]
    msg.pose.orientation.y = orientation[1]
    msg.pose.orientation.z = orientation[2]
    msg.pose.orientation.w = orientation[3]

    return msg