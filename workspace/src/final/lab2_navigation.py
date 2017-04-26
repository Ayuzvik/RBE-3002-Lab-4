#!/usr/bin/env python

import rospy, tf, copy, math
from math import degrees
from kobuki_msgs.msg import BumperEvent
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, PoseStamped, PoseWithCovarianceStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose2D
from tf.transformations import euler_from_quaternion
import numpy as np
from std_msgs.msg import String
# Add additional imports for each of the message types used

#Radius of the wheels in m and distance between them
wheelR = 0.038
wheelB = 0.3048 

def moveMsg(linearVel, angularVel): #
    global pub
    message = Twist()
    message.linear.x = linearVel
    message.angular.z = angularVel
    pub.publish(message)

#drive to a goal subscribed as /move_base_simple/goal
def navToPose(goal): #+-
    global pose
    global initPose
    global first
    print pose, initPose
   # flags for pre-build map initial positions
    # if first:
    #     first = False
    #     initPosX = 2.0
    #     initPosY = 2.0
    #     startAngleRad = 0.0
    # else:
    initPosX = pose.position.x + initPose.pose.position.x
    initPosY = pose.position.y + initPose.pose.position.y
    startAngleRad = pose.orientation.z + initPose.pose.orientation.z
    goalY = goal.pose.position.y
    goalX = goal.pose.position.x
    
    startAngleRad = pose.orientation.z
    startAngleDeg = startAngleRad * (180/math.pi)

    distanceLeft = math.sqrt(math.pow((goalY - initPosY),2)+math.pow(goalX-initPosX,2))
    goalAngle = math.atan2((goal.pose.position.y - initPosY), (goal.pose.position.x - initPosX))
    anglef = goalAngle-startAngleRad 
    anglef *= (180/math.pi)

    ####
    x_q = goal.pose.orientation.x
    y_q = goal.pose.orientation.y
    z_q = goal.pose.orientation.z
    w_q = goal.pose.orientation.w

    quat = goal.pose.orientation         #in quaternion
    q = [x_q, y_q, z_q, w_q] 
    roll, pitch, yaw = euler_from_quaternion(q) #from quat to euler 

    fx = (math.pow(x_q,2)+math.pow(w_q,2)-math.pow(y_q,2)-math.pow(z_q,2))
    fy = 2 * (x_q * y_q + z_q * w_q)
    # yaw = math.atan2(fy, fx)

    goalTheta = yaw*(180/math.pi)     #convert to degrees
    th2 = (-1 * goalTheta)
    print "Get to", (goalX, goalY, goalTheta), " from" , (initPosX, initPosY, startAngleDeg)
    print "angle1", anglef
    print "distance", distanceLeft
    print "goalTheta", goalTheta
    print "\nstarting pose angle", math.degrees(pose.orientation.z)
    rotate(anglef)
    driveStraight(0.3, distanceLeft)
    rotate(goalTheta)
    newStart = PoseWithCovarianceStamped()
    newStart.pose.pose.position = goal.pose.position
    newStart.pose.pose.orientation = goal.pose.orientation
    print "About to publish to aStar node:\n", newStart    
    pub_startPose.publish(newStart)


#This function sequentially calls methods to perform a trajectory.
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def executeTrajectory(): #
    print "Executing tajectory"
    #Drive forward: 60cm 
    driveStraight(1, 0.6) #FIX SPEED
    #Turn right: 90
    rotate(math.degrees((math.pi/2))) #- or +?
    #Drive forward: 45cm 
    driveStraight(1, 0.45) #FIX SPEED
    #Turn left: 135
    rotate(math.degrees(((3*math.pi)/4)))
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++



#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#This function accepts two wheel velocities and a time interval.
def spinWheels(u1, u2, time): #

    global pub
    #math for linear and angular speeds
    linVel = (u1 + u2)/2
    angVel = (1/wheelB) * (u1 - u2)
    #print linVel
    #print angVel
    #Moving messages
    moveM = Twist();
    stopM = Twist();
    moveM.linear.x = linVel
    moveM.angular.z = angVel
    stopM.linear.x = 0
    stopM.angular.z = 0

    now = rospy.Time.now().secs
    #while the time running is less then time passed run
    while(rospy.Time().now().secs - now < time):
        moveMsg(linVel,angVel)
    moveMsg(0,0) #stop the run
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#This function accepts a speed and a distance for the robot to move in a straight line
def driveStraight(speed, distance):#

    global pose 
    rospy.Duration(0.2, 0)
    tolerance = 0.05
    initX = pose.position.x 
    initY = pose.position.y
    yay = False #Flag for while loop. Has goal been acomplished
    print "start driving"
    while(not yay): 
        goalX = pose.position.x     
        goalY = pose.position.y
        distanceTraveled = math.sqrt(math.pow((goalX - initX),2) + math.pow((goalY - initY),2))
        print "distance traveled", distanceTraveled, "    goal dist", distance
        if (distanceTraveled >= distance - tolerance and distanceTraveled <=distance + tolerance): 
            print "done driving"
            yay = True  #If at the position change flag    
            moveMsg(0, 0)      # stop the robot
            
        else:
            moveMsg(speed,0)  #Move robot
            rospy.sleep(0.15)
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    



#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#Accepts an angle and makes the robot rotate around it.

# currently this rotates the robot to match the angle given (w/respect to x_global)
def rotate(angle):

    #+-
    global pose 
    global odom_list

    
    error = angle
    goalAngle = angle + math.degrees(pose.orientation.z)
    vel = Twist()
    if angle >= 0:
        vel.angular.z = 0.7
    else:
        vel.angular.z = -0.7

    if angle > 180:
        angle -= 360
    elif angle < -180:
        angle += 360
    print "angle", angle
    while abs(error) >= 4 and not rospy.is_shutdown():
        print "turning, current pose:", math.degrees(pose.orientation.z)
        pub.publish(vel)
        rospy.sleep(0.001)
        # error = angle - math.degrees(pose.orientation.z)
        error = goalAngle - math.degrees(pose.orientation.z)
    print "Final angle rotated", angle
    print "Final pose angle", math.degrees(pose.orientation.z)
    print "goal angle", goalAngle
    print "Error", error
    vel.angular.z = 0
    pub.publish(vel)


#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++




#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#This function works the same as rotate how ever it does not publish linear velocities.
def driveArc(radius, speed, angle): #

    global pose
    #calculate velocity and how much rotate
    angVel = speed / radius
    howMuchR = math.degrees(pose.orientation.z) + angle
    #while absolute value of how much to rotate is more then 2
    while (abs(howMuchR) >= 1): 
        moveMsg(speed, angVel) 
        #check angle again
        howMuchR = angle + math.degrees(pose.orientation.z) 
    moveMsg(0, 0) 
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

def setInitPose(msg):
    global initPose
    initPose.pose.position.x = msg.pose.pose.position.x
    initPose.pose.position.y = msg.pose.pose.position.y
    initPose.pose.orientation.z = msg.pose.pose.orientation.z

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#Bumper Event Callback function
def readBumper(msg): #
    if (msg.state == 1):
        #when bumper is pressed execute trajectory
        print "execute Trajectory"
        executeTrajectory()
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

def doACompleteSpin():
    rotate(180)
    rotate(-180)

# (Optional) If you need something to happen repeatedly at a fixed interval, write the code here.
# Start the timer with the following line of code: 
#   rospy.Timer(rospy.Duration(.01), timerCallback)
def timerCallback(event): #+-
    global pose
    pose = Pose() 
    try:
        # print "Transforming to /odom from /base_footprint"
        odom_list.waitForTransform('odom','base_footprint', rospy.Time(0), rospy.Duration(2.0)) 
        (position, orientation) = odom_list.lookupTransform('odom','base_footprint', rospy.Time(0))
        pose.position.x = position[0] 
        pose.position.y = position[1]
        pose.position.z = position[2]
        quat = orientation         #in quaternion
        q = [quat[0], quat[1], quat[2], quat[3]] 
        roll, pitch, yaw = euler_from_quaternion(q) #from quat to euler 

        pose.orientation.z = yaw    
        theta = math.degrees(yaw)  
    except rospy.RosInterruptException():
        print "could not transform:\n", pose

# This is the program's main function
if __name__ == '__main__':  #
    # Change this node name to include your username
    rospy.init_node('lab2_navigation')

    # These are global variables. Write "global <variable_name>" in any other function to gain access to these global variables 
    global pub
    global pose
    global initPose
    global pub_startPose
    global odom_tf
    global odom_list
    global init_sub
    global init2_sub
    global first
    first = True
    initPose = PoseStamped()
    pose = Pose()

    # Replace the elipses '...' in the following lines to set up the publishers and subscribers the lab requires
    pub = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist, None, queue_size = 1) # Publisher for commanding robot motion
    pub_startPose = rospy.Publisher('/intermediatepose', PoseWithCovarianceStamped, None, queue_size = 1)
    # bumper_sub = rospy.Subscriber('mobile_base/events/bumper', BumperEvent, readBumper, queue_size=1) # Callback function to handle bumper events
    nav_sub = rospy.Subscriber('/mywaypoint', PoseStamped, navToPose, queue_size=10)
    init_sub = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, setInitPose, queue_size=10)
    #spin_sub = rospy.Subscriber('/spin', Twist, doACompleteSpin, queue_size = 10)
    odom_list = tf.TransformListener()
    
    # Use this command to make the program wait for some seconds
    # rospy.sleep(rospy.Duration(1, 0))
    print "Starting Lab 2"

    #make the robot keep doing something...
    rospy.Timer(rospy.Duration(0.05), timerCallback)

    #rotate(-180)
    #rotate(180)
    while(not rospy.is_shutdown()): #while  loop
        x=5
    #      print"boop?"
    # Make the robot do stuff...
    #straight(1, 1)    #+
    #spinWheels(0.1,0.2,3)      #+
    #rotate(math.pi/2)      #+
    #bumper test rostopic pub /mobile_base/events/bumper kobuki_msgs/BumperEvent "state: 1"
    #driveArc(0.5,0.1,30)     #+
    #navToPose(goal)

    print "Lab 2 complete!"

