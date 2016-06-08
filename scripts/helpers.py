#!/usr/bin/env python
import rospy
from math import *
from geometry_msgs.msg import Pose, PoseArray, PointStamped, Quaternion, Point, Twist
from fantastic_maze.msg import GoalSense

cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
goal_sensor = rospy.Publisher('goal_sense', GoalSense, queue_size = 10)

# hard coded constants: TODO modularize
board_size = 10
position = [0,4] # start position
goal = [4,9]
direction = 0

# function to move robot one grid position forward in current angle. 
def move():
	twist = Twist()
	twist.linear.x = 20
	cmd_vel_pub.publish(twist)
	rospy.sleep(1)
	twist = Twist()
	cmd_vel_pub.publish(twist)

	#calculate new position and publish goal sensor values
	if direction == 0:
		position[0] += 1
	elif direction == 90:
		position[1] -= 1
	elif direction == 180:
		position[0] -= 1
	elif direction == 270:
		position[1] += 1
	senseGoal()

# turns robot parameter angle number of degrees, without moving. 
def turn(angle):
	print('TURNING %d angles' % angle)
	global direction
	direction = (direction + angle) % 360
	senseGoal()

	""" First Turn (in 10 degrees increments) """
	angle_in_rad = float(angle*pi/180.0)
	if (angle > 0):
		increment = 10	
	else:
		increment = -10

	increment_in_rad = float(increment*pi/180.0)
	while angle_in_rad != 0:
		twist = Twist()
		""" Stage Doesn't Handle Large Turns accurately"""
		if abs(angle_in_rad) > abs(increment_in_rad) :
			twist.angular.z = increment_in_rad
			angle_in_rad -= increment_in_rad
		else:
			twist.angular.z = angle_in_rad
			angle_in_rad = 0

		cmd_vel_pub.publish(twist)
		rospy.sleep(1)
		twist = Twist()
		cmd_vel_pub.publish(twist)

"""
goal sensor, returns 4 boolean values depending upon whether the goal is within 
half of the width of the entire gridsize in that direction.
"""
def senseGoal():
	straight, back, left, right = False, False, False, False
	xDist = abs(position[0] - goal[0])
	yDist = abs(position[1] - goal[1])
	if yDist < board_size / 2 and position[0] == goal[0]:
		if direction == 90:
			if position[1] - goal[1] > 0:
				straight = True
			else:
				back = True
		elif direction == 270:
			if position[1] - goal[1] > 0:
				back = True
			else:
				straight = True
		elif direction == 0:
			if position[1] - goal[1] > 0:
				left = True
			else:
				right = True
		elif direction == 180:
			if position[1] - goal[1] > 0:
				right = True
			else:
				left = True
	elif xDist < board_size / 2 and position[1] == goal[1]:
		if direction == 0:
			if position[0] - goal[0] > 0:
				back = True
			else:
				straight = True
		elif direction == 90:
			if position[0] - goal[0] > 0:
				left = True
			else:
				right = True
		elif direction == 180:
			if position[0] - goal[0] > 0:
				straight = True
			else:
				back = True
		elif direction == 270:
			if position[0] - goal[0] > 0:
				left = False
			else:
				right = True

	g = GoalSense()
	g.straight = straight
	g.back = back
	g.left = left
	g.right = right
	g.goal = position[0] == goal[0] and position[1] == goal[1]

	goal_sensor.publish(g)