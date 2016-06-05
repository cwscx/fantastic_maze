#!/usr/bin/env python
import rospy

from maze_builder import Maze
from helpers import *
from sensor_msgs.msg import LaserScan
from fantastic_maze.msg import GoalSense

class Robot():
	def __init__(self):
		# maze = Maze(10)
		rospy.init_node('robot')
		self.laserSub = rospy.Subscriber('/base_scan', LaserScan, self.handleLaser)
		self.goalScan = rospy.Subscriber('/goal_sense', GoalSense, self.handleGoal)
		self.laser = LaserScan()	# class variable which will always contain the latest laser scan data
		self.goals = GoalSense()	# class variable which will always contain the latest goal sensor data
		rospy.sleep(2)
		
		move()
		rospy.sleep(1)
		turn(90)
		rospy.sleep(1)
		
	def handleLaser(self, message):
		self.laser = message

	def handleGoal(self, message):
		self.goals = message

if __name__ == "__main__":
	r = Robot()
