#!/usr/bin/env python
import rospy

from maze_builder import Maze


if __name__ == "__main__":
	maze = Maze(10)
	rospy.sleep(50)
