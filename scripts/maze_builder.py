import numpy as np
import random

class Maze:

	def __init__(self, size=10, ob_per=0.5, charger=False, pits=False, monster=False):
		# Initialize a square of given size
		self.maze = np.zeros((size, size), dtype=np.int)
		self.size = size

		# random.seed(0)

		# Select all the outside positions
		outsides = [(i, j) for j in xrange(size) for i in [0, size - 1]] + \
					[(i, j) for j in [0, size - 1] for i in xrange(1, size - 1)]
		
		# Randomly pick different two positions in outsides as entry and exit
		# Both entry and exit are tuples
		self.entry = random.choice(outsides)
		self.exit = random.choice(filter(lambda x: x != self.entry, outsides))

		path = self.DFS_path(self.entry, self.exit)		# Solution path
		# Run DFS until the solution path occupies less space then size ^ 2 - obstales
		while len(path) >= size ** 2 * ob_per:
			path = self.DFS_path(self.entry, self.exit)

		positions = [(i, j) for i in xrange(size) for j in xrange(size)]
		i = 0.0

		while i < (size ** 2 * ob_per):
			p = random.choice(positions)
			if p not in path and self.maze[p] == 0:
				self.maze[p] = 1
				i += 1

		# for pos in path:
		# 	self.maze[pos] = 2;
		# self.maze[self.entry] = 9;
		# self.maze[self.exit] = 8;

		# print self.maze
		

	def DFS_path(self, start, goal, random=False):
		# Do a standard DFS to find a path
		if not random:
			stack = [start]		# Stack to store front nodes
			path = []
			
			# DFS
			while len(stack) != 0:
				last_n = stack.pop()
				
				if last_n in path:
					continue

				path.append(last_n)

				if last_n == goal:
					break

				neighbours = self._return_free_neighbours(last_n)
				stack = stack + neighbours

			return path


	def _return_free_neighbours(self, pos):
		# Neighbours for the given position
		neighbours = [
			(pos[0] - 1, pos[1]),
			(pos[0] + 1, pos[1]),
			(pos[0], pos[1] - 1),
			(pos[0], pos[1] + 1),
		]

		random.shuffle(neighbours)	# Shuffle the neighbours for randomness

		# Return all the valid neighbours for the position
		return filter(lambda n: n[0] >= 0 and n[0] < self.size and 
								n[1] >= 0 and n[1] < self.size and
								self.maze[n] == 0, 
								neighbours)

