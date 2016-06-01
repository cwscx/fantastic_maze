import numpy as np
import random

class Maze:

	def __init__(self, size=10, charger=False, pits=False, monster=False):
		# Initialize a square of given size
		self.maze = np.zeros((size, size), dtype=np.int)
		self.size = size

		# Select all the outside positions
		outsides = [(i, j) for j in xrange(size) for i in [0, size - 1]] + \
					[(i, j) for j in [0, size - 1] for i in xrange(1, size - 1)]
		
		# Randomly pick different two positions in outsides as entry and exit
		# Both entry and exit are tuples
		self.entry = random.choice(outsides)
		self.exit = random.choice(filter(lambda x: x != self.entry, outsides))

		

	def DFS_path(self, start=self.entry, goal=self.exit, random=False):
		if not random:
			stack = [start]
			path = []
			
			while not stack.empty():
				node = path[0]
				



	def _return_free_neighbours(self, pos):
		neighbours = [
			(pos[0] - 1, pos[1] - 1),
			(pos[0] - 1, pos[1] + 1),
			(pos[0] + 1, pos[1] - 1),
			(pos[0] + 1, pos[1] + 1),
		]

		return filter(lambda n: n[0] >= 0 or n[0] < self.size or 
								n[1] >= 0 or n[1] < self.size or
								self.maze[n] == 0, 
								neighbours)

