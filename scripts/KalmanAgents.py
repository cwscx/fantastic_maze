import numpy as np
import math
import rospy

from Kalman import *
from helpers import *
from sensor_msgs.msg import LaserScan
from fantastic_maze.msg import GoalSense

"""
	Transfer
	[(t1, t2, t3, ..., tn), (t1', t2', t3', ..., tn'), (t1'', t2'', t3'', ..., tn''')] ===> 
	[(t1, t1', t1'', ...,), (t2, t2', t2'', ...), (t3, t3', t3'', ...), ..., (tn, tn', tn'',...)]
"""
def _batch_from_array_of_tuples(array_of_tuples):
	"""
	Same as
	for i in xrange(len(x_data_m[0])):
		l = list()
		for j in xrange(len(x_data_m)):
			l.append(x_data_m[j][i])
		bath_x_data.append(l)
	"""
	return [tuple([array_of_tuples[j][i] for j in xrange(len(array_of_tuples))]) for i in xrange(len(array_of_tuples[0]))]


"""
	Transfer
	[(t1, t1', t1'', ...,), (t2, t2', t2'', ...), (t3, t3', t3'', ...), ..., (tn, tn', tn'',...)] ====>
	[(t1, t2, t3, ..., tn), (t1', t2', t3', ..., tn'), (t1'', t2'', t3'', ..., tn''')]
"""
def _batch_to_array_of_tuples(batch):
	return [ tuple([batch[j][i] for j in xrange(len(batch))]) for i in xrange(len(batch[0]))]


class simpleKalmanAgent():
	"""
		velocity : double
		pos      : tuple
		direction: {
			0     : right
			pi / 2: up
			pi    : left
			-pi/ 2: down 
		}
	"""
	def __init__(self, velocity, pos, direction, sensor_num, time_diff):
		self.velocity = velocity
		self.pos = pos
		self.direction = direction
		self.goal = None

		self._init_sensor(sensor_num, time_diff)

		# Keep track of visited free spaces / walls
		# Free Space keep track of location and its possibility of being a free space / wall
		self.knowledge_map = {
			'Entry'		: pos,
			# 'Free Space': {pos: 1.0},
			'Wall'		: [],
			'Visited'	: [pos],
			'Exit'		: None
		}
		self.x_laserData = []
		self.y_laserData = []
		self.subscriber()

		for i in range(4):
			self.process_move()

		print self.knowledge_map
		rospy.spin()

	"""
		So the sensor here is fixed to the angles, not related to the robot's directions, because
		in Kalman Model, matrix 'A' need to be fixed.
	"""
	def _init_sensor(self, sensor_num, time_diff):
		self.thetas = [i * (2 * math.pi / sensor_num) for i in xrange(sensor_num)]

		# because of math.cos(self.thetas[i]) for each different direction, 
		# we can pass in velocity on x_axis for each sensor without further modification
		self.x_As = [np.matrix([ [1, math.cos(self.thetas[i]) * time_diff], [0, 1] ]) for i in xrange(sensor_num)] 
		self.y_As = [np.matrix([ [1, math.sin(self.thetas[i]) * time_diff], [0, 1] ]) for i in xrange(sensor_num)]

		self.B = np.matrix(np.zeros([2, 2]))
		self.C = np.matrix([[1, 0], [0, 1]])

		self.R = np.matrix([[100, 0], [0, 100]])
		self.P = np.matrix([[1, 0], [0, 1]])

		# List of Kalman filters for distance measurement on x_axis of different directions
		# same for y_axis
		self.x_kalmans = [Kalman(A, self.B, self.C, self.R, self.P) for A in self.x_As]
		self.y_kalmans = [Kalman(A, self.B, self.C, self.R, self.P) for A in self.y_As]

	
	def process_move(self):
		rospy.sleep(2)
		x, y = self.process_data(self.x_laserData, self.y_laserData, [(np.matrix([[0], [0]]), np.matrix([[0], [0]]), np.matrix([[0], [0]]), np.matrix([[0], [0]])) for i in xrange(len(self.x_laserData))])
		x = x[-1]
		y = y[-1]

		print self.x_laserData[-1]
		print self.y_laserData[-1]

		d = self.get_direction(x, y)
		print d

		# Turn
		turn(((d - self.direction) % 4) * 90)
		self.direction = d
		
		# Move
		move()
		self.pos = self.dir_to_pos(self.direction)

		if self.pos not in self.knowledge_map["Visited"]:
			self.knowledge_map['Visited'].append(self.pos)

		self.x_laserData = []
		self.y_laserData = []


	def subscriber(self):
		# rospy.init_node('kalman')
		self.laserSub = rospy.Subscriber('/base_scan', LaserScan, self.handleLaser)
		self.goalScan = rospy.Subscriber('/goal_sense', GoalSense, self.handleGoal)
		rospy.sleep(1)


	def handleLaser(self, message):
		if self.direction == 0:
			self.x_laserData.append((message.ranges[2], 0, message.ranges[0], 0))
			self.y_laserData.append((0, message.ranges[3], 0, message.ranges[1]))
		elif self.direction == 1:
			self.x_laserData.append((message.ranges[1], 0, message.ranges[3], 0))
			self.y_laserData.append((0, message.ranges[2], 0, message.ranges[0]))
		elif self.direction == 2:
			self.x_laserData.append((message.ranges[0], 0, message.ranges[2], 0))
			self.y_laserData.append((0, message.ranges[1], 0, message.ranges[3]))
		else:
			self.x_laserData.append((message.ranges[3], 0, message.ranges[1], 0))
			self.y_laserData.append((0, message.ranges[0], 0, message.ranges[2]))
			
		#print(self.laser)

	def handleGoal(self, message):
		self.goals = message


	def get_direction(self, x, y):
		ds = [x[0], y[1], x[2], y[3]]
		print ds
		
		rounded = [round(d, -1) for d in ds]
		avail = filter(lambda x: x != -100, [-100 if abs(d) <= 10 else index for index, d in enumerate(rounded)])
		
		### Adding Walls
		unava = filter(lambda x: x != -100, [index if abs(d) <= 10 else -100 for index, d in enumerate(rounded)])

		for direction in unava:
			pos = self.dir_to_pos(direction)

			if pos not in self.knowledge_map["Wall"]:
				self.knowledge_map['Wall'].append(pos)
		###############

		sorted_avail = sorted([(d, ds[d]) for d in avail], 
							cmp=lambda x,y: cmp(x[1], y[1]), reverse=True)

		for (direction, distance) in sorted_avail:
			pos = self.dir_to_pos(direction)
			if pos not in self.knowledge_map["Visited"]:
				return direction

		return ds.index(max([ds[d] for d in avail]))


	def dir_to_pos(self, direction):
		if direction == 0:
			pos = (self.pos[0] + 1, self.pos[1])
		elif direction == 1:
			pos = (self.pos[0], self.pos[1] + 1)
		elif direction == 2:
			pos = (self.pos[0] - 1, self.pos[1])
		else:
			pos = (self.pos[0], self.pos[1] - 1)

		return pos

	"""
		x_data should have the form List[(x1, x2, x3, ..., xn), (x1', x2', x3', ..., xn'), ...] if there are n sensors
		Same as y_data
		u_data should have the same length, but each of which should be a matrix
		---------------------------------
		return value: List[(m1, m2, m3, ..., mn), (m1', m2', m3', ..., mn'), ...]
	"""
	def process_data(self, x_data, y_data, u_data):
		if len(x_data[0]) != len(self.x_kalmans):
			raise Exception ("The number of data is not the same as the number of sensors")

		if len(x_data) != len(y_data) or len(x_data) != len(u_data):
			raise Exception ("The length of x_data, y_data, u_data are not same")

		if not isinstance(u_data[0][0], np.matrix):
			raise Exception ("u_data should be matrix")

		# Transfer measure data to matrix with velocity
		x_data_m = [ tuple(np.matrix([[x], [self.velocity * math.cos(self.direction)]]) for index, x in enumerate(x_d)) 
						for x_d in x_data]
		y_data_m = [ tuple(np.matrix([[y], [self.velocity * math.sin(self.direction)]]) for index, y in enumerate(y_d)) 
						for y_d in y_data]

		batch_x_data = _batch_from_array_of_tuples(x_data_m)
		batch_y_data = _batch_from_array_of_tuples(y_data_m)
		batch_u_data = _batch_from_array_of_tuples(u_data)

		###### Start Process Data ######
		x_res = []
		y_res = []	

		for index, x_d in enumerate(batch_x_data):
			x_res.append(self.x_kalmans[index].process_data(x_d, batch_u_data[index]))

		for index, y_d in enumerate(batch_y_data):
			y_res.append(self.y_kalmans[index].process_data(y_d, batch_u_data[index]))

		###### Finish Process Data ######
		
		x_res = _batch_to_array_of_tuples(x_res)
		y_res = _batch_to_array_of_tuples(y_res)

		x_measures = [ tuple([x.item(0) for x in x_tuple]) for x_tuple in x_res]
		y_measures = [ tuple([y.item(0) for y in y_tuple]) for y_tuple in y_res]

		# print x_measures, y_measures
		return x_measures, y_measures
