import numpy as np
import math

from Kalman import *

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


	"""
		So the sensor here is fixed to the angles, not related to the robot's directions, because
		in Kalman Model, matrix 'A' need to be fixed.
	"""
	def _init_sensor(self, sensor_num, time_diff):
		self.thetas = [i * (2 * math.pi / sensor_num) for i in xrange(sensor_num)]
		self.x_As = [np.matrix([ [1, math.cos(self.thetas[i]) * time_diff], [0, 1] ]) for i in xrange(sensor_num)] 
		self.y_As = [np.matrix([ [1, math.sin(self.thetas[i]) * time_diff], [0, 1] ]) for i in xrange(sensor_num)]

		self.B = np.matrix(np.zeros([2, 2]))
		self.C = np.matrix([[1, 0], [0, 1]])

		self.R = np.matrix([[100, 0], [0, 100]])
		self.P = np.matrix([[1, 0], [0, 1]])

		self.x_kalmans = [Kalman(A, self.B, self.C, self.R, self.P) for A in self.x_As]
		self.y_kalmans = [Kalman(A, self.B, self.C, self.R, self.P) for A in self.y_As]


	"""
		x_data should have the form List[(x1, x2, x3, ..., xn), (x1', x2', x3', ..., xn'), ...] if there are n sensors
		Same as y_data
		u_data should have the same length, but each of which should be a matrix
	"""
	def process_data(self, x_data, y_data, u_data):
		if len(x_data[0]) != len(self.x_kalmans):
			raise Exception ("The number of data is not the same as the number of sensors")

		if len(x_data) != len(y_data) or len(x_data) != len(u_data):
			raise Exception ("The length of x_data, y_data, u_data are not same")

		if not isinstance(u_data[0][0], np.matrix):
			raise Exception ("u_data should be matrix")

		x_data_m = [ tuple(np.matrix([[x], [self.velocity * math.cos(self.direction)]]) for index, x in enumerate(x_d)) 
						for x_d in x_data]
		y_data_m = [ tuple(np.matrix([[y], [self.velocity * math.sin(self.direction)]]) for index, y in enumerate(y_d)) 
						for y_d in y_data]
		
		batch_x_data = []
		batch_y_data = []
	
		"""
		Same as
		for i in xrange(len(x_data_m[0])):
			l = list()
			for j in xrange(len(x_data_m)):
				l.append(x_data_m[j][i])
			bath_x_data.append(l)
		"""
		batch_x_data = [[x_data_m[j][i] for j in xrange(len(x_data_m))] for i in xrange(len(x_data_m[0]))]
		batch_y_data = [[y_data_m[j][i] for j in xrange(len(y_data_m))] for i in xrange(len(y_data_m[0]))]
		batch_u_data = [[u_data[j][i]   for j in xrange(len(u_data))]   for i in xrange(len(u_data[0]))]

		x_res = []
		y_res = []	

		for index, x_d in enumerate(batch_x_data):
			x_res[len(x_res):] = self.x_kalmans[index].process_data(x_d, batch_u_data[index])

		for index, y_d in enumerate(batch_y_data):
			y_res[len(y_res):] = self.y_kalmans[index].process_data(y_d, batch_u_data[index])

		print x_res
		print y_res

		return x_res, y_res

	