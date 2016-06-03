import numpy as np
import math

from Kalman import *

class simpleKalmanAgent():
	def __init__(self, sensor_num, time_diff):
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
	def process_data(self, x_data, y_data, u_data):
		if len(x_data[0]) != len(self.x_kalmans):
			raise Exception ("The number of data is not the same as the number of sensors")

		x_res = [x_k.process_data(x_data, u_data) for x_k in self.x_kalmans]
		y_res = [y_k.process_data(y_data, u_data) for y_k in self.y_kalmans]

		return x_res, y_res
	"""