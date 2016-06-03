import numpy as np


class Kalman:

	"""
		x_k = A * x_(k-1) + B * u_k
		z_k = C * x_k + noise
		A: the linear coefficient matrix for the model measurement x
		B: the linear coefficient matrix for (possible) other data u, can be zero matrix if none
		C: the linear coefficient matrix for observed data and measurement
		R: the varaiance of the noise signal to represent the accuracy of sensor (which should be a constant)
		P: the covariance of the estimation process at step k. The average of the squared error of predictions
		---------------------------------------
		A: n by n 	x: n by 1	z: n by 1
		B: n by n 	u: n by 1
		C: n by n
		R: n by n
		P: n by n
	"""
	def __init__(self, A, B, C, R, P):
		# Check type of A,B,C,R,P
		if (not isinstance(A, np.matrix)) or (not isinstance(B, np.matrix)) or \
		   (not isinstance(C, np.matrix)) or (not isinstance(R, np.matrix)) or \
		   (not isinstance(P, np.matrix)):
			raise Exception("Initialization failed! All parameters should have type: numpy.matrix")

		# Check shape of matrix
		if A.shape[0] != A.shape[1] or A.shape != B.shape or A.shape != C.shape or \
		   A.shape != R.shape or A.shape != P.shape:
		   	raise Exception("A | B | C | P | R should all have shape n * n")

		self.A = A
		self.B = B
		self.C = C
		self.R = R
		self.P = P


	"""
		u: Other observed data which has a linear relationship with model. Can be zero matrix if none.
		x_k = A * x_(k-1) + B * u_k
		P_k = A * P_(k-1) * A_T
	"""
	def predict(self, u):
		# If the Kalman hasn't receive any data yet, just do nothing
		if hasattr(self, 'x'):
			self.x = np.dot(self.A, self.x) + np.dot(self.B, u)
			self.P = np.dot(np.dot(self.A, self.P), self.A.T)

	"""
		G_k = P_k * C_T * (C * P_k * C_T + R)^-1
		x_k = x_k + G_k * (z_k - C * x_k)
		P_k = (I - G_k * C) * P_k
	"""
	def update(self, z):
		if hasattr(self, 'x'):
			G = np.dot(np.dot(self.P, self.C.T), (np.dot(np.dot(self.C, self.P), self.C.T) + self.R).I)
			self.x = self.x + np.dot(G, (z - np.dot(self.C, self.x)))
			self.P = np.dot((np.identity(len(self.P)) - np.dot(G, self.C)), self.P)
		# If Kalman hasn't received any data yet, just set the estimatino to the observed data
		else:
			self.x = z


	def update_with_data(self, x_data, u_data):
		if len(x_data) != len(u_data) or len(x_data) == 0:
			raise Exception ("Invalid Size")

		if not isinstance(x_data[0], np.matrix) or not isinstance(u_data[0], np.matrix):
			raise Exception ("The element of data should have type numpy.matrix")

		res = list()

		for index, x_d in enumerate(x_data):
			self.predict(u_data[index])
			self.update(x_d)
			res.append(self.x)

		return res