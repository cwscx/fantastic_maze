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
		z: First observation data
	"""
	def __init__(self, A, B, C, R, P, z):
		if (not isinstance(A, np.matrix)) or (not isinstance(B, np.matrix)) or \
		   (not isinstance(C, np.matrix)) or (not isinstance(R, np.matrix)) or \
		   (not isinstance(P, np.matrix)) or (not isinstance(z, np.matrix)):
			raise Exception("Initialization failed! All parameters should have type: numpy.matrix")

		self.A = A
		self.B = B
		self.C = C
		self.R = R
		self.P = P
		self.x = z


	"""
		u: Other observed data which has a linear relationship with model. Can be zero matrix if none.
		x_k = A * x_(k-1) + B * u_k
		P_k = A * P_(k-1) * A_T
	"""
	def predict(self, u):
		self.x = np.dot(self.A, self.x) + np.dot(self.B, u)
		self.P = np.dot(np.dot(self.A, self.P), self.A.T)

	"""
		G_k = P_k * C_T * (C * P_k * C_T + R)^-1
		x_k = x_k + G_k * (z_k - C * x_k)
		P_k = (I - G_k * C) * P_k
	"""
	def update(self, z):
		G = np.dot(np.dot(self.P, self.C.T), (np.dot(np.dot(self.C, self.P), self.C.T) + self.R).I)
		self.x = self.x + np.dot(G, (z - np.dot(self.C, self.x)))
		self.P = np.dot((np.identity(len(self.P)) - np.dot(G, self.C)), self.P)
