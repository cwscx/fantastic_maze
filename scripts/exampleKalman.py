#!/usr/bin/env python
import numpy as np
import math
import matplotlib.pyplot as plt

from Kalman import *
from KalmanAgents import *

k = Kalman(np.matrix([0.75]),
		   np.matrix([0]),
		   np.matrix([1]),
		   np.matrix([200]),
		   np.matrix([1]))

observed_data = [994, 655, 594, 563, 305, 57, 343, 216, 41, -62]
x_data = [np.matrix([d]) for d in observed_data]
u_data = [np.matrix([0]) for i in xrange(10)]

res = k.process_data(x_data, u_data)

res = [r.item(0) for r in res]

# plt.plot(res)
# plt.show()

ka = simpleKalmanAgent(2, (0,0), 0, 4, 1)

x_data = [(1,0,13,0)]
y_data = [(0,2,0,4)]
u_data = [(np.matrix([[0], [0]]), np.matrix([[0], [0]]), np.matrix([[0], [0]]), np.matrix([[0], [0]]))]

ka.process_data(x_data, y_data, u_data)
