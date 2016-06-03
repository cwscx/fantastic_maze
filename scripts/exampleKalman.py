#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt

from Kalman import *

k = Kalman(np.matrix([0.75]),
		   np.matrix([0]),
		   np.matrix([1]),
		   np.matrix([200]),
		   np.matrix([1]))

observed_data = [994, 655, 594, 563, 305, 57, 343, 216, 41, -62]

res = k.update_with_data([np.matrix([d]) for d in observed_data], [np.matrix([0]) for i in xrange(10)])

res = [r.item(0) for r in res]

plt.plot(res)
plt.show()
