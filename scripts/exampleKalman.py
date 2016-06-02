#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt

from Kalman import *

res = []

k = Kalman(np.matrix([0.75]),
		   np.matrix([0]),
		   np.matrix([1]),
		   np.matrix([200]),
		   np.matrix([1]),
		   np.matrix([994]))

observed_data = [655, 594, 563, 305, 57, 343, 216, 41, -62]

for d in observed_data:
	k.predict(np.matrix([0]))
	k.update(np.matrix([d]))
	res.append(k.x.item(0))

plt.plot(res)
plt.show()
