#!/usr/bin/env python
import numpy as np
from Kalman import *

res = []

k = Kalman(np.matrix([0.75]),
		   np.matrix([0]),
		   np.matrix([1]),
		   np.matrix([200]),
		   np.matrix([1]),
		   np.matrix([994]))

k.predict(np.matrix([0]))
k.update(np.matrix([655]))
res.append(k.x)

k.predict(np.matrix([0]))
k.update(np.matrix([594]))
res.append(k.x)

k.predict(np.matrix([0]))
k.update(np.matrix([563]))
res.append(k.x)

k.predict(np.matrix([0]))
k.update(np.matrix([305]))
res.append(k.x)

k.predict(np.matrix([0]))
k.update(np.matrix([57]))
res.append(k.x)

k.predict(np.matrix([0]))
k.update(np.matrix([343]))
res.append(k.x)

k.predict(np.matrix([0]))
k.update(np.matrix([216]))
res.append(k.x)

k.predict(np.matrix([0]))
k.update(np.matrix([41]))
res.append(k.x)

k.predict(np.matrix([0]))
k.update(np.matrix([-62]))
res.append(k.x)

print res