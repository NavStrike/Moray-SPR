import numpy as np
from functions.process import equalizeMin
m = equalizeMin(np.array([1,2,4,2,1,0.5,0.1]), np.array([4,5,10,6,5,3,2]))
print(m.newCurve())