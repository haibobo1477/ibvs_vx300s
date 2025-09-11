
import numpy as np



v_b = np.array([1,2,3,4,5,6])
v_b = np.hstack((v_b[3:], v_b[:3]))
print(v_b)