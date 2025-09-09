
import numpy as np


# 假设检测到的点 (x,y)
x = np.array([10, 20, 30, 40])
y = np.array([50, 60, 70, 80])

# 假设参考点 (x_ref,y_ref)
x_ref = np.array([1, 2, 3, 4])
y_ref = np.array([5, 6, 7, 8])

# 计算误差
error = np.column_stack([x - x_ref, y - y_ref]).reshape(-1,1)

print("误差向量 error = \n", error)