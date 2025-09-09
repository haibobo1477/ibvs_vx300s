import numpy as np



def L_point(x, y, Z):
    """
    输入:
        x, y, Z : numpy 数组 (长度 = 特征点数, 通常是4个角点)
    输出:
        L : (2N x 6) 交互矩阵
    """
    N = len(x)
    L = np.zeros((2*N, 6))

    for i in range(N):
        xi, yi, Zi = x[i], y[i], Z[i]
        L[2*i:2*i+2, :] = np.array([
            [-1/Zi,      0, xi/Zi,  xi*yi, -(1+xi*xi),  yi],
            [0,     -1/Zi, yi/Zi,  1+yi*yi,   -xi*yi, -xi]
        ])

    return L


# 假设检测到的点 (x,y)
x = np.array([10, 20, 30, 40])
y = np.array([50, 60, 70, 80])
Z = np.array([5, 6, 7, 8])



L = L_point(x,y,Z)
print(L[0:2, :])