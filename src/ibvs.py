import numpy as np
import modern_robotics as mr




Slist = np.array([[0.0, 0.0, 1.0,  0.0,     0.0,     0.0],
                  [0.0, 1.0, 0.0, -0.12705, 0.0,     0.0],
                  [0.0, 1.0, 0.0, -0.42705, 0.0,     0.05955],
                  [1.0, 0.0, 0.0,  0.0,     0.42705, 0.0],
                  [0.0, 1.0, 0.0, -0.42705, 0.0,     0.35955],
                  [1.0, 0.0, 0.0,  0.0,     0.42705, 0.0]]).T


Blist = np.array([[0.0, 0.0, 1.0, 0.0, 0.43, 0.0],
                  [0.0, 1.0, 0.0, 0.3, 0.0, -0.43],
                  [0.0, 1.0, 0.0, 0.0, 0.0, -0.37],
                  [1.0, 0.0, 0.0, 0.0, 0.0,  0.0],
                  [0.0, 1.0, 0.0, 0.0, 0.0, -0.07],
                  [1.0, 0.0, 0.0, 0.0, 0.0,  0.0]]).T



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



# 相机和基座，标定而来
def base_to_camera(cTb):
    cVb = mr.Adjoint(cTb)
    return cVb



# 末端和基座,夹子悬空的坐标为末端
def end_to_base(bTe):
    bVe = mr.Adjoint(bTe)
    return bVe



# 末端雅可比矩阵
def get_body_jacobian(Slist, theta, bTs):
    Js = mr.JacobianSpace(Slist, theta)
    eJe = mr.Adjoint(bTs) @ Js
    return eJe



def get_space_jacobian(Slist, theta):
    Js = mr.JacobianSpace(Slist, theta)
    return Js

    