import numpy as np
import modern_robotics as mr




Slist = np.array([[0.0, 0.0, 1.0,  0.0,     0.0,     0.0],
                  [0.0, 1.0, 0.0, -0.12705, 0.0,     0.0],
                  [0.0, 1.0, 0.0, -0.42705, 0.0,     0.05955],
                  [1.0, 0.0, 0.0,  0.0,     0.42705, 0.0],
                  [0.0, 1.0, 0.0, -0.42705, 0.0,     0.35955],
                  [1.0, 0.0, 0.0,  0.0,     0.42705, 0.0]]).T



def L_point(x,y,Z):
    L = np.array([[-1/Z, 0, x/Z, x*y, -(1+x*x), y],
                  [0, -1/Z, y/Z, 1+y*y, -x*y, -x]])
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



    