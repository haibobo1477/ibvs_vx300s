import cv2
from ibvs_run import ibvs_run   # 假设你的类文件名是 ibvs.py
import numpy as np
import time
from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup

import matplotlib.pyplot as plt
import modern_robotics as mr




Slist = np.array([[0.0, 0.0, 1.0,  0.0,     0.0,     0.0],
                  [0.0, 1.0, 0.0, -0.12705, 0.0,     0.0],
                  [0.0, 1.0, 0.0, -0.42705, 0.0,     0.05955],
                  [1.0, 0.0, 0.0,  0.0,     0.42705, 0.0],
                  [0.0, 1.0, 0.0, -0.42705, 0.0,     0.35955],
                  [1.0, 0.0, 0.0,  0.0,     0.42705, 0.0]]).T


cTb = np.array([
    [-0.040,  0.999, -0.021,  0.060],
    [ 0.792,  0.019, -0.611, -0.301],
    [-0.610, -0.041, -0.792,  0.862],
    [ 0.000,  0.000,  0.000,  1.000]
])


bTc = np.array([
    [-0.040,  0.792, -0.610,  0.766],
    [ 0.999,  0.019, -0.041, -0.019],
    [-0.021, -0.611, -0.792,  0.499],
    [ 0.000,  0.000,  0.000,  1.000]
])

bRc = np.array([
    [-0.040,  0.792, -0.610],
    [ 0.999,  0.019, -0.041],
    [-0.021, -0.611, -0.792]
])


def main():
    ibvs = ibvs_run()
    robot_startup()

    prev_time = time.time()

    v_log = []

    try:
        while True:
            now = time.time()
            dt = now - prev_time
            prev_time = now

            color, gray = ibvs.set_camera()
            if gray is None:
                continue

            [cx_img, cy_img] = ibvs.draw_reference(color)
            # print([cx_img, cy_img])
            q_current, ee = ibvs.get_jointstate()
            q_current = np.array(q_current, dtype=float)

            u, v, Z_center, t = ibvs.detect_tag(gray, color)

            time.sleep(0.05)

            if t is not None:
                x_c = (u - ibvs.cx) / ibvs.fx
                y_c = (v - ibvs.cy) / ibvs.fy

                L = ibvs.L_point(x_c, y_c, Z_center)


                e = np.array([[u - cx_img], [v - cy_img]])     # 2x1


                # 4) IBVS 控制律：相机系的 twist v_c = -λ * pinv(L) * e
                lambda_gain = 0.001  # 先小后大，避免抖动
                L_pinv = np.linalg.pinv(L)
                v_c = lambda_gain * (L_pinv @ e)   # 6x1: [vx,vy,vz, wx,wy,wz] (camera frame)
                # print(v_c)

                # 5) 限幅（很重要，防止突变）

                # 6) 相机系 -> base 系 twist 变换：v_b = Ad_{bTc} * v_c
                Ad_bTc = mr.Adjoint(bTc)   # 6x6
                # print(Ad_bTc)
                v_b = Ad_bTc @ v_c         # 6x1

                v_b = np.vstack((v_b[3:], v_b[:3]))
                # print(v_b)
                # print(q_current.shape)

                Js = mr.JacobianSpace(Slist, q_current)  # 6xn
                Js_pinv = np.linalg.pinv(Js)

                dq = Js_pinv @ v_b
                dq = dq.reshape(-1)
                # print(dq)
                # print(Js)
                # print(q_current)

                q_next = q_current + dq * dt
                # print(dt)
                print("q_next:", q_next)

                ibvs.move_robotic(q_next)


                    
                    # 保存到日志
                    # v_log.append([vx, vy, vz, wx, wy, wz])
                    # print(v_log)
                    # print(vx, vy, vz)

            else:
                print("未检测到 Tag")

            cv2.imshow("AprilTag + Reference", color)
            cv2.waitKey(1)   # 仅保证窗口刷新，不用 q 来退出

    except KeyboardInterrupt:
        print("\n检测到 Ctrl+C,准备退出程序...")

    finally:
        cv2.destroyAllWindows()

        

if __name__ == "__main__":
    main()
    

