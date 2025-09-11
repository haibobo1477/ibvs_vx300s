import cv2
from ibvs_run import ibvs_run   # 假设你的类文件名是 ibvs.py
import numpy as np
import time
from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup

import matplotlib.pyplot as plt




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
    x_prev, y_prev = None, None

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
            q_current = ibvs.get_jointstate()
            u, v, Z_center, t = ibvs.detect_tag(gray, color)

            time.sleep(0.05)

            if t is not None:
                x_current = (u - ibvs.cx) / ibvs.fx
                y_current = (v - ibvs.cy) / ibvs.fy

                if x_prev is not None and y_prev is not None:
                    dx = (x_current - x_prev) / dt
                    dy = (y_current - y_prev) / dt

                    L = ibvs.L_point(x_current, y_current, Z_center)
                    s_dot = np.array([[dx],[dy]])
                    v = np.linalg.pinv(L) @ s_dot

                    # v 是 (6,1)，前3是线速度，后3是角速度
                    linear_c = v[0:3].reshape(3,1)   # 相机坐标系下线速度
                    angular_c = v[3:6].reshape(3,1)  # 相机坐标系下角速度

                    linear_b = bRc @ linear_c
                    angular_b = bRc @ angular_c

                    v_base = np.vstack((linear_b, angular_b))   # (6,1)，基座系下速度

                    print("tag速度 v:\n", v)
                    v_log.append(v_base.flatten())

                x_prev, y_prev = x_current, y_current
            else:
                print("未检测到 Tag")

            cv2.imshow("AprilTag + Reference", color)
            cv2.waitKey(1)   # 仅保证窗口刷新，不用 q 来退出

    except KeyboardInterrupt:
        print("\n检测到 Ctrl+C,准备退出程序...")

    finally:
        cv2.destroyAllWindows()

        if len(v_log) == 0:
            print("⚠ 没有记录到速度数据，无法绘图")
            return

        v_log = np.array(v_log)
        t_axis = np.arange(len(v_log)) * 0.05

        # === 绘制曲线 ===
        plt.figure()
        plt.plot(t_axis, v_log[:,0], label="vx")
        plt.plot(t_axis, v_log[:,1], label="vy")
        plt.plot(t_axis, v_log[:,2], label="vz")
        plt.xlabel("Time [s]")
        plt.ylabel("Linear velocity")
        plt.legend()
        plt.title("Linear Velocity (vx, vy, vz)")
        plt.grid(True)

        plt.figure()
        plt.plot(t_axis, v_log[:,3], label="wx")
        plt.plot(t_axis, v_log[:,4], label="wy")
        plt.plot(t_axis, v_log[:,5], label="wz")
        plt.xlabel("Time [s]")
        plt.ylabel("Angular velocity")
        plt.legend()
        plt.title("Angular Velocity (wx, wy, wz)")
        plt.grid(True)

        plt.show()




# def main():
#     ibvs = ibvs_run()
#     robot_startup()

#     prev_time = time.time()   # 初始化时间
#     x_prev, y_prev = None, None   # 初始化上一帧的归一化坐标
    
#     v_log = []   # <<< 在这里初始化

    

#     while True:

#         now = time.time()
#         dt = now - prev_time
#         prev_time = now

#         # 获取相机帧
#         color, gray = ibvs.set_camera()
#         if gray is None:
#             continue


#         [cx_img, cy_img] = ibvs.draw_reference(color)
#         # print([cx_img, cy_img])

#         q_current = ibvs.get_jointstate()
#         # print(q)
     
#         # AprilTag 检测
#         u, v, Z_center, t = ibvs.detect_tag(gray, color)

        
#         time.sleep(0.05)

#         if t is not None:

#             x_current = (u - ibvs.cx) / ibvs.fx
#             y_current = (v - ibvs.cy) / ibvs.fy


#             if x_prev is not None and y_prev is not None:
#                 dx = (x_current - x_prev) / dt
#                 dy = (y_current - y_prev) / dt

#                 L = ibvs.L_point(x_current, y_current, Z_center)

#                 s_dot = np.array([[dx],[dy]])

#                 v = np.linalg.pinv(L) @ s_dot

#                 print("tag速度 v:\n", v)

#                 v_log.append(v.flatten())   # <<< 保存一帧速度


#             x_prev, y_prev = x_current, y_current



#             # ibvs.move_robotic(q_update)


#         else:
#             print("未检测到 Tag")
#             # ibvs.move_robotic([0, 0, 0, 0, 0, 0], max_vel=0.0)
            
#         # 显示结果
#         cv2.imshow("AprilTag + Reference", color)
#         if cv2.waitKey(10) & 0xFF == ord("q"): 
#             break
        
#     cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
    
