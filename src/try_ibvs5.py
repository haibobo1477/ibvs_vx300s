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
    x_prev_base, y_prev_base, z_prev_base = None, None, None
    x_prev_camera, y_prev_camera, z_prev_camera = None, None, None

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
            u, v, Z_center, t = ibvs.detect_tag(gray, color)

            time.sleep(0.05)

            if t is not None:
                x_c = (u - ibvs.cx) / ibvs.fx
                y_c = (v - ibvs.cy) / ibvs.fy

                L = ibvs.L_point(x_c, y_c, Z_center)

                # print(t)

                p_c = np.array([t[0,0], t[1,0], t[2,0], 1.0])   # 3x1

                p_b = bTc @ p_c   # 3x1

                x_current_base = p_b[0]
                y_current_base = p_b[1]
                z_current_base = p_b[2]


                x_current_camera = p_c[0]
                y_current_camera = p_c[1]
                z_current_camera = p_c[2]


                # print(x_current, y_current, z_current)

                if x_prev_base is not None and y_prev_base is not None:
                    vx_base = (x_current_base - x_prev_base) / dt
                    vy_base = (y_current_base - y_prev_base) / dt
                    vz_base = (z_current_base - z_prev_base) / dt

                    vx_camera = (x_current_camera - x_prev_camera) / dt
                    vy_camera = (y_current_camera - y_prev_camera) / dt
                    vz_camera = (z_current_camera - z_prev_camera) / dt

                    # 因为你不考虑旋转，y速度设为 0
                    wx, wy, wz = 0.0, 0.0, 0.0
 
                    
                    v_tag_camera = np.array([vx_camera, vy_camera, vz_camera, wx, wy, wz])

                    v_tag_base = np.array([vx_base, vy_base, vz_base, wx, wy, wz])
                    v_tag_mr =  np.array([wx, wy, wz, vx_base, vy_base, vz_base])

                    s_dot = L @ v_tag_camera.T

                    u_update = u + dt*s_dot[0]
                    v_update = v + dt*s_dot[1]
                    # print(v)
                    # print(u_update, v_update)

                    error = np.array([u_update - cx_img, v_update - cy_img])
                    print(error)

                  
                    
                    # 保存到日志
                    # v_log.append([vx, vy, vz, wx, wy, wz])
                    # print(v_log)
                    # print(vx, vy, vz)
            
                x_prev_base, y_prev_base, z_prev_base = x_current_base, y_current_base, z_current_base
                x_prev_camera, y_prev_camera, z_prev_camera = x_current_camera, y_current_camera, z_current_camera

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
    


# if len(v_log) == 0:
        #     print("⚠ 没有记录到速度数据，无法绘图")
        #     return

        # v_log = np.array(v_log)
        # t_axis = np.arange(len(v_log)) * 0.05

        # # === 绘制曲线 ===
        # plt.figure()
        # plt.plot(t_axis, v_log[:,0], label="vx")
        # plt.plot(t_axis, v_log[:,1], label="vy")
        # plt.plot(t_axis, v_log[:,2], label="vz")
        # plt.xlabel("Time [s]")
        # plt.ylabel("Linear velocity")
        # plt.legend()
        # plt.title("Linear Velocity (vx, vy, vz)")
        # plt.grid(True)

        # plt.figure()
        # plt.plot(t_axis, v_log[:,3], label="wx")
        # plt.plot(t_axis, v_log[:,4], label="wy")
        # plt.plot(t_axis, v_log[:,5], label="wz")
        # plt.xlabel("Time [s]")
        # plt.ylabel("Angular velocity")
        # plt.legend()
        # plt.title("Angular Velocity (wx, wy, wz)")
        # plt.grid(True)

        # plt.show()