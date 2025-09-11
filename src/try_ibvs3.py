import cv2
from ibvs_run import ibvs_run   # 假设你的类文件名是 ibvs.py
import numpy as np
import time
from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup
import ibvs



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

    prev_time = time.time()   # 初始化时间
    
    while True:

        now = time.time()
        dt = now - prev_time
        prev_time = now

        # 获取相机帧
        color, gray = ibvs.set_camera()
        if gray is None:
            continue


        [cx_img, cy_img] = ibvs.draw_reference(color)
        # print([cx_img, cy_img])

        q_current = ibvs.get_jointstate()
        # print(q)
     
        # AprilTag 检测
        cX, cY, Z_center, t = ibvs.detect_tag(gray, color)
        
        
        
        # time.sleep(0.05)

        if cX is not None and cY is not None and Z_center is not None:
            # print(cX, cY, Z_center)
            print(t)
            # print(q_current)

            error_u = ibvs._lambda*(cX - cx_img)   
            error_v = ibvs._lambda*(cY - cy_img)
            error_Z = -ibvs._lambda*(Z_center - 0.15)


            # print(error_u, error_v, error_Z)

            dq = ibvs.control_law(q_current, error_u, error_v, error_Z, bRc, Slist)
            # print(dq.shape)
            # print(dq)
            # print(dt)


            q_update = q_current + dq * dt  # s
            # print(q_update)


            # ibvs.move_robotic(q_update)


        else:
            print("未检测到 Tag")
            # ibvs.move_robotic([0, 0, 0, 0, 0, 0], max_vel=0.0)
            
        # 显示结果
        cv2.imshow("AprilTag + Reference", color)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
