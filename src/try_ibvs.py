# import time
# from ibvs_run import ibvs_run
# from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup

# def main():
#     ibvs = ibvs_run()
#     robot_startup()

#     try:
#         while True:
#             q, ee = ibvs.get_jointstate()
#             print("关节角度:", q)
#             print("末端位姿:", ee)

#             time.sleep(0.1)  # 每 0.1 秒刷新一次

#     except KeyboardInterrupt:
#         print("用户中断，退出程序")

# if __name__ == "__main__":
#     main()




import cv2
from ibvs_run import ibvs_run   # 假设你的类文件名是 ibvs.py
import numpy as np
import time
from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup
import ibvs


Blist = np.array([[0.0, 0.0, 1.0, 0.0, 0.5370, 0.0],
                  [0.0, 1.0, 0.0, 0.3, 0.0,   -0.5370],
                  [0.0, 1.0, 0.0, 0.0, 0.0,   -0.4770],
                  [1.0, 0.0, 0.0, 0.0, 0.0,    0.0],
                  [0.0, 1.0, 0.0, 0.0, 0.0,   -0.1770],
                  [1.0, 0.0, 0.0, 0.0, 0.0,    0.0]]).T


Slist = np.array([[0.0, 0.0, 1.0,  0.0,     0.0,     0.0],
                  [0.0, 1.0, 0.0, -0.12705, 0.0,     0.0],
                  [0.0, 1.0, 0.0, -0.42705, 0.0,     0.05955],
                  [1.0, 0.0, 0.0,  0.0,     0.42705, 0.0],
                  [0.0, 1.0, 0.0, -0.42705, 0.0,     0.35955],
                  [1.0, 0.0, 0.0,  0.0,     0.42705, 0.0]]).T



cTb = np.array([
    [-0.116, 0.993, 0.004, 0.136],
    [ 0.785, 0.094,-0.613,-0.293],
    [-0.609,-0.068,-0.790, 0.860],
    [ 0.0,   0.0,   0.0,   1.0],
])

# Z_star = [0.102, 0.102, 0.102, 0.102]
Z_star = 0.202

def main():
    ibvs = ibvs_run()
    robot_startup()
    
    while True:
        # 获取相机帧
        color, gray = ibvs.set_camera()
        if gray is None:
            continue

        # 画中心正方形参考点
        # square_pts = ibvs.draw_reference(color)     # 对    (4,2)
        [cx_img, cy_img] = ibvs.draw_reference(color)

        # print(square_pts.shape)
        # print(square_pts)

        x_ref = (cx_img - ibvs.cx) / ibvs.fx    # 对   (4,1)
        y_ref = (cy_img - ibvs.cy) / ibvs.fy    # 对   (4,1)

        # print(x_ref)
        # print(y_ref)
    

        # AprilTag 检测
        # pts, Z_corners, x, y = ibvs.detect_tag(gray, color)  # pts->(4,2)   Z_corners->(4,1)  x->(4,1) y->(4,1)
        x, y, Z = ibvs.detect_tag(gray, color)
        
        time.sleep(0.05)

        if x is not None and y is not None:

            # square_pts = np.array(square_pts, dtype=np.float32)
            # pts = np.array(pts, dtype=np.float32)

            
            # print(y.shape)
            # print(y)
            # error = pts - square_pts

            error = np.column_stack([x - x_ref, y - y_ref]).reshape(-1,1)
            # print(square_pts.flatten())
            # print(pts.flatten())
            # print(f"检测到 Tag, 距离 Z = {Z_corners} m, 误差为 e = {error}" )
            # print(error.shape)
            # print(error)
            q = ibvs.get_jointstate()
            
            # print("x is:", x)
            # print("y is:", y)

            # print(q)
            # print(ee)
            

            dq = ibvs.control_law(q, error, x, y, Z, cTb, Slist, lam=2.0)
            # dq = ibvs.control_law(q, error, x, y, Z_corners, cTb, Slist, lam=0.01)
            # dq = ibvs.control_law(q, error, x_ref, y_ref, Z_star, cTb, Slist, lam=0.2)
            # print(dq.shape)
            print(dq)
 

            ibvs.move_robotic(dq, max_vel=0.3)


        else:
            print("未检测到 Tag")
            ibvs.move_robotic([0, 0, 0, 0, 0, 0], max_vel=0.0)
            
        # 显示结果
        cv2.imshow("AprilTag + Reference", color)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()






# import cv2
# from ibvs_run import ibvs_run

# def main():
#     ibvs = ibvs_run()

#     while True:
#         # 获取相机帧
#         color, gray = ibvs.set_camera()
#         if gray is None:
#             # 没有拿到新帧，直接跳过这轮
#             continue

#         # AprilTag 检测
#         pts, Z = ibvs.detect_tag(gray, color)

#         if pts is not None:
#             print(f"检测到 Tag, 距离 Z = {Z:.3f} m")
#         else:
#             print("未检测到 Tag")

#         # 显示结果（无论有没有检测到，都显示相机画面）
#         cv2.imshow("AprilTag Detection", color)
#         if cv2.waitKey(1) & 0xFF == ord("q"):
#             break

#     cv2.destroyAllWindows()

# if __name__ == "__main__":
#     main()