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




def main():
    ibvs = ibvs_run()
    robot_startup()
    
    while True:
        # 获取相机帧
        color, gray = ibvs.set_camera()
        if gray is None:
            continue

     
        # AprilTag 检测
        u, v, Z_center, t = ibvs.detect_tag(gray, color)
        

        
        # print(t_cam)
        
        time.sleep(0.05)

        if t is not None:

            print(t)

            t_cam = np.array([t[0], t[1], t[2], 1.0]).reshape(4, 1)
            t_base = bTc @ t_cam

            print("Tag 在基座坐标系下的位置 (米):", t_base[:3].flatten())
            print(t_base[0])

            # q = ibvs.get_jointstate()
            # print(q)
            
            # ibvs.move_robotic(x=t_base[0], y=t_base[1], z=t_base[2])


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






